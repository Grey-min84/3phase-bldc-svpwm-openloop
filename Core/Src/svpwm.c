/**
 * @file    svpwm.c
 * @brief   SVPWM 구현 - 6섹터 스위칭 시간 계산 및 중앙정렬 PWM 출력
 */

#include "svpwm.h"
#include <math.h>


/* 오픈루프 제어 변수 */
volatile float g_angle = 0.0f;           // 현재 전기각 [rad]
volatile float g_omega = 0.0f;           // 목표 각속도 [rad/s]
volatile float g_voltage = 0.0f;         // 출력 전압 크기 [0~1 정규화]

#define CONTROL_FREQ    10000.0f         // 제어 루프 주파수 [Hz]
#define DT              (1.0f / CONTROL_FREQ)



/* 타이머 핸들 */
static TIM_HandleTypeDef *pHTim = NULL;

/* SVPWM 상태 */
static SVPWM_State_t svpwm_state;




/**
 * @brief 오픈루프 속도 설정
 * @param freq_hz   전기 주파수 [Hz] (모터 극쌍수에 따라 기계 속도 결정)
 * @param voltage   전압 크기 [0.0 ~ 1.0]
 */
void OpenLoop_SetSpeed(float freq_hz, float voltage)
{
    g_omega = 2.0f * PI * freq_hz;
    g_voltage = (voltage > 1.0f) ? 1.0f : ((voltage < 0.0f) ? 0.0f : voltage);
}

/**
 * @brief TIM6 인터럽트 콜백 (제어 루프)
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        // 각도 업데이트
        g_angle += g_omega * DT;
        
        // 각도 범위 제한 [0, 2π)
        if (g_angle >= TWO_PI)
            g_angle -= TWO_PI;
        else if (g_angle < 0.0f)
            g_angle += TWO_PI;
        
        // α-β 전압 계산
        float Valpha = g_voltage * cosf(g_angle);
        float Vbeta  = g_voltage * sinf(g_angle);
        
        // SVPWM 실행
        SVPWM_Run(Valpha, Vbeta);
    }
}








/* ============================================================
 * 섹터 판별
 * ============================================================ */
/**
 * @brief Vα, Vβ로부터 섹터 판별 (1~6)
 * 
 *              β축
 *               |
 *        섹터2  |  섹터1
 *          \    |    /
 *           \   |   /
 *            \  |  /
 *   섹터3 -----(0,0)----- 섹터6  → α축
 *            /  |  \
 *           /   |   \
 *          /    |    \
 *        섹터4  |  섹터5
 *               |
 */
static uint8_t SVPWM_GetSector(float Valpha, float Vbeta)
{
    uint8_t sector;
    
    // 3개 기준선 판별
    float Vref1 = Vbeta;
    float Vref2 = (SQRT3_HALF * Valpha) - (0.5f * Vbeta);
    float Vref3 = (-SQRT3_HALF * Valpha) - (0.5f * Vbeta);
    
    uint8_t A = (Vref1 > 0) ? 1 : 0;
    uint8_t B = (Vref2 > 0) ? 1 : 0;
    uint8_t C = (Vref3 > 0) ? 1 : 0;
    
    // 섹터 매핑 테이블
    // N = A + 2B + 4C
    static const uint8_t sector_table[8] = {0, 2, 6, 1, 4, 3, 5, 0};
    sector = sector_table[A + (B << 1) + (C << 2)];
    
    return sector;
}

/* ============================================================
 * 스위칭 시간 계산 (6섹터)
 * ============================================================ */
/**
 * @brief 섹터별 T1, T2, T0 시간 비율 계산
 * 
 * 입력: 정규화된 Vα, Vβ (범위 약 -1 ~ +1)
 * 
 * 각 섹터의 인접 활성벡터:
 *   섹터1: V1(100), V2(110)   0° ~ 60°
 *   섹터2: V2(110), V3(010)  60° ~ 120°
 *   섹터3: V3(010), V4(011) 120° ~ 180°
 *   섹터4: V4(011), V5(001) 180° ~ 240°
 *   섹터5: V5(001), V6(101) 240° ~ 300°
 *   섹터6: V6(101), V1(100) 300° ~ 360°
 */
static void SVPWM_CalcTimes(float Valpha, float Vbeta, SVPWM_State_t *pState)
{
    float T1, T2, T0;
    
    // 섹터 판별
    pState->sector = SVPWM_GetSector(Valpha, Vbeta);
    
    // 공통 중간값 계산
    // X = √3 * Vβ
    // Y = (3/2)*Vα + (√3/2)*Vβ  
    // Z = -(3/2)*Vα + (√3/2)*Vβ
    float X = SQRT3 * Vbeta;
    float Y = (1.5f * Valpha) + (SQRT3_HALF * Vbeta);
    float Z = (-1.5f * Valpha) + (SQRT3_HALF * Vbeta);
    
    // 섹터별 T1, T2 계산
    switch (pState->sector)
    {
        case 1:  // 0° ~ 60°: V1(100) → V2(110)
            T1 = Y;      // T1 ∝ sin(60° - θ)
            T2 = X;      // T2 ∝ sin(θ)
            break;
            
        case 2:  // 60° ~ 120°: V2(110) → V3(010)
            T1 = -Z;     // T1 ∝ sin(120° - θ)
            T2 = Y;      // T2 ∝ sin(θ - 60°)
            break;
            
        case 3:  // 120° ~ 180°: V3(010) → V4(011)
            T1 = X;      // T1 ∝ sin(180° - θ)
            T2 = Z;      // T2 ∝ sin(θ - 120°)
            break;
            
        case 4:  // 180° ~ 240°: V4(011) → V5(001)
            T1 = -Y;     // T1 ∝ sin(240° - θ)
            T2 = -X;     // T2 ∝ sin(θ - 180°)
            break;
            
        case 5:  // 240° ~ 300°: V5(001) → V6(101)
            T1 = Z;      // T1 ∝ sin(300° - θ)
            T2 = -Y;     // T2 ∝ sin(θ - 240°)
            break;
            
        case 6:  // 300° ~ 360°: V6(101) → V1(100)
            T1 = -X;     // T1 ∝ sin(360° - θ)
            T2 = -Z;     // T2 ∝ sin(θ - 300°)
            break;
            
        default:
            T1 = 0;
            T2 = 0;
            break;
    }
    
    // 음수 방지
    if (T1 < 0) T1 = 0;
    if (T2 < 0) T2 = 0;
    
    // 과변조 처리 (T1 + T2 > 1 일 때)
    float Tsum = T1 + T2;
    if (Tsum > 1.0f)
    {
        T1 = T1 / Tsum;
        T2 = T2 / Tsum;
        T0 = 0;
    }
    else
    {
        T0 = 1.0f - Tsum;
    }
    
    pState->T1 = T1;
    pState->T2 = T2;
    pState->T0 = T0;
}

/* ============================================================
 * CCR 값 계산 (중앙정렬 PWM)
 * ============================================================ */
/**
 * @brief 섹터별 CCR 값 계산
 * 
 * 중앙정렬 PWM 대칭 패턴:
 * 
 *     |<-------- Ts -------->|
 *     |                      |
 *     |  T0/2  T1  T2  T0/2  |
 *     | (V0) (Vx)(Vy) (V7)   |
 *     
 * 카운터:  0 → ARR → 0
 *          ↑ CCR 비교로 HIGH/LOW 결정
 * 
 * CCR 값이 클수록 HIGH 구간이 길어짐
 */
static void SVPWM_CalcCCR(SVPWM_State_t *pState)
{
    float Ta, Tb, Tc;  // 각 상의 ON 시간 비율 (0~1)
    
    float T0_half = pState->T0 * 0.5f;
    float T1 = pState->T1;
    float T2 = pState->T2;
    
    /**
     * 섹터별 스위칭 시퀀스 및 ON 시간 계산
     * 
     * 예) 섹터1: 000 → 100 → 110 → 111 → 110 → 100 → 000
     *     - A상: V1, V2, V7에서 HIGH → Ta = T1 + T2 + T0/2
     *     - B상: V2, V7에서 HIGH     → Tb = T2 + T0/2
     *     - C상: V7에서만 HIGH       → Tc = T0/2
     */
    switch (pState->sector)
    {
        case 1:  // 000 → 100 → 110 → 111
            Ta = T1 + T2 + T0_half;
            Tb = T2 + T0_half;
            Tc = T0_half;
            break;
            
        case 2:  // 000 → 010 → 110 → 111
            Ta = T1 + T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T0_half;
            break;
            
        case 3:  // 000 → 010 → 011 → 111
            Ta = T0_half;
            Tb = T1 + T2 + T0_half;
            Tc = T2 + T0_half;
            break;
            
        case 4:  // 000 → 001 → 011 → 111
            Ta = T0_half;
            Tb = T1 + T0_half;
            Tc = T1 + T2 + T0_half;
            break;
            
        case 5:  // 000 → 001 → 101 → 111
            Ta = T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T2 + T0_half;
            break;
            
        case 6:  // 000 → 100 → 101 → 111
            Ta = T1 + T2 + T0_half;
            Tb = T0_half;
            Tc = T1 + T0_half;
            break;
            
        default:
            Ta = Tb = Tc = 0.5f;
            break;
    }
    
    // CCR = ON비율 * (ARR + 1)
    float period_f = (float)(PWM_PERIOD + 1);
    
    pState->CCR_A = (uint16_t)(Ta * period_f);
    pState->CCR_B = (uint16_t)(Tb * period_f);
    pState->CCR_C = (uint16_t)(Tc * period_f);
    
    // 범위 제한
    if (pState->CCR_A > PWM_PERIOD) pState->CCR_A = PWM_PERIOD;
    if (pState->CCR_B > PWM_PERIOD) pState->CCR_B = PWM_PERIOD;
    if (pState->CCR_C > PWM_PERIOD) pState->CCR_C = PWM_PERIOD;
}

/* ============================================================
 * PWM 출력 업데이트
 * ============================================================ */
static void SVPWM_UpdatePWM(SVPWM_State_t *pState)
{
    if (pHTim == NULL) return;
    
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_1, pState->CCR_A);
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_2, pState->CCR_B);
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_3, pState->CCR_C);
}

/* ============================================================
 * Public 함수
 * ============================================================ */

/**
 * @brief SVPWM 초기화
 * @param htim  TIM1 핸들 포인터
 */
void SVPWM_Init(TIM_HandleTypeDef *htim)
{
    pHTim = htim;
    
    // 상태 초기화
    svpwm_state.sector = 1;
    svpwm_state.T1 = 0;
    svpwm_state.T2 = 0;
    svpwm_state.T0 = 1.0f;
    svpwm_state.CCR_A = 0;
    svpwm_state.CCR_B = 0;
    svpwm_state.CCR_C = 0;
    
    // PWM 채널 시작
    HAL_TIM_PWM_Start(pHTim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pHTim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(pHTim, TIM_CHANNEL_3);
    
    // TIM1은 Advanced Timer이므로 MOE 비트 활성화 필요
    //__HAL_TIM_MOE_ENABLE(pHTim);
}

/**
 * @brief SVPWM 실행 (메인 함수)
 * @param Valpha  α축 전압 (정규화: -1 ~ +1)
 * @param Vbeta   β축 전압 (정규화: -1 ~ +1)
 * 
 * 사용 예:
 *   float angle = omega * t;
 *   SVPWM_Run(V * cosf(angle), V * sinf(angle));
 */
void SVPWM_Run(float Valpha, float Vbeta)
{
    // 1. T1, T2, T0 계산 (섹터 판별 포함)
    SVPWM_CalcTimes(Valpha, Vbeta, &svpwm_state);
    
    // 2. CCR 값 계산
    SVPWM_CalcCCR(&svpwm_state);
    
    // 3. PWM 레지스터 업데이트
    SVPWM_UpdatePWM(&svpwm_state);
}

/**
 * @brief SVPWM 정지 (모든 출력 LOW)
 */
void SVPWM_Stop(void)
{
    if (pHTim == NULL) return;
    
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(pHTim, TIM_CHANNEL_3, 0);
}

/**
 * @brief 현재 상태 반환 (디버깅용)
 */
SVPWM_State_t* SVPWM_GetState(void)
{
    return &svpwm_state;
}
