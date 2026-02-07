/**
 * @file    svpwm.h
 * @brief   SVPWM 헤더 - STM32G431용
 */

#ifndef __SVPWM_H
#define __SVPWM_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ============== 상수 정의 ============== */
#define PI              3.14159265f
#define TWO_PI          6.28318530f
#define SQRT3           1.7320508f
#define SQRT3_HALF      0.8660254f     // √3/2
#define SQRT3_INV       0.57735027f    // 1/√3

/* PWM 설정 */
#define PWM_PERIOD      8499           // ARR 

/* ============== 타입 정의 ============== */
typedef struct {
    uint8_t  sector;    // 현재 섹터 (1~6)
    float    T1;        // 첫 번째 활성벡터 시간 비율
    float    T2;        // 두 번째 활성벡터 시간 비율
    float    T0;        // 영벡터 시간 비율
    uint16_t CCR_A;     // CH1 (A상) 비교값
    uint16_t CCR_B;     // CH2 (B상) 비교값
    uint16_t CCR_C;     // CH3 (C상) 비교값
} SVPWM_State_t;

/* ============== 함수 선언 ============== */

/**
 * @brief 현재 상태 반환 (디버깅용)
 */
SVPWM_State_t* SVPWM_GetState(void);




/**
 * @brief SVPWM 초기화 및 PWM 출력 시작
 * @param htim  TIM1 핸들 포인터
 */
void SVPWM_Init(TIM_HandleTypeDef *htim);

/**
 * @brief SVPWM 실행
 * @param Valpha  α축 전압 (정규화: -1 ~ +1)
 * @param Vbeta   β축 전압 (정규화: -1 ~ +1)
 */
void SVPWM_Run(float Valpha, float Vbeta);

/**
 * @brief SVPWM 정지
 */
void SVPWM_Stop(void);



/**
 * @brief 오픈루프 속도 설정
 * @param freq_hz   전기 주파수 [Hz] (모터 극쌍수에 따라 기계 속도 결정)
 * @param voltage   전압 크기 [0.0 ~ 1.0]
 */
void OpenLoop_SetSpeed(float freq_hz, float voltage);




#endif /* __SVPWM_H */
