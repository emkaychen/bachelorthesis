/* 
 * File:   pwrtrain_steering.h
 * Author: emkay
 *
 * Created on 2. Juli 2013, 08:40
 */

#ifndef PWRTRAIN_STEERING_H
#define	PWRTRAIN_STEERING_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "general.h"


    //Lenkung    
    /** Lenkung nicht eingeschlagen. */
#define STEERING_DIR_NEUTRAL            0       
    /** Lenkung nach links eingeschlagen. */
#define STEERING_DIR_LEFT               1
    /** Lenkung nach rechts eingeschlagen. */
#define STEERING_DIR_RIGHT              2

    /** PWM Signal für die neutrale Lenkstellung. */
#define STEERING_POS_NEUTRAL            3900
    /** PWM Signal für den maximalen Linkseinschlag. */
#define STEERING_POS_LEFT_MAX           3120
    /** PWM Signal für den maximalen rechtseinschlag. */
#define STEERING_POS_RIGHT_MAX          4700

    //Antrieb
    /** Antrieb angehalten. */
#define PWRTRAIN_DIR_STOPPED            0
    /** Antrieb forwärts. */
#define PWRTRAIN_DIR_FORWARDS           1
    /** Antrieb rückwärts. */
#define PWRTRAIN_DIR_BACKWARDS          2

    /** PWM Signal für die neutrale Lenkstellung. */
#define PWRTRAIN_POS_NEUTRAL            3900
    /** PWM Signal für die maximale Forwärtsfahrt. */
#define PWRTRAIN_POS_FORWARDS_MAX       4700
    /** PWM Signal für die maximale Rückwärtsfahrt. */
#define PWRTRAIN_POS_BACKWARDS_MAX      3100

    //Fernsteuerung
    /** Fernsteuerung ausgeschaltet. */
#define REMOTE_INAKTIVE                 0
    /** Fernsteuerung eingeschaltet. */
#define REMOTE_AKTIVE                   1

#define STEERING_REMOTE_MID             1350
#define STEERING_REMOTE_PATCH           20
#define STEERING_REMOTE_LEFT            STEERING_REMOTE_MID - STEERING_REMOTE_PATCH
#define STEERING_REMOTE_RIGHT           STEERING_REMOTE_MID + STEERING_REMOTE_PATCH
#define STEERING_REMOTE_MAX_LEFT        970   
#define STEERING_REMOTE_MAX_RIGHT       1720

#define PWRTRAIN_REMOTE_MID              1300
#define PWRTRAIN_REMOTE_PATCH            20
#define PWRTRAIN_REMOTE_FORWARDS         PWRTRAIN_REMOTE_MID + PWRTRAIN_REMOTE_PATCH
#define PWRTRAIN_REMOTE_BACKWARDS        PWRTRAIN_REMOTE_MID - PWRTRAIN_REMOTE_PATCH
#define PWRTRAIN_REMOTE_MAX_FORWARDS     1710
#define PWRTRAIN_REMOTE_MAX_BACKWARDS    840

#define SetTIM1Duty( val )    TIM1->CCR3 = val
    void initPWMInput();
    void initPWMOutput();

    void vSteeringTask(void *pvParameters);
    void setStreeringValue(uint16_t value);
    static uint8_t getSteeringDir();
    static uint16_t getSteeringVal();

    void vDrivingTask(void *pvParameters);
    void setDrivingValue(uint16_t value);
    static uint8_t getDrivingDir();
    static uint16_t getDrivingVal();

#ifdef	__cplusplus
}
#endif

#endif	/* PWRTRAIN_STEERING_H */

