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
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "general.h"
    
    #define SetTIM1Duty( val )    TIM1->CCR3 = val
    void initPWMInput();
    void initPWMOutput();


#ifdef	__cplusplus
}
#endif

#endif	/* PWRTRAIN_STEERING_H */

