/* 
 * File:   communication.h
 * Author: emkay
 *
 * Created on 4. Juli 2013, 07:59
 */

#ifndef COMMUNICATION_H
#define	COMMUNICATION_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "stm32f4xx_usart.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4_discovery.h"

    void initSerial();



#ifdef	__cplusplus
}
#endif

#endif	/* COMMUNICATION_H */

