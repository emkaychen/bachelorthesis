/* 
 * File:   accelerometer.h
 * Author: emkay
 *
 * Created on 18. Juli 2013, 07:58
 */

#ifndef ACCELEROMETER_H
#define	ACCELEROMETER_H

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
#include "stm32f4_discovery_lis302dl.h"  




    void initAccelerometer();
    void vAccelerometerTask(void *pvParameters);



#ifdef	__cplusplus
}
#endif

#endif	/* ACCELEROMETER_H */

