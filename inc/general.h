/* 
 * File:   general.h
 * Author: emkay
 *
 * Created on 4. Juli 2013, 07:51
 */

#ifndef GENERAL_H
#define	GENERAL_H

#ifdef	__cplusplus
extern "C" {
#endif


#define DEBUG 1
#define debug_print2(fmt, ...) \
        do { if (DEBUG) printf("%s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__, __VA_ARGS__); } while (0)
#define debug_print(fmt) \
        do { if (DEBUG) printf("%s:%d:%s(): " fmt, __FILE__, \
                                __LINE__, __func__); } while (0)


#define TASK_STEERING_PRIO              1
#define TASK_STEERING_PERIOD            15

#define TASK_DRIVING_PRIO               1
#define TASK_DRIVING_PERIOD             15

#define TASK_ACCELEROMETER_PRIO         2
#define TASK_ACCELEROMETER_PERIOD       500

    typedef struct {
        __IO uint32_t x, y, z;
    } ACCELEROMETER_T;

    typedef struct {
        uint8_t steering_direction : 2;
        uint16_t steering_value;
        uint8_t driving_direction : 2;
        uint16_t driving_value;
        uint8_t remote : 2;
        ACCELEROMETER_T accelerometer;
    } RC_STATE_T;





#ifdef	__cplusplus
}
#endif

#endif	/* GENERAL_H */

