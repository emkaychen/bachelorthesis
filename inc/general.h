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
#define STEERING_POS_LEFT_MAX           3100
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
#define PWRTRAIN_POS_FORWARDS_MAX       3100
    /** PWM Signal für die maximale Rückwärtsfahrt. */
#define PWRTRAIN_POS_BACKWARDS_MAX      4700

    
    
    typedef struct {
        uint8_t steering_direction : 2;
        uint8_t driving_direction : 2;
    } RC_STATE_T;



#ifdef	__cplusplus
}
#endif

#endif	/* GENERAL_H */

