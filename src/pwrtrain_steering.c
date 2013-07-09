#include "pwrtrain_steering.h"

extern RC_STATE_T rc_state;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
/**
 * @brief @brief Initialisiert den PWM Input für die Lenkung und den Antrieb.
 */
void initPWMInput() {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM pin to AF2 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);

    /* Enable the TIM4 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    //TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    /* Select the TIM4 Input Trigger: TI2FP2 */
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

    /* TIM enable counter */
    TIM_Cmd(TIM2, ENABLE);

    /* Enable the CC2 Interrupt Request */
    //TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    
    
    
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect TIM pin to AF2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

    /* Enable the TIM4 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);

    /* Select the TIM4 Input Trigger: TI2FP2 */
    TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);

    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);

    /* TIM enable counter */
    TIM_Cmd(TIM5, ENABLE);

    /* Enable the CC2 Interrupt Request */
    //TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
}

/**
 * @brief Initialisiert den PWM Output für die Lenkung und den Antrieb.
 * 
 */
void initPWMOutput() {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

    uint16_t TimerPeriod = 47752; //(SystemCoreClock /1);
    uint16_t Channel1Pulse = (uint16_t) (((uint32_t) 7816 * (TimerPeriod - 1)) / 100000);

    printf("\rTimerPeriod: %d\n\rChannelPulse: %d\n\r", TimerPeriod, Channel1Pulse);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 64;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    TIM_Cmd(TIM1, DISABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);


    //PWM Input

    //    //PWM Input
    //    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
    //    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //
    //    /* GPIOB clock enable */
    //    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);
    //    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    //
    //    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    //    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    //    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    //    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    //    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //
    //    /* Connect TIM pin to AF2 */
    //    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    //
    //    /* Enable the TIM4 global Interrupt */
    //    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //    NVIC_Init(&NVIC_InitStructure);
    //
    //    TIM_ICInitTypeDef TIM_ICInitStructure;
    //
    //    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    //    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    //    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    //    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    //    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    //
    //    TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
    //
    //    /* Select the TIM4 Input Trigger: TI2FP2 */
    //    TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
    //
    //    /* Select the slave Mode: Reset Mode */
    //    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    //    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    //
    //    /* TIM enable counter */
    //    TIM_Cmd(TIM4, ENABLE);
    //
    //    /* Enable the CC2 Interrupt Request */
    //    TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
}

__IO uint32_t value = 0, dir = 0; // 0 = neutral, 1 = links, 2 = rechts

void TIM2_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    value = (TIM_GetCapture1(TIM2) / 100);

    if (value > 1280 && value < 1310) {
        rc_state.steering_direction = STEERING_DIR_NEUTRAL;
    } else if (value > 1310) {
        rc_state.steering_direction = STEERING_DIR_RIGHT;
    } else if (value < 1280) {
        rc_state.steering_direction = STEERING_DIR_LEFT;
    }
    //printf("TIM2\tDutyCycle: %d\tSteering Direction: %d\n\r", value, rc_state.steering_direction);
    if(rc_state.steering_direction == STEERING_DIR_NEUTRAL){
        SetTIM1Duty(NEUTRAL_POS);
    }else if(rc_state.steering_direction == STEERING_DIR_LEFT){
        printf("DutyCycle: %d\tlinks: %d \n\r", value,(3732-2*(1280-value)));
        SetTIM1Duty(3732-2*(1280-value));
    }else if(rc_state.steering_direction == STEERING_DIR_RIGHT){
        printf("DutyCycle: %d\trechts: %d \n\r", value,(3832+2*(value-1310)));
        SetTIM1Duty(3832+2*(value-1310));
    }
}

__IO uint32_t value2 = 0;

void TIM5_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
    value2 = (TIM_GetCapture2(TIM5) / 100);

    
    //printf("TIM5\tDutyCycle: %d\tDriving Direction: %d\n\r", value2, rc_state.driving_direction);
    /*if(dir == 0){
        SetTIM1Duty(3732);
    }else if(dir == 1){
        printf("DutyCycle: %d\tlinks: %d \n\r", value,(3732-3*((65500-value) / 100)));
        SetTIM1Duty(3732-2*((65500-value) / 100));
    }else if(dir == 2){
        printf("DutyCycle: %d\trechts: %d \n\r", value,(3732+3*(value / 100)));
        SetTIM1Duty(3732+2*(value / 100));
    }*/
}
