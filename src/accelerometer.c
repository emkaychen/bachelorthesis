#include "accelerometer.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define ACCELEROMETER_VAL_COUNT 10 //Bei Änderung index ändern!
extern RC_STATE_T rc_state;

static struct {
    ACCELEROMETER_T buffer[ACCELEROMETER_VAL_COUNT];
    __IO uint8_t index : 4;
} acc_fifo;

__IO uint8_t accelerBuffer[6];

static void init_acc_fifo();
static void read_value_acc_fifo();
static void update_avg_acc_fifo();

static void init_acc_fifo() {
    for (acc_fifo.index = 0; acc_fifo.index < ACCELEROMETER_VAL_COUNT; acc_fifo.index++) {
        LIS302DL_ReadACC(&acc_fifo.buffer[acc_fifo.index]);
        //        LIS302DL_Read(accelerBuffer, LIS302DL_OUT_X_ADDR, 6);
        //        acc_fifo.buffer[acc_fifo.index].x = accelerBuffer[0];
        //        acc_fifo.buffer[acc_fifo.index].y = accelerBuffer[2];
        //        acc_fifo.buffer[acc_fifo.index].z = accelerBuffer[4];
    }
    acc_fifo.index = 0;
    update_avg_acc_fifo();
}

static void read_value_acc_fifo() {
    //    uint8_t tmp = 0;
    //    do {
    //        LIS302DL_Read(&tmp, LIS302DL_STATUS_REG_ADDR, 1);
    //    } while (!(tmp & 0x08));
    //
    //    LIS302DL_Read(accelerBuffer, LIS302DL_OUT_X_ADDR, 6);
    //    acc_fifo.buffer[acc_fifo.index].x = accelerBuffer[0];
    //    acc_fifo.buffer[acc_fifo.index].y = accelerBuffer[2];
    //    acc_fifo.buffer[acc_fifo.index].z = accelerBuffer[4];
    LIS302DL_ReadACC(&acc_fifo.buffer[acc_fifo.index]);
    if (acc_fifo.index >= ACCELEROMETER_VAL_COUNT - 1) {
        acc_fifo.index = 0;
    } else {
        acc_fifo.index++;
    }
}

static void update_avg_acc_fifo() {
    int32_t sumx = 0, sumy = 0, sumz = 0;
    for (int8_t i = 0; i < ACCELEROMETER_VAL_COUNT; i++) {
        sumx += acc_fifo.buffer[i].x;
        sumy += acc_fifo.buffer[i].y;
        sumz += acc_fifo.buffer[i].z;
    }
    rc_state.accelerometer.x = (sumx / ACCELEROMETER_VAL_COUNT);
    rc_state.accelerometer.y = (sumy / ACCELEROMETER_VAL_COUNT);
    rc_state.accelerometer.z = (sumz / ACCELEROMETER_VAL_COUNT);
}

static void info() {
    int32_t sumx = 0, sumy = 0, sumz = 0;
    for (uint8_t i = 0; i < ACCELEROMETER_VAL_COUNT; i++) {
        printf("[%d]\tX:%d\tY:%d\tZ:%d\t\n\r", i, acc_fifo.buffer[i].x, acc_fifo.buffer[i].y, acc_fifo.buffer[i].z);
        sumx += acc_fifo.buffer[i].x;
        sumy += acc_fifo.buffer[i].y;
        sumz += acc_fifo.buffer[i].z;
    }
    printf("S:\tX:%d\tY:%d\tZ:%d\t\n\r", sumx, sumy, sumz);
    update_avg_acc_fifo();
    printf("S:\tX:%d\tY:%d\tZ:%d\t\n\r", rc_state.accelerometer.x, rc_state.accelerometer.y, rc_state.accelerometer.z);
}

void initAccelerometer() {
    LIS302DL_InitTypeDef LIS302DL_InitStruct;
    LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
    LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
    LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
    LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
    LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
    LIS302DL_Init(&LIS302DL_InitStruct);

    for (uint64_t i = 0; i < 10000000; i++);


    //    LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;
    //    LIS302DL_FilterStruct.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
    //    LIS302DL_FilterStruct.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
    //    LIS302DL_FilterStruct.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
    //    LIS302DL_FilterConfig(&LIS302DL_FilterStruct);

    init_acc_fifo();


    //        LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;
    //        LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
    //        LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_XYZ_ENABLE;
    //        LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_XYZ_DISABLE;
    //        LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);


    uint8_t ctrl = 0x00;
    //        LIS302DL_Write(&ctrl, LIS302DL_FF_WU_CFG1_REG_ADDR, 1);
    ctrl = 0x04;
    LIS302DL_Write(&ctrl, LIS302DL_CTRL_REG3_ADDR, 1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);


    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
uint8_t X_Offset, Y_Offset, Z_Offset = 0x00;

void EXTI0_IRQHandler(void) {
    //        debug_print("CB\n\r");
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        read_value_acc_fifo();
        update_avg_acc_fifo();
        EXTI_ClearITPendingBit(EXTI_Line0);
        /* Clear the EXTI line 0 pending bit */
    }
}

void vAccelerometerTask(void *pvParameters) {
    debug_print("Accelerometer Task started.\n\r");
    const portTickType xDelay = TASK_ACCELEROMETER_PERIOD / portTICK_RATE_MS;
    portTickType xLastWakeTime = xTaskGetTickCount();
    uint8_t Buffer[6];
    for (;;) {
        update_avg_acc_fifo();
        printf("TASK: Accelerometer:X:%3d\tY:%3d\tZ:%3d\n\r", rc_state.accelerometer.x, rc_state.accelerometer.y, rc_state.accelerometer.z);
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}
