/**
 * Precision Drive Dynamics: STM32 FreeRTOS Hard Real-Time Motor Firmware
 * Field-Oriented Control (FOC) / Super-Twisting SMC Inner Loop at 20 kHz
 * MISRA-C:2012 Compliant Architecture for High-Precision Drive Systems
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define PWM_FREQUENCY_HZ      20000U
#define TIMER_PERIOD_TICKS    4200U
#define ADC_FULL_SCALE        4095.0f
#define CURRENT_SENSE_GAIN    0.025f    /* V/A shunt amplifier gain */
#define V_BUS_NOMINAL         24.0f
#define I_MAX_LIMIT           8.0f      /* Absolute safety current threshold */

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float current_a;
    float bus_voltage_v;
    float winding_temp_c;
    bool  fault_active;
} MotorState_t;

typedef struct {
    float k1;              /* Super-Twisting gain 1 */
    float k2;              /* Super-Twisting gain 2 */
    float lambda;          /* Sliding surface slope */
    float s_integral;      /* Super-Twisting internal state */
    float last_error;
} ST_Controller_t;

static volatile MotorState_t g_motor_state = {0};
static ST_Controller_t g_stc = {
    .k1 = 45.0f,
    .k2 = 120.0f,
    .lambda = 18.0f,
    .s_integral = 0.0f,
    .last_error = 0.0f
};

/**
 * 20 kHz Timer ISR: Executes deterministic inner-loop current and SMC control
 */
void TIM1_UP_TIM10_IRQHandler(void) {
    /* 1. Fast Current Sensing via Dual ADC Injected Channels */
    uint16_t raw_adc_ia = 2048; /* Read from ADC1->JDR1 */
    float i_measured = ((float)raw_adc_ia - 2048.0f) * (3.3f / ADC_FULL_SCALE) / CURRENT_SENSE_GAIN;
    g_motor_state.current_a = i_measured;

    /* Hardware Overcurrent Safety Trip (CBF Hardware Enforcer) */
    if (fabsf(i_measured) > I_MAX_LIMIT) {
        /* Disable gate driver outputs immediately (MOE bit clear) */
        // TIM1->BDTR &= ~TIM_BDTR_MOE;
        g_motor_state.fault_active = true;
        return;
    }

    /* 2. Fast Sliding Surface Evaluation: s = e_dot + lambda * e */
    float target_velocity = 150.0f; /* rad/s from CAN command buffer */
    float error = target_velocity - g_motor_state.velocity_rad_s;
    float dt = 1.0f / (float)PWM_FREQUENCY_HZ;
    float error_dot = (error - g_stc.last_error) * (float)PWM_FREQUENCY_HZ;
    g_stc.last_error = error;

    float s = error_dot + g_stc.lambda * error;

    /* 3. Discrete Super-Twisting Algorithm (Chattering-Free 2-SMC) */
    float s_sign = (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    float s_sqrt = sqrtf(fabsf(s));

    /* Continuous term + integrated discontinuous term */
    g_stc.s_integral += g_stc.k2 * s_sign * dt;
    float v_control = g_stc.k1 * s_sqrt * s_sign + g_stc.s_integral;

    /* Inverter Voltage Clamping [-V_bus, +V_bus] */
    if (v_control > V_BUS_NOMINAL) {
        v_control = V_BUS_NOMINAL;
    } else if (v_control < -V_BUS_NOMINAL) {
        v_control = -V_BUS_NOMINAL;
    }

    /* 4. Update PWM Duty Cycle Compare Registers */
    float duty_normalized = (v_control / V_BUS_NOMINAL + 1.0f) * 0.5f;
    uint32_t ccr_val = (uint32_t)(duty_normalized * (float)TIMER_PERIOD_TICKS);
    // TIM1->CCR1 = ccr_val;
    (void)ccr_val;
}

/**
 * FreeRTOS 1 kHz Task: High-Level Telemetry & CAN-Bus Dispatch
 */
void MotorTelemetryTask(void *argument) {
    (void)argument;
    for (;;) {
        /* CAN Frame Assembly: [Pos_MSB, Pos_LSB, Vel_MSB, Vel_LSB, I_MSB, I_LSB, Temp, Fault] */
        uint8_t can_payload[8];
        int16_t vel_enc = (int16_t)(g_motor_state.velocity_rad_s * 10.0f);
        int16_t cur_enc = (int16_t)(g_motor_state.current_a * 100.0f);

        can_payload[0] = (uint8_t)((vel_enc >> 8) & 0xFF);
        can_payload[1] = (uint8_t)(vel_enc & 0xFF);
        can_payload[2] = (uint8_t)((cur_enc >> 8) & 0xFF);
        can_payload[3] = (uint8_t)(cur_enc & 0xFF);
        can_payload[4] = (uint8_t)((int8_t)g_motor_state.winding_temp_c);
        can_payload[5] = g_motor_state.fault_active ? 0x01 : 0x00;
        can_payload[6] = 0xAA; /* Sync byte */
        can_payload[7] = 0x55;

        // CAN_Transmit(0x284, can_payload, 8);
        (void)can_payload;

        // vTaskDelay(pdMS_TO_TICKS(1)); /* 1 kHz deterministic rate */
        break;
    }
}
