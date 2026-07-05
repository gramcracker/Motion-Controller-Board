#include "motion_control.h"
#include "controller.h"
#include "config.h"
#include "logger.h"
#include "pins.h"
#include "tim.h"
#include "main.h"

bool MotionControl::initialize(Drivetrain *p_drivetrain)
{
    if (p_drivetrain == nullptr)
    {
        gLogger.error("Motion init FAILED no drivetrain");
        return false;
    }

    gLogger.info("Motion init ...");

    m_pDrivetrain = p_drivetrain;

    m_pDrivetrain->resetEncoder(MOTOR_SIDE_LEFT);
    m_pDrivetrain->resetEncoder(MOTOR_SIDE_RIGHT);

    float circumference_mm     = 3.14159265f * float(WHEEL_DIAMETER_MM);
    float counts_per_wheel_rev = float(ENCODER_PPR) * float(ENCODER_QUADRATURE) * GEAR_RATIO;

    if (counts_per_wheel_rev > 0.0f)
    {
        m_mmPerCount = circumference_mm / counts_per_wheel_rev;
    }

    m_encTotalLeft  = 0;
    m_encTotalRight = 0;
    m_velLeft       = 0.0f;
    m_velRight      = 0.0f;

    m_ready = true;

    gLogger.info("Motion init ok");

    return true;
}

void MotionControl::tick()
{
    if (m_ready == false)
    {
        return;
    }

    int16_t delta_left  = 0;
    int16_t delta_right = 0;

    m_pDrivetrain->readEncoderDelta(MOTOR_SIDE_LEFT, delta_left);
    m_pDrivetrain->readEncoderDelta(MOTOR_SIDE_RIGHT, delta_right);

    m_encTotalLeft  += int32_t(delta_left);
    m_encTotalRight += int32_t(delta_right);

    float vel_left  = float(delta_left)  * m_mmPerCount * float(CONTROL_RATE_HZ);
    float vel_right = float(delta_right) * m_mmPerCount * float(CONTROL_RATE_HZ);

    m_velLeft  = (m_velLeft  * MOTION_VEL_FILTER_OLD) + (vel_left  * MOTION_VEL_FILTER_NEW);
    m_velRight = (m_velRight * MOTION_VEL_FILTER_OLD) + (vel_right * MOTION_VEL_FILTER_NEW);

    if (ENABLE_VELOCITY_LOOP == 1)
    {
        runVelocityLoop();
    }
}

void MotionControl::runVelocityLoop()
{
    float half_track = float(TRACK_WIDTH_MM) * 0.5f;
    float w_radps    = float(m_targetW) / 1000.0f;

    float target_left  = float(m_targetV) - (w_radps * half_track);
    float target_right = float(m_targetV) + (w_radps * half_track);

    int16_t duty_left  = pidStep(target_left,  m_velLeft,  m_pidIntegralLeft,  m_pidPrevErrorLeft);
    int16_t duty_right = pidStep(target_right, m_velRight, m_pidIntegralRight, m_pidPrevErrorRight);

    m_pDrivetrain->drive(MOTOR_SIDE_LEFT, duty_left);
    m_pDrivetrain->drive(MOTOR_SIDE_RIGHT, duty_right);
}

int16_t MotionControl::pidStep(float target, float measured, float &integral, float &prev_error)
{
    float dt    = 1.0f / float(CONTROL_RATE_HZ);
    float error = target - measured;

    integral += error * dt;

    if (integral > VEL_PID_INTEGRAL_MAX)
    {
        integral = VEL_PID_INTEGRAL_MAX;
    }

    if (integral < -VEL_PID_INTEGRAL_MAX)
    {
        integral = -VEL_PID_INTEGRAL_MAX;
    }

    float derivative = (error - prev_error) / dt;
    prev_error = error;

    float output = (VEL_PID_KP * error) + (VEL_PID_KI * integral) + (VEL_PID_KD * derivative);

    if (output > float(Pins::MOTOR_PWM_MAX))
    {
        output = float(Pins::MOTOR_PWM_MAX);
    }

    if (output < -float(Pins::MOTOR_PWM_MAX))
    {
        output = -float(Pins::MOTOR_PWM_MAX);
    }

    return int16_t(output);
}

void MotionControl::setTargets(int16_t v_mm_s, int16_t w_mrad_s)
{
    m_targetV = v_mm_s;
    m_targetW = w_mrad_s;
}

void MotionControl::getEncoderTotals(int32_t &out_left, int32_t &out_right)
{
    out_left  = m_encTotalLeft;
    out_right = m_encTotalRight;
}

void MotionControl::getBodyVelocity(int16_t &out_v_mm_s, int16_t &out_w_mrad_s)
{
    float v = (m_velLeft + m_velRight) * 0.5f;
    float w = 0.0f;

    if (TRACK_WIDTH_MM > 0)
    {
        w = ((m_velRight - m_velLeft) / float(TRACK_WIDTH_MM)) * 1000.0f;
    }

    out_v_mm_s   = int16_t(v);
    out_w_mrad_s = int16_t(w);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *p_htim)
{
    if (p_htim->Instance == CONTROL_LOOP_TIM)
    {
        gController.controlTick();
    }
}