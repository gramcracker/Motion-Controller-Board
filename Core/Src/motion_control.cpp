#include "motion_control.h"
#include "controller.h"
#include "config.h"
#include "logger.h"
#include "pins.h"
#include "tim.h"
#include "main.h"
#include <math.h>

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

    m_params.mm_per_count = 0.0f;

    if (counts_per_wheel_rev > 0.0f)
    {
        m_params.mm_per_count = circumference_mm / counts_per_wheel_rev;
    }

    m_params.track_width_mm = float(TRACK_WIDTH_MM);
    m_params.kp             = VEL_PID_KP;
    m_params.ki             = VEL_PID_KI;
    m_params.kd             = VEL_PID_KD;

    m_encTotalLeft  = 0;
    m_encTotalRight = 0;
    m_velLeft       = 0.0f;
    m_velRight      = 0.0f;
    m_poseX         = 0.0f;
    m_poseY         = 0.0f;
    m_poseTheta     = 0.0f;

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

    float vel_left  = float(delta_left)  * m_params.mm_per_count * float(CONTROL_RATE_HZ);
    float vel_right = float(delta_right) * m_params.mm_per_count * float(CONTROL_RATE_HZ);

    m_velLeft  = (m_velLeft  * MOTION_VEL_FILTER_OLD) + (vel_left  * MOTION_VEL_FILTER_NEW);
    m_velRight = (m_velRight * MOTION_VEL_FILTER_OLD) + (vel_right * MOTION_VEL_FILTER_NEW);

    integratePose();

    if (ENABLE_VELOCITY_LOOP == 1)
    {
        runVelocityLoop();
    }
}

void MotionControl::integratePose()
{
    float dt = 1.0f / float(CONTROL_RATE_HZ);
    float v  = (m_velLeft + m_velRight) * 0.5f;
    float w  = 0.0f;

    if (m_params.track_width_mm > 0.0f)
    {
        w = (m_velRight - m_velLeft) / m_params.track_width_mm;
    }

    m_poseTheta += w * dt;

    if (m_poseTheta > 3.14159265f)
    {
        m_poseTheta -= 6.28318531f;
    }

    if (m_poseTheta < -3.14159265f)
    {
        m_poseTheta += 6.28318531f;
    }

    m_poseX += v * cosf(m_poseTheta) * dt;
    m_poseY += v * sinf(m_poseTheta) * dt;
}

void MotionControl::runVelocityLoop()
{
    float half_track = m_params.track_width_mm * 0.5f;
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

    float output = (m_params.kp * error) + (m_params.ki * integral) + (m_params.kd * derivative);

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

    if (m_params.track_width_mm > 0.0f)
    {
        w = ((m_velRight - m_velLeft) / m_params.track_width_mm) * 1000.0f;
    }

    out_v_mm_s   = int16_t(v);
    out_w_mrad_s = int16_t(w);
}

void MotionControl::getPose(int16_t &out_x_mm, int16_t &out_y_mm, int16_t &out_theta_cdeg)
{
    out_x_mm       = int16_t(m_poseX);
    out_y_mm       = int16_t(m_poseY);
    out_theta_cdeg = int16_t(m_poseTheta * 5729.578f);
}

void MotionControl::getParams(MotionParams &out_params)
{
    out_params = m_params;
}

void MotionControl::setParams(const MotionParams &params)
{
    __disable_irq();
    m_params = params;
    __enable_irq();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *p_htim)
{
    if (p_htim->Instance == CONTROL_LOOP_TIM)
    {
        gController.controlTick();
    }
}