#pragma once

#include "drivetrain.h"
#include "motion_control_globals.h"
#include <cstdint>

struct MotionParams
{
    float mm_per_count;
    float track_width_mm;
    float kp;
    float ki;
    float kd;
};

class MotionControl
{
public:
    bool initialize(Drivetrain *p_drivetrain);
    void tick();
    void setTargets(int16_t v_mm_s, int16_t w_mrad_s);

    void getEncoderTotals(int32_t &out_left, int32_t &out_right);
    void getBodyVelocity(int16_t &out_v_mm_s, int16_t &out_w_mrad_s);
    void getPose(int16_t &out_x_mm, int16_t &out_y_mm, int16_t &out_theta_cdeg);

    void getParams(MotionParams &out_params);
    void setParams(const MotionParams &params);

private:
    void    runVelocityLoop();
    int16_t pidStep(float target, float measured, float &integral, float &prev_error);
    void    integratePose();

    Drivetrain *m_pDrivetrain = nullptr;
    bool        m_ready = false;

    MotionParams m_params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    int32_t     m_encTotalLeft  = 0;
    int32_t     m_encTotalRight = 0;

    float       m_velLeft  = 0.0f;
    float       m_velRight = 0.0f;

    float       m_poseX     = 0.0f;
    float       m_poseY     = 0.0f;
    float       m_poseTheta = 0.0f;

    int16_t     m_targetV = 0;
    int16_t     m_targetW = 0;

    float       m_pidIntegralLeft   = 0.0f;
    float       m_pidIntegralRight  = 0.0f;
    float       m_pidPrevErrorLeft  = 0.0f;
    float       m_pidPrevErrorRight = 0.0f;
};