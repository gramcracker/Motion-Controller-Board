#include "controller.h"
#include "ir_array_globals.h"
#include <string.h>
#include "config.h"
#include "globals.h"
#include "pins.h"
#include "logger.h"
#include "stm32g4xx_hal.h"

Controller gController;

static void putU8(uint8_t *p_buf, int &idx, uint8_t value)
{
    p_buf[idx] = value;
    idx++;
}

static void putU16(uint8_t *p_buf, int &idx, uint16_t value)
{
    p_buf[idx] = uint8_t(value & 0xFF);
    idx++;
    p_buf[idx] = uint8_t((value >> 8) & 0xFF);
    idx++;
}

static void putU32(uint8_t *p_buf, int &idx, uint32_t value)
{
    p_buf[idx] = uint8_t(value & 0xFF);
    idx++;
    p_buf[idx] = uint8_t((value >> 8) & 0xFF);
    idx++;
    p_buf[idx] = uint8_t((value >> 16) & 0xFF);
    idx++;
    p_buf[idx] = uint8_t((value >> 24) & 0xFF);
    idx++;
}

static void putFloat(uint8_t *p_buf, int &idx, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    putU32(p_buf, idx, bits);
}

static float getFloat(const uint8_t *p_buf, int &idx)
{
    uint32_t bits = uint32_t(p_buf[idx]) | (uint32_t(p_buf[idx + 1]) << 8) | (uint32_t(p_buf[idx + 2]) << 16) | (uint32_t(p_buf[idx + 3]) << 24);
    idx += 4;

    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));

    return value;
}

static bool ledBlink(int count)
{
    int blink = 0;

    for (blink = 0; blink < count; blink++)
    {
        HAL_GPIO_WritePin(Pins::LED_HEARTBEAT.p_port, Pins::LED_HEARTBEAT.pin, GPIO_PIN_SET);
        HAL_Delay(120);
        HAL_GPIO_WritePin(Pins::LED_HEARTBEAT.p_port, Pins::LED_HEARTBEAT.pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }

    return true;
}

bool Controller::initialize()
{
    if (gLogger.initialize(Pins::DEBUG_UART) == false)
    {
        return false;
    }

    buildTestList();
    m_stateMachine.initialize(ControllerState::BOOT);

    return true;
}

bool Controller::execute()
{
    serviceLink();

    switch (m_stateMachine.getState())
    {
        case ControllerState::BOOT:
            handleBoot();
            break;

        case ControllerState::PASSIVE_TEST:
            handlePassiveTest();
            break;

        case ControllerState::DEBUG_TEST:
            handleDebugTest();
            break;

        case ControllerState::RUN:
            handleRun();
            break;

        case ControllerState::FAULT:
            handleFault();
            break;

        default:
            break;
    }

    return true;
}

bool Controller::buildTestList()
{
    m_testCount = 0;

#if ENABLE_LINK == 1
    m_pTests[m_testCount] = &m_link;
    m_testCount++;
#endif

#if ENABLE_POWER == 1
    m_pTests[m_testCount] = &m_power;
    m_testCount++;
#endif

#if ENABLE_IMU == 1
    m_pTests[m_testCount] = &m_imu;
    m_testCount++;
#endif

#if ENABLE_DRIVETRAIN == 1
    m_pTests[m_testCount] = &m_drivetrain;
    m_testCount++;
#endif

#if ENABLE_CLIFF == 1
    m_pTests[m_testCount] = &m_cliff;
    m_testCount++;
#endif

#if ENABLE_IR == 1
    m_pTests[m_testCount] = &m_irArray;
    m_testCount++;
#endif

    return true;
}

bool Controller::runPassiveSuite(StatusCode &out_worst)
{
    int        test = 0;
    char       text[STATUS_TEXT_LEN] = {0};
    StatusCode result;

    out_worst = makeOk(gThisBoard, ComponentId::SYSTEM, 0);

    for (test = 0; test < m_testCount; test++)
    {
        m_pTests[test]->runPassiveTest(result);
        m_results[test] = result;
        combineWorst(out_worst, result);

        statusToString(result, text, sizeof(text));
        gLogger.info("Passive %s result:%s", m_pTests[test]->getName(), text);
    }

    return out_worst.isOk();
}

bool Controller::runAllInteractive()
{
    int        test = 0;
    char       text[STATUS_TEXT_LEN] = {0};
    StatusCode result;

    for (test = 0; test < m_testCount; test++)
    {
        gLogger.info("Test %d of %d: %s", test + 1, m_testCount, m_pTests[test]->getName());

        m_pTests[test]->runInteractiveTest(result);
        statusToString(result, text, sizeof(text));
        gLogger.info("Interactive %s result:%s", m_pTests[test]->getName(), text);
    }

    return true;
}

bool Controller::isButtonDown()
{
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
        return true;
    }

    return false;
}

void Controller::serviceLink()
{
    LinkFrame frame;

    while (m_link.poll(frame) == true)
    {
        dispatchLink(frame);
    }
}

SelfTest *Controller::findTestByComponent(ComponentId component)
{
    int test = 0;

    for (test = 0; test < m_testCount; test++)
    {
        if (m_pTests[test]->getComponent() == component)
        {
            return m_pTests[test];
        }
    }

    return nullptr;
}

void Controller::handleRunTestCommand(const LinkFrame &frame)
{
    StatusCode  result;
    ComponentId component = ComponentId::NONE;
    SelfTest   *p_test    = nullptr;

    if (frame.length < 1)
    {
        m_link.sendNack();
        return;
    }

    component = ComponentId(frame.payload[0]);
    p_test    = findTestByComponent(component);

    if (p_test == nullptr)
    {
        result = makeStatus(gThisBoard, component, 0, Fault::INVALID_ID);
        m_link.sendTestDone(result);
        return;
    }

    p_test->runPassiveTest(result);
    m_link.sendTestDone(result);
}

void Controller::dispatchLink(const LinkFrame &frame)
{
    switch (frame.type)
    {
        case LinkCmd::Ping:
            m_link.sendPong();
            break;

        case LinkCmd::GetResults:
            m_link.sendResults(m_results, m_testCount);
            break;

        case LinkCmd::RunTest:
            handleRunTestCommand(frame);
            break;

        case LinkCmd::StartRun:
            if (ENABLE_DRIVETRAIN == 1)
            {
                m_drivetrain.setEnabled(true);
            }
            m_stateMachine.setState(ControllerState::RUN);
            m_link.sendAck();
            break;

        case LinkCmd::Drive:
            handleDriveCommand(frame);
            break;

        case LinkCmd::SetIr:
            handleSetIrCommand(frame);
            break;

        case LinkCmd::GetParams:
            handleGetParams();
            break;

        case LinkCmd::SetParams:
            handleSetParams(frame);
            break;

        case LinkCmd::Reboot:
            m_link.sendAck();
            HAL_NVIC_SystemReset();
            break;

        case LinkCmd::EStop:
            m_driveV = 0;
            m_driveW = 0;
            HAL_GPIO_WritePin(Pins::MOTOR_STBY.p_port, Pins::MOTOR_STBY.pin, GPIO_PIN_RESET);
            m_stateMachine.setState(ControllerState::FAULT);
            m_link.sendAck();
            break;

        default:
            m_link.sendNack();
            break;
    }
}

void Controller::handleDriveCommand(const LinkFrame &frame)
{
    if (frame.length < 4)
    {
        return;
    }

    int16_t v = int16_t(uint16_t(frame.payload[0]) | (uint16_t(frame.payload[1]) << 8));
    int16_t w = int16_t(uint16_t(frame.payload[2]) | (uint16_t(frame.payload[3]) << 8));

    m_driveV      = v;
    m_driveW      = w;
    m_lastDriveMs = HAL_GetTick();

    m_motion.setTargets(v, w);

    applyDrive();
}

void Controller::handleSetIrCommand(const LinkFrame &frame)
{
    if (frame.length < 1)
    {
        m_link.sendNack();
        return;
    }

    m_irMode = frame.payload[0];
    m_link.sendAck();
}

void Controller::applyDrive()
{
    if (ENABLE_DRIVETRAIN == 0)
    {
        return;
    }

    if (ENABLE_VELOCITY_LOOP == 1)
    {
        return;
    }

    if (m_stateMachine.getState() != ControllerState::RUN)
    {
        return;
    }

    int32_t left  = int32_t(m_driveV) - int32_t(m_driveW);
    int32_t right = int32_t(m_driveV) + int32_t(m_driveW);

    if (left > int32_t(Pins::MOTOR_PWM_MAX))
    {
        left = int32_t(Pins::MOTOR_PWM_MAX);
    }

    if (left < -int32_t(Pins::MOTOR_PWM_MAX))
    {
        left = -int32_t(Pins::MOTOR_PWM_MAX);
    }

    if (right > int32_t(Pins::MOTOR_PWM_MAX))
    {
        right = int32_t(Pins::MOTOR_PWM_MAX);
    }

    if (right < -int32_t(Pins::MOTOR_PWM_MAX))
    {
        right = -int32_t(Pins::MOTOR_PWM_MAX);
    }

    m_drivetrain.drive(MOTOR_SIDE_LEFT, int16_t(left));
    m_drivetrain.drive(MOTOR_SIDE_RIGHT, int16_t(right));
}

void Controller::driveWatchdog()
{
    if (ENABLE_DRIVETRAIN == 0)
    {
        return;
    }

    if ((HAL_GetTick() - m_lastDriveMs) <= DRIVE_WATCHDOG_MS)
    {
        return;
    }

    if ((m_driveV == 0) && (m_driveW == 0))
    {
        return;
    }

    m_driveV = 0;
    m_driveW = 0;

    m_motion.setTargets(0, 0);

    if (ENABLE_VELOCITY_LOOP == 0)
    {
        m_drivetrain.coast(MOTOR_SIDE_LEFT);
        m_drivetrain.coast(MOTOR_SIDE_RIGHT);
    }
}

void Controller::controlTick()
{
    if (m_stateMachine.getState() != ControllerState::RUN)
    {
        return;
    }

    m_motion.tick();
}

void Controller::handleGetParams()
{
    MotionParams params;
    uint8_t      payload[20] = {0};
    int          idx = 0;

    m_motion.getParams(params);

    putFloat(payload, idx, params.mm_per_count);
    putFloat(payload, idx, params.track_width_mm);
    putFloat(payload, idx, params.kp);
    putFloat(payload, idx, params.ki);
    putFloat(payload, idx, params.kd);

    m_link.sendResponse(LinkResp::Params, payload, uint8_t(idx));
}

void Controller::handleSetParams(const LinkFrame &frame)
{
    MotionParams params;
    int          idx = 0;

    if (frame.length < 20)
    {
        m_link.sendNack();
        return;
    }

    params.mm_per_count   = getFloat(frame.payload, idx);
    params.track_width_mm = getFloat(frame.payload, idx);
    params.kp             = getFloat(frame.payload, idx);
    params.ki             = getFloat(frame.payload, idx);
    params.kd             = getFloat(frame.payload, idx);

    m_motion.setParams(params);
    m_link.sendAck();
}

void Controller::sendTelemetry()
{
    uint8_t  payload[LINK_MAX_PAYLOAD] = {0};
    int      idx   = 0;
    int      pair  = 0;
    int32_t  enc_l = 0;
    int32_t  enc_r = 0;
    int16_t  meas_v = 0;
    int16_t  meas_w = 0;
    int16_t  pose_x = 0;
    int16_t  pose_y = 0;
    int16_t  pose_theta = 0;
    float    ax = 0.0f;
    float    ay = 0.0f;
    float    az = 0.0f;
    float    gx = 0.0f;
    float    gy = 0.0f;
    float    gz = 0.0f;
    uint16_t wall[TELEMETRY_WALL_COUNT] = {0};
    uint16_t cliff = 0;

    if (ENABLE_DRIVETRAIN == 1)
    {
        m_motion.getEncoderTotals(enc_l, enc_r);
        m_motion.getBodyVelocity(meas_v, meas_w);
        m_motion.getPose(pose_x, pose_y, pose_theta);
    }

    if (ENABLE_IMU == 1)
    {
        m_imu.readAccel(ax, ay, az);
        m_imu.readGyro(gx, gy, gz);
    }

    if (ENABLE_IR == 1)
    {
        for (pair = 0; pair < IR_PAIR_COUNT; pair++)
        {
            if (pair < TELEMETRY_WALL_COUNT)
            {
                m_irArray.readAmbient(uint8_t(pair), wall[pair]);
            }
        }
    }

    if (ENABLE_CLIFF == 1)
    {
        m_cliff.readValue(cliff);
    }

    // layout: see Communication Protocol doc, Telemetry frame.
    // accel packed as milli-g, gyro packed as deci-deg/s.
    putU8(payload, idx, TELEMETRY_VERSION);
    putU8(payload, idx, m_telemetrySeq);
    putU32(payload, idx, HAL_GetTick());
    putU8(payload, idx, uint8_t(m_stateMachine.getState()));
    putU16(payload, idx, 0);
    putU32(payload, idx, uint32_t(enc_l));
    putU32(payload, idx, uint32_t(enc_r));
    putU16(payload, idx, uint16_t(pose_x));
    putU16(payload, idx, uint16_t(pose_y));
    putU16(payload, idx, uint16_t(pose_theta));
    putU16(payload, idx, uint16_t(meas_v));
    putU16(payload, idx, uint16_t(meas_w));
    putU16(payload, idx, uint16_t(int16_t(ax * 1000.0f)));
    putU16(payload, idx, uint16_t(int16_t(ay * 1000.0f)));
    putU16(payload, idx, uint16_t(int16_t(az * 1000.0f)));
    putU16(payload, idx, uint16_t(int16_t(gx * 10.0f)));
    putU16(payload, idx, uint16_t(int16_t(gy * 10.0f)));
    putU16(payload, idx, uint16_t(int16_t(gz * 10.0f)));

    for (pair = 0; pair < TELEMETRY_WALL_COUNT; pair++)
    {
        putU16(payload, idx, wall[pair]);
    }

    putU16(payload, idx, cliff);
    putU16(payload, idx, 0);

    m_telemetrySeq++;

    m_link.sendResponse(LinkResp::Telemetry, payload, uint8_t(idx));
}

void Controller::sendBootedOnce()
{
    if (m_bootedSent == true)
    {
        return;
    }

    m_link.sendBooted();
    m_bootedSent = true;
}

void Controller::handleBoot()
{
    int test = 0;

    for (test = 0; test < m_testCount; test++)
    {
        m_pTests[test]->initialize();
    }

    HAL_Delay(BOOT_BUTTON_SETTLE_MS);

    if (isButtonDown() == true)
    {
        gLogger.info("Button held at boot, entering debug mode");
        m_stateMachine.setState(ControllerState::DEBUG_TEST);
        return;
    }

    m_stateMachine.setState(ControllerState::PASSIVE_TEST);
}

void Controller::handlePassiveTest()
{
    StatusCode worst;

    runPassiveSuite(worst);
    gLogger.info("Boot status raw:0x%04X", worst.getRaw());

    if (worst.isOk() == true)
    {
        ledBlink(1);
    }

    if (worst.isOk() == false)
    {
        ledBlink(3);
    }

    if (ENABLE_DRIVETRAIN == 1)
    {
        m_motion.initialize(&m_drivetrain);
    }

    sendBootedOnce();
    m_stateMachine.setState(ControllerState::RUN);
}

void Controller::handleDebugTest()
{
    StatusCode worst;

    if (m_stateMachine.isNewState() == false)
    {
        return;
    }

    runPassiveSuite(worst);
    sendBootedOnce();
    gLogger.info("Debug mode, each step waits for a button press");
    runAllInteractive();
    gLogger.info("Debug tests complete, reset board to repeat");
}

void Controller::handleRun()
{
    driveWatchdog();

    if ((HAL_GetTick() - m_lastTelemetryMs) >= TELEMETRY_PERIOD_MS)
    {
        m_lastTelemetryMs = HAL_GetTick();
        sendTelemetry();
    }

    if ((HAL_GetTick() - m_lastHeartbeat) < HEARTBEAT_PERIOD_MS)
    {
        return;
    }

    m_lastHeartbeat = HAL_GetTick();
    HAL_GPIO_TogglePin(Pins::LED_HEARTBEAT.p_port, Pins::LED_HEARTBEAT.pin);
}

void Controller::handleFault()
{
    ledBlink(5);
    HAL_Delay(500);
}