#include "controller.h"
#include "config.h"
#include "globals.h"
#include "pins.h"
#include "logger.h"
#include "stm32f4xx_hal.h"

Controller gController;

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

    m_drivetrain.coast(MOTOR_SIDE_LEFT);
    m_drivetrain.coast(MOTOR_SIDE_RIGHT);
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