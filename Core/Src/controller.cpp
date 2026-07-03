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
    gLogger.info("Debug mode, each step waits for a button press");
    runAllInteractive();
    gLogger.info("Debug tests complete, reset board to repeat");
}

void Controller::handleRun()
{
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
