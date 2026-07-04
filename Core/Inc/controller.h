#pragma once

#include "controller_globals.h"
#include "controller_state_machine.h"
#include "self_test.h"
#include "link.h"
#include "power.h"
#include "imu.h"
#include "drivetrain.h"
#include "cliff.h"
#include "ir_array.h"
#include <cstdint>

class Controller
{
public:
    bool initialize();
    bool execute();

private:
    bool buildTestList();
    bool runPassiveSuite(StatusCode &out_worst);
    bool runAllInteractive();
    bool isButtonDown();

    void      serviceLink();
    void      dispatchLink(const LinkFrame &frame);
    void      handleRunTestCommand(const LinkFrame &frame);
    void      handleDriveCommand(const LinkFrame &frame);
    void      handleSetIrCommand(const LinkFrame &frame);
    void      applyDrive();
    void      driveWatchdog();
    void      sendBootedOnce();
    SelfTest *findTestByComponent(ComponentId component);

    void handleBoot();
    void handlePassiveTest();
    void handleDebugTest();
    void handleRun();
    void handleFault();

    ControllerStateMachine m_stateMachine;

    Link       m_link;
    Power      m_power;
    Imu        m_imu;
    Drivetrain m_drivetrain;
    Cliff      m_cliff;
    IrArray    m_irArray;

    SelfTest  *m_pTests[CONTROLLER_MAX_TESTS] = {nullptr};
    int        m_testCount = 0;
    StatusCode m_results[CONTROLLER_MAX_TESTS];

    uint32_t   m_lastHeartbeat = 0;
    bool       m_bootedSent    = false;

    int16_t    m_driveV      = 0;
    int16_t    m_driveW      = 0;
    uint32_t   m_lastDriveMs = 0;
    uint8_t    m_irMode      = 0;
};

extern Controller gController;