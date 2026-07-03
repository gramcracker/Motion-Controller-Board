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
};

extern Controller gController;
