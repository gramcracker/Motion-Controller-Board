#pragma once

#include "status_codes.h"

class SelfTest
{
public:
    virtual bool        initialize() = 0;
    virtual const char *getName() = 0;
    virtual ComponentId getComponent() = 0;

    virtual bool runPassiveTest(StatusCode &out_status) = 0;

    virtual bool runInteractiveTest(StatusCode &out_status)
    {
        out_status = makeStatus(Board::STM32, getComponent(), 0, Fault::NOT_TESTED);

        return false;
    }

    virtual ~SelfTest() {}
};
