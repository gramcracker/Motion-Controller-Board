// status_codes.cpp  (compiled only in DEBUG_BUILD)
#include "status_codes.h"
#if defined(DEBUG_BUILD)
#include <cstdio>

const char* board_str(Board b) {
    switch (b) { case Board::STM32: return "STM32";
                 case Board::ESP32: return "ESP32"; default: return "-"; }
}
const char* component_str(Component c) {
    switch (c) {
    case Component::System: return "System"; case Component::Power: return "Power";
    case Component::Imu: return "Imu"; case Component::Encoder: return "Encoder";
    case Component::Motor: return "Motor"; case Component::Drivetrain: return "Drivetrain";
    case Component::IrEmitter: return "IrEmitter"; case Component::IrReceiver: return "IrReceiver";
    case Component::IrPair: return "IrPair"; case Component::Cliff: return "Cliff";
    case Component::UartLink: return "UartLink"; case Component::Display: return "Display";
    case Component::Camera: return "Camera"; case Component::Flash: return "Flash";
    default: return "None";
    }
}
const char* fault_str(Fault f) {
    switch (f) {
    case Fault::Ok: return "Ok"; case Fault::NoResponse: return "NoResponse";
    case Fault::OutOfRange: return "OutOfRange"; case Fault::StuckHigh: return "StuckHigh";
    case Fault::StuckLow: return "StuckLow"; case Fault::NoChange: return "NoChange";
    case Fault::WrongDirection: return "WrongDirection"; case Fault::Timeout: return "Timeout";
    case Fault::Underperform: return "Underperform"; case Fault::Crosstalk: return "Crosstalk";
    case Fault::NotPresent: return "NotPresent"; case Fault::NotTested: return "NotTested";
    case Fault::UserFail: return "UserFail"; case Fault::InvalidId: return "InvalidId";
    default: return "?";
    }
}
void status_to_string(StatusCode s, char* buf, uint32_t buflen) {
    snprintf(buf, buflen, "%s/%s[%u]: %s", board_str(s.board()),
             component_str(s.component()), s.instance(), fault_str(s.fault()));
}
#endif