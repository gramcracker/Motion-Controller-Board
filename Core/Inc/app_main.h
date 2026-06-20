// app_main.h
// Bridge from CubeMX's C main.c into the C++ application.
// In main.c:  #include "app_main.h"
//   after the MX_*_Init() calls:   app_setup();
//   inside while(1):               app_loop();
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void app_setup(void);
void app_loop(void);

#ifdef __cplusplus
}
#endif