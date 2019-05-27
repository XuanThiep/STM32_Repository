/*
 * graphicTask.h
 *
 *  Created on: May 26, 2019
 *      Author: boome
 */

#ifndef GRAPHICTASK_H_
#define GRAPHICTASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "cmsis_os.h"
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_sdram.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_ts.h"
#include "lvgl.h"

#include "testThemes.h"
#include "ApplicationConfig.h"


#define VSYNC               1
#define VBP                 1
#define VFP                 1
#define VACT                480
#define HSYNC               1
#define HBP                 1
#define HFP                 1
#define HACT                800


extern void graphicTask_Function(void const * argument);
extern void LittlevGL_Graphic_Init(void);
void tft_init(void);
#ifdef __cplusplus
}
#endif


#endif /* GRAPHICTASK_H_ */
