/*
 * graphicTask.h
 *
 *  Created on: Apr 23, 2019
 *      Author: boome
 */

#ifndef APPLICATION_GRAPHICTASK_H_
#define APPLICATION_GRAPHICTASK_H_


#ifdef __cplusplus
extern "C"
{
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "ApplicationConfig.h"


extern	osThreadId graphicTaskHandle;


void graphicTask_Function(void const *argument);


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_GRAPHICTASK_H_ */
