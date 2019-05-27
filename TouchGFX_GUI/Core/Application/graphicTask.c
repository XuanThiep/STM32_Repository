/*
 * graphicTask.c
 *
 *  Created on: Apr 23, 2019
 *      Author: boome
 */

/* Include Header */
#include "graphicTask.h"

osThreadId graphicTaskHandle;

void touchGFX_Enter();

/* Function Inplement */
void graphicTask_Function(void const *argument)
{
	DBG("\r\nGraphic Task Function !");
	touchGFX_Enter();

	while(1)
	{
		osDelay(10);

	}
}
