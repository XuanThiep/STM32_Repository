/*
 * ApplicationConfig.h
 *
 *  Created on: May 26, 2019
 *      Author: boome
 */

#ifndef CONFIG_APPLICATIONCONFIG_H_
#define CONFIG_APPLICATIONCONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmsis_os.h"
#include "printf.h"


/*	Define Type of Debug */
typedef enum
{
	DEBUG_TYPE_SWV = 0,
	DEBUG_TYPE_UART
}Debug_Type;

/* Comment this define to turn off debug feature */
#define		DEBUG_TYPE					DEBUG_TYPE_SWV


#if defined(DEBUG_TYPE)
#define		DBG							printf
#else
#define		DBG							(void)
#endif



/*	Define task Stack Size (WORD) */
#define GRAPHIC_TASK_STACK_SIZE			(1024)	/* 2048 Bytes */

/* Define task Priority	*/
#define GRAPHIC_TASK_PRIORITY			(osPriorityLow)



#ifdef __cplusplus
}
#endif

#endif /* CONFIG_APPLICATIONCONFIG_H_ */
