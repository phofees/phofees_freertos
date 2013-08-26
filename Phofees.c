/*
 * ### Phofees Project (FreeRTOS version) ###
 *
 * Phofees is a open source project, for more information see:
 *
 * 1. http://www.phofees.org and http://wiki.phofees.org
 * 2. Mailing list:
 *
 * This project is sponsored by Phi Innovations (www.phiinnovations.com)
 *
 * ### FreeBSD License clauses ####
 *
 * Copyright (c) 2013, Phofees (www.phoffes.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
*/

#include "Phofees_types.h"
#include "Phofees.h"
#include "task.h"

#ifdef DEBUG_MEASURE_TASK_MAX_TIME
extern unsigned long ulFreeCounter;
unsigned long ulMaxTaskProcess = 0;
#endif

typedef struct {
	pdPHOFEESTASK_CODE xCodeFunction;
	void *pvContext;
	void *pvNext;
	PhofeesExtTimer_t *pxExtTimers;
	PhofeesExtTimer_t xTimerLoaded;
	PhofeesEvent_t xEventStatus;
	PhofeesTimer_t xTimer;
	unsigned char ucExtTimerNbr;
	PhofeesTaskId_t xPhofeesTaskId;
} PhofeesTaskConfig_t;

typedef struct {
	PhofeesTaskConfig_t *pxPhofeesTaskHead;
	void *pvNext;
	PhofeesMsgHeader_t *pxQueueHeader;
	xQueueHandle xQueue;
	xSemaphoreHandle xSemaphore;
	PhofeesTaskQueueSize_t xQueueSize;
	PhofeesTaskQueueSize_t xQueueSend;
	PhofeesTaskQueueSize_t xQueueReceive;
} PhofeesTaskListConfig_t;

//typedef PhofeesTaskConfig_t * PhofeesTaskHandle_t;
//typedef PhofeesTaskListConfig_t * PhofeesTaskListHandle_t;
#if ( configUSE_PHOFEES_TICK_HOOK == 1 )
	extern void vPhofeesApplicationTickHook(void);
#endif


static PhofeesTaskListConfig_t *pxPhofeesTaskListsHead = (void *)0;

PhofeesTaskListConfig_t *pxPhofeesTaskGetListFromPhofeesTaskId(PhofeesTaskId_t xPhofeesTaskId)
{
	PhofeesTaskConfig_t *pxCurrent;
	PhofeesTaskListConfig_t *pxTaskCurrent = pxPhofeesTaskListsHead;

	/* Search inside all Tasks */
	for(;;)
	{
		/* Check for end of list */
		if(pxTaskCurrent == (PhofeesTaskListConfig_t *)0)
		{
			/* No such task id */
			break;
		}
		/* Get PhofeesTask list */
		pxCurrent = (PhofeesTaskConfig_t *)pxTaskCurrent->pxPhofeesTaskHead;
		/* Roll over all PhofeesTasks */
		for(;;)
		{
			/* Check for end of list */
			if(pxCurrent == (PhofeesTaskConfig_t *)0)
			{
				/* Not inside this task */
				break;
			}
			/* Check PhofeesTask id */
			if(pxCurrent->xPhofeesTaskId == xPhofeesTaskId)
			{
				/* Found it */
				return pxTaskCurrent;
			}
			/* Next PhofeesTask */
			pxCurrent = (PhofeesTaskConfig_t *)pxCurrent->pvNext;
		}
		/* Next task */
		pxTaskCurrent = (PhofeesTaskListConfig_t *)pxTaskCurrent->pvNext;
	}
	return (PhofeesTaskListConfig_t *)0;
}

PhofeesTaskHandle_t xPhofeesTaskGetHandle( PhofeesTaskId_t xPhofeesTaskId )
{
	PhofeesTaskConfig_t *pxCurrent;
	PhofeesTaskListConfig_t *pxTaskCurrent = pxPhofeesTaskListsHead;

	/* Search inside all Tasks */
	for(;;)
	{
		/* Check for end of list */
		if(pxTaskCurrent == (PhofeesTaskListConfig_t *)0)
		{
			/* No such task id */
			break;
		}
		/* Get PhofeesTask list */
		pxCurrent = (PhofeesTaskConfig_t *)pxTaskCurrent->pxPhofeesTaskHead;
		/* Roll over all PhofeesTasks */
		for(;;)
		{
			/* Check PhofeesTask list empty */
			if(pxCurrent == (PhofeesTaskConfig_t *)0)
			{
				/* Not inside this task */
				break;
			}
			/* Check PhofeesTask id */
			if(pxCurrent->xPhofeesTaskId == xPhofeesTaskId)
			{
				/* Found it */
				return (PhofeesTaskHandle_t)pxCurrent;
			}
			/* Next PhofeesTask */
			pxCurrent = (PhofeesTaskConfig_t *)pxCurrent->pvNext;
		}
		/* Next task */
		pxTaskCurrent = (PhofeesTaskListConfig_t *)pxTaskCurrent->pvNext;
	}
	return (PhofeesTaskHandle_t)0;
}

PhofeesTaskContext_t *pxPhofeesTaskGetContext(PhofeesTaskHandle_t xPhofeesTask)
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	return (PhofeesTaskContext_t *)pxPhosfeesTaskConfig->pvContext;
}

PhofeesTaskListHandle_t xPhofeesTaskListCreate( PhofeesTaskQueueSize_t uxNumOfItems, unsigned short uxItemSize )
{
	PhofeesTaskListConfig_t **ppxCurrent = &pxPhofeesTaskListsHead;
	PhofeesTaskListConfig_t *pxCurrent = pxPhofeesTaskListsHead;

	/* Find Last Task List item */
	for(;;)
	{
		/* Check for end of list */
		if(pxCurrent == (PhofeesTaskListConfig_t *)0)
		{
			/* Last item found */
			break;
		}
		/* Next Task */
		ppxCurrent = (PhofeesTaskListConfig_t **)&pxCurrent->pvNext;
		pxCurrent = (PhofeesTaskListConfig_t *)pxCurrent->pvNext;
	}

	/* Initialize item */
	pxCurrent =
		(PhofeesTaskListConfig_t *)pvPortMalloc( sizeof(PhofeesTaskListConfig_t) );		/* Insert PhofeesTask list */
	vSemaphoreCreateBinary(pxCurrent->xSemaphore);								/* Create Task semaphore */
	pxCurrent->pvNext = (void *)0;												/* Set end of list marker */
	if(uxNumOfItems)
	{
		/* Create Queue for this Task */
		pxCurrent->xQueue = xQueueCreate(uxNumOfItems,uxItemSize);
		/* Assign to a Queue Destiny */
		pxCurrent->pxQueueHeader = (PhofeesMsgHeader_t *) pvPortMalloc( sizeof(PhofeesMsgHeader_t)*uxNumOfItems );
		pxCurrent->xQueueSize = uxNumOfItems;
		pxCurrent->xQueueSend = 0;
		pxCurrent->xQueueReceive = 0;
	}
	else
	{
		pxCurrent->xQueue = (xQueueHandle *) 0;									/* Assign to a Queue Handler */
		pxCurrent->pxQueueHeader = (PhofeesMsgHeader_t *)0;								/* Assign to a Queue Destiny */
	}
	/* Save back to the list */
	*ppxCurrent = pxCurrent;
	return (PhofeesTaskListHandle_t)pxCurrent;
}

PhofeesTaskHandle_t xPhofeesTaskCreate( PhofeesTaskListHandle_t xPhofeesTaskList, PhofeesTaskId_t xPhofeesTaskId,
		pdPHOFEESTASK_CODE xCodeFunction, unsigned short usContextSize, unsigned char ucExtTimerNbr)
{
	PhofeesTaskConfig_t **ppxCurrent;
	PhofeesTaskConfig_t *pxCurrent;
	PhofeesTaskListConfig_t *pxTask;
	pxTask = (PhofeesTaskListConfig_t*) xPhofeesTaskList;
	ppxCurrent =  (PhofeesTaskConfig_t **)&pxTask->pxPhofeesTaskHead;
	pxCurrent = (PhofeesTaskConfig_t *)pxTask->pxPhofeesTaskHead;
	unsigned int ulCount;

	/* Find last PhofeesTask item */
	for(;;)
	{
		/* Check for end of list */
		if(pxCurrent == (PhofeesTaskConfig_t *)0)
		{
			/* Last item found */
			break;
		}
		/* Next PhofeesTask */
		ppxCurrent = (PhofeesTaskConfig_t **)&pxCurrent->pvNext;
		pxCurrent = (PhofeesTaskConfig_t *)pxCurrent->pvNext;
	}

	/* Initialize item */
	pxCurrent = (PhofeesTaskConfig_t *)pvPortMalloc( sizeof(PhofeesTaskConfig_t) );		/* Insert PhofeesTask */
	if(ucExtTimerNbr>0)
	{
		pxCurrent->pxExtTimers = (PhofeesExtTimer_t *)pvPortMalloc( sizeof(PhofeesExtTimer_t)*ucExtTimerNbr );		/* Create Ext Timers */
		for(ulCount = 0; ulCount<ucExtTimerNbr; ulCount++)
		{
			pxCurrent->pxExtTimers[ulCount] = 0;
		}
	}
	else
	{
		pxCurrent->pxExtTimers = NULL;
	}
	if(usContextSize>0)
	{
		pxCurrent->pvContext = (void *)pvPortMalloc( usContextSize );		/* Create Ext Timers */
	}
	else
	{
		pxCurrent->pvContext = NULL;
	}
	pxCurrent->ucExtTimerNbr = ucExtTimerNbr;
	pxCurrent->xPhofeesTaskId = xPhofeesTaskId;											/* Configure PhofeesTask id */
	pxCurrent->xEventStatus = 0;												/* Clear status */
	pxCurrent->xTimer = 0;														/* Clear timer */
	pxCurrent->xCodeFunction = xCodeFunction;									/* Configure Code function */
	pxCurrent->pvNext = (void *)0;												/* Set end of list marker */
	/* Save back to the list */
	*ppxCurrent = pxCurrent;
	return pxCurrent;
}

void vPhofeesIncrementTick( PhofeesTimer_t xTicks )
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	unsigned portBASE_TYPE uxSavedInterruptStatus;
	unsigned char ucGiveSem;
	PhofeesTaskConfig_t *pxCurrent;
	PhofeesTaskListConfig_t *pxTaskCurrent = pxPhofeesTaskListsHead;

	/* Roll over all Tasks */
	for(;;)
	{
		/* Check for end of list */
		if(pxTaskCurrent == (PhofeesTaskListConfig_t *)0)
		{
			/* End of tasks */
			break;
		}
		/* Get PhofeesTasks list */
		pxCurrent = (PhofeesTaskConfig_t *)pxTaskCurrent->pxPhofeesTaskHead;
		ucGiveSem = 0;
		/* Roll over all PhofeesTasks */
		for(;;)
		{
			/* Check for end of list */
			if(pxCurrent == (PhofeesTaskConfig_t *)0)
			{
				/* End of PhofeesTasks */
				break;
			}
			/* Process timer count down */
			/* Timer just expired, generates event */
			if(pxCurrent->xTimer <= xTicks)
			{
				if(pxCurrent->xTimer)
				{
					uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
					pxCurrent->xEventStatus |= (1<<ePhofeesGbEvent_Timer);
					portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedInterruptStatus );
					pxCurrent->xTimer = 0;
				}
			}
			else
			{
				pxCurrent->xTimer -= xTicks;
			}
			/* Check for any event to this PhofeesTask */
			if(pxCurrent->xEventStatus)
			{
				ucGiveSem = 1;
			}
			/* Next PhofeesTask */
			pxCurrent = (PhofeesTaskConfig_t *)pxCurrent->pvNext;
		}
		/* If there was configured a Queue for this task, check it */
		if(pxTaskCurrent->xQueue)
		{
			/* Check for messages for this task */
			if(uxQueueMessagesWaitingFromISR(pxTaskCurrent->xQueue))
			{
				ucGiveSem = 1;
			}
		}
		/* Check if event was set for the task or PhofeesTask */
		if(ucGiveSem)
		{
			/* Gives the semaphore to the task */
			xSemaphoreGiveFromISR(pxTaskCurrent->xSemaphore,&xHigherPriorityTaskWoken);
			/* Now the buffer is empty we can switch context if necessary. */
			if( xHigherPriorityTaskWoken )
			{
				/* Actual macro used here is port specific. */
				portYIELD_FROM_ISR(1);
			}
		}
		/* Next Task */
		pxTaskCurrent = (PhofeesTaskListConfig_t *)pxTaskCurrent->pvNext;
	}
}

void vPhofeesTaskStartScheduler( PhofeesTaskListHandle_t xPhofeesTaskList )
{
	unsigned char ucNoEvents = 1;
	PhofeesTaskConfig_t *pxCurrent;
	PhofeesTaskListConfig_t *pxTask;
	pxTask = (PhofeesTaskListConfig_t*) xPhofeesTaskList;
	pxCurrent = (PhofeesTaskConfig_t *)pxTask->pxPhofeesTaskHead;
#ifdef DEBUG_MEASURE_TASK_MAX_TIME
	unsigned long ulMeasure = 0;
#endif

	/* Sanity check */
	if((pxTask == (PhofeesTaskListConfig_t *)0) || (pxCurrent == (PhofeesTaskConfig_t *)0))
		return;

	/* Just clear the semaphore before begin */
	xSemaphoreTake(pxTask->xSemaphore, 0 );

	for(;;)
	{
		/* Feed the dog*/
//		vApiWatchDogFeed();

		/* Check end of PhofeesTask list marker */
		if(pxCurrent == (PhofeesTaskConfig_t *)0)
		{
			/* No events detected on list let's wait in blocked mode */
			if(ucNoEvents)
			{
#ifdef DEBUG_MEASURE_TASK_MAX_TIME
				ulMeasure = ulFreeCounter-ulMeasure;
				if(ulMeasure>ulMaxTaskProcess)
					ulMaxTaskProcess = ulMeasure;
#endif
				/* Wait for semaphore */
				xSemaphoreTake(pxTask->xSemaphore, portMAX_DELAY );
#ifdef DEBUG_MEASURE_TASK_MAX_TIME
				ulMeasure = ulFreeCounter;
#endif
			}
			/* Restart the list */
			pxCurrent = (PhofeesTaskConfig_t *)pxTask->pxPhofeesTaskHead;
			ucNoEvents = 1;
		}
		/* If there was configured a Queue for this task, check it */
		if(pxTask->xQueue)
		{
			/* Check for messages for this task */
			taskENTER_CRITICAL();
			if(uxQueueMessagesWaitingFromISR(pxTask->xQueue))
			{
				/* Check for message for this PhofeesTask */
				if(pxCurrent->xPhofeesTaskId == pxTask->pxQueueHeader[pxTask->xQueueReceive].xPhofeesTaskDest)
					pxCurrent->xEventStatus |= (1<<ePhofeesGbEvent_Message);
			}
			taskEXIT_CRITICAL();
		}
		/* Check for any event for current PhofeesTask */
		if(pxCurrent->xEventStatus)
		{
			/* Let's call the PhofeesTask */
			pxCurrent->xCodeFunction( (void *)pxCurrent );
			/* At least one event detected, clear flag to sacn again */
			ucNoEvents = 0;
		}
		/* Go to next PhofeesTask */
		pxCurrent = (PhofeesTaskConfig_t *)pxCurrent->pvNext;
	}
}

void vPhofeesEventSet( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	taskENTER_CRITICAL();
	pxPhosfeesTaskConfig->xEventStatus |= (1<<cEventId);
	taskEXIT_CRITICAL();
}

void vPhofeesEventSetISR( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	pxPhosfeesTaskConfig->xEventStatus |= (1<<cEventId);
}

void vPhofeesEventClear( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	taskENTER_CRITICAL();
	pxPhosfeesTaskConfig->xEventStatus &= ~(1<<cEventId);
	taskEXIT_CRITICAL();
}

void vPhofeesEventClearISR( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	pxPhosfeesTaskConfig->xEventStatus &= ~(1<<cEventId);
}

signed portBASE_TYPE xPhofeesEventCheck( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	return ((pxPhosfeesTaskConfig->xEventStatus & (1<<cEventId)) != 0);
}

signed portBASE_TYPE xPhofeesEventCheckAll( PhofeesTaskHandle_t xPhofeesTask )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	return (pxPhosfeesTaskConfig->xEventStatus != 0);
}

void vPhofeesTimerSet( PhofeesTaskHandle_t xPhofeesTask , PhofeesTimer_t xTicks )
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	taskENTER_CRITICAL();
	pxPhosfeesTaskConfig->xTimer = xTicks;
	pxPhosfeesTaskConfig->xTimerLoaded = xTicks;
	taskEXIT_CRITICAL();
}

void vPhofeesExtTimerUpdate( PhofeesTaskHandle_t xPhofeesTask )
{
	PhofeesExtTimer_t xMin = PHOFEESTASK_TIMER_OVERFLOW;
	unsigned char ucCount;
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;

	for(ucCount=0;ucCount<pxPhosfeesTaskConfig->ucExtTimerNbr;ucCount++)
	{
		if(pxPhosfeesTaskConfig->pxExtTimers[ucCount])
		{
			if(pxPhosfeesTaskConfig->pxExtTimers[ucCount] < PHOFEESTASK_TIMER_OVERFLOW)
			{
				if(pxPhosfeesTaskConfig->pxExtTimers[ucCount] > pxPhosfeesTaskConfig->xTimerLoaded)
				{
					pxPhosfeesTaskConfig->pxExtTimers[ucCount] -= pxPhosfeesTaskConfig->xTimerLoaded;
				}
				else
				{
					pxPhosfeesTaskConfig->pxExtTimers[ucCount] = PHOFEESTASK_TIMER_OVERFLOW;
				}
				if(pxPhosfeesTaskConfig->pxExtTimers[ucCount]<xMin)
				{
					xMin = pxPhosfeesTaskConfig->pxExtTimers[ucCount];
				}
			}
		}
	}
	if(xMin < PHOFEESTASK_TIMER_OVERFLOW)
	{
		if(xMin > 0xFFFF)
			xMin = 0xFFFF;
	}
	else
	{
		xMin = 0;
	}
	taskENTER_CRITICAL();
	pxPhosfeesTaskConfig->xEventStatus &= ~(1<<ePhofeesGbEvent_Timer);
	pxPhosfeesTaskConfig->xTimer = xMin;
	pxPhosfeesTaskConfig->xTimerLoaded = xMin;
	taskEXIT_CRITICAL();
}

void vPhofeesExtTimerSetGen( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId, PhofeesExtTimer_t xExtTicks , unsigned char ucFromISR)
{
	PhofeesTimer_t xDelta;
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;

	if(pxPhosfeesTaskConfig->ucExtTimerNbr<=ucExtTimerId)
		return;
	pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId] = xExtTicks;
	/* If a timer is set */
	if(xExtTicks)
	{
		unsigned short usTimerEvt = pxPhosfeesTaskConfig->xEventStatus;
		usTimerEvt = usTimerEvt & (1<<ePhofeesGbEvent_Timer);
		/* Limit the cycle to a 16bit variable (timer length) */
		if(xExtTicks > 0xFFFF)
			xExtTicks = 0xFFFF;
		if(!ucFromISR)
			taskENTER_CRITICAL();
		/* No timer is set yet, lets load it */
		if((!pxPhosfeesTaskConfig->xTimer) && (usTimerEvt==0))
		{
			pxPhosfeesTaskConfig->xTimer = xExtTicks;
			pxPhosfeesTaskConfig->xTimerLoaded = xExtTicks;
		}
		/* We have a timer running, let's adjust the base */
		else
		{
			/* Calculates how much time is passed by since the timer was set */
			xDelta = pxPhosfeesTaskConfig->xTimerLoaded - pxPhosfeesTaskConfig->xTimer;
			/* Add that to this timer, it will be forcedly decremented from it */
			pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId] += xDelta;
//			if(pxPhofeesTask->xExtTimers[ucExtTimerId]>(PhofeesTaskExtTimer_t)0x0FFFFFFF)
//			{
//				__error(ERROR_PRT_TRACE,"[TAF]");
//			}
			/* New timer is smaller than current one, let's change the timer */
			if(pxPhosfeesTaskConfig->xTimer > xExtTicks)
			{
				/* For a internal delta we are interested on limited variable */
				xDelta += xExtTicks;
				/* Adjust the spent time when timer finishes */
				pxPhosfeesTaskConfig->xTimerLoaded = xDelta;
				/* Load the new period */
				pxPhosfeesTaskConfig->xTimer = xExtTicks;
			}
		}
		if(!ucFromISR)
			taskEXIT_CRITICAL();
	}
}

signed portBASE_TYPE xPhofeesExtTimerCheck( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId)
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;

	if(pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId] == PHOFEESTASK_TIMER_OVERFLOW)
	{
		pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId] = 0;
		return 1;
	}
	return 0;
}

signed portBASE_TYPE xPhofeesExtTimerIsStopped( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId)
{
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;

	if(pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId] == 0)
	{
		return 1;
	}
	return 0;
}

PhofeesExtTimer_t xPhofeesExtTimerChronReadGen( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId , unsigned char ucFromISR)
{
	PhofeesExtTimer_t ulChron;
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;

	if(!ucFromISR)
		taskENTER_CRITICAL();

	// Loaded value
	ulChron  = PHOFEESTASK_TIMER_CHRON_START;
	// Decrement remaining time to expire
	ulChron -= pxPhosfeesTaskConfig->pxExtTimers[ucExtTimerId];
	// Add passed time from current period
	ulChron += (pxPhosfeesTaskConfig->xTimerLoaded - pxPhosfeesTaskConfig->xTimer);

	if(!ucFromISR)
		taskEXIT_CRITICAL();

	return ulChron;
}


signed portBASE_TYPE xPhofeesMsgGenericSend( PhofeesMsgHeader_t xMsgHeader, const void * const pvItemToQueue, portBASE_TYPE xCopyPosition )
{
	signed portBASE_TYPE ret, xHigherPriorityTaskWoken;
	PhofeesTaskListConfig_t *pxTaskCurrent;
	pxTaskCurrent = pxPhofeesTaskGetListFromPhofeesTaskId(xMsgHeader.xPhofeesTaskDest);

	/* Sanity check */
	if((pxTaskCurrent == (PhofeesTaskListConfig_t *)0) || (pxTaskCurrent->xQueue == (xQueueHandle)0))
		return errNO_TASK_TO_RUN;

	taskENTER_CRITICAL();
	/* Send message using native Queue Handler (ISR to do not touch critical context) */
	ret = xQueueGenericSendFromISR(pxTaskCurrent->xQueue, pvItemToQueue, &xHigherPriorityTaskWoken, xCopyPosition);

	/* Check for errors */
	if(ret == pdPASS)
	{
		/* Message sent, just put receiver control */
		if(xCopyPosition == queueSEND_TO_BACK)
		{
			/* Put PhofeesTaskID in queue */
			pxTaskCurrent->pxQueueHeader[pxTaskCurrent->xQueueSend] = xMsgHeader;
			/* Post increment pointer */
			pxTaskCurrent->xQueueSend++;
			/* Simply roll the buffer, no need to test for space, native queue controls that */
			if(pxTaskCurrent->xQueueSend >= pxTaskCurrent->xQueueSize)
				pxTaskCurrent->xQueueSend = 0;
		}
		else
		{
			/* Pre decrement Receiver pointer to put it on front of queue */
			pxTaskCurrent->xQueueReceive--;
			/* Simply roll the buffer, no need to test for space, native queue controls that */
			/* Check for overflow of unsigned variable */
			if(pxTaskCurrent->xQueueReceive >= pxTaskCurrent->xQueueSize)
				pxTaskCurrent->xQueueReceive = pxTaskCurrent->xQueueSize-1;
			/* Put PhofeesTaskID in queue */
			pxTaskCurrent->pxQueueHeader[pxTaskCurrent->xQueueReceive] = xMsgHeader;
		}

	}
//	else
//	{
//		__error(ERROR_PRT_CRITICAL,"xPhofeesMsgGenericSend: FIFO FULL");
//	}
	taskEXIT_CRITICAL();
	return ret;
}

PhofeesMsgHeader_t xPhofeesMsgReceive( PhofeesTaskHandle_t xPhofeesTask, void * const pvBuffer )
{
	PhofeesMsgHeader_t header;
	signed portBASE_TYPE xTaskWoken, ret;
	PhofeesTaskListConfig_t *pxTaskCurrent;
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;

	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTask;
	pxTaskCurrent = pxPhofeesTaskGetListFromPhofeesTaskId(pxPhosfeesTaskConfig->xPhofeesTaskId);

	header.xPhofeesMsgId = 0;

	/* Sanity check */
	if((pxTaskCurrent == (PhofeesTaskListConfig_t *)0) || (pxTaskCurrent->xQueue == (xQueueHandle)0))
		return header;

	taskENTER_CRITICAL();
	if(pxTaskCurrent->pxQueueHeader[pxTaskCurrent->xQueueReceive].xPhofeesTaskDest==pxPhosfeesTaskConfig->xPhofeesTaskId)
	{
		/* Get the message from native Queue Handler (ISR to do not touch critical context) */
		ret = xQueueReceiveFromISR(pxTaskCurrent->xQueue, pvBuffer, &xTaskWoken);
		/* Check for errors */
		if(ret == pdPASS)
		{
			header = pxTaskCurrent->pxQueueHeader[pxTaskCurrent->xQueueReceive];
			/* Discard the message */
			pxTaskCurrent->xQueueReceive++;
			/* Simply roll the buffer, no need to test for space, native queue controls that */
			if(pxTaskCurrent->xQueueReceive >= pxTaskCurrent->xQueueSize)
				pxTaskCurrent->xQueueReceive = 0;

		}
	}
	taskEXIT_CRITICAL();
	return header;
}

PhofeesMsgHeader_t xPhofeesMsgFillHeader( PhofeesTaskHandle_t xPhofeesTaskSource, PhofeesTaskId_t xPhofeesTaskDestId , PhofeesMsgId_t xMsgId )
{
	PhofeesMsgHeader_t header;
	PhofeesTaskConfig_t *pxPhosfeesTaskConfig;
	pxPhosfeesTaskConfig = (PhofeesTaskConfig_t*)xPhofeesTaskSource;
	header.xPhofeesMsgId = xMsgId;
	header.xPhofeesTaskDest = xPhofeesTaskDestId;
	header.xPhofeesTaskSource = pxPhosfeesTaskConfig->xPhofeesTaskId;
	return header;
}

/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
	vPhofeesIncrementTick(1);
#if ( configUSE_PHOFEES_TICK_HOOK == 1 )
	vPhofeesApplicationTickHook();
#endif

}
