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
#ifndef __PHOFEES_H
#define __PHOFEES_H

#include <stddef.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "Phofees_types.h"

typedef void * PhofeesTaskHandle_t;
typedef void PhofeesTaskContext_t;
typedef void * PhofeesTaskListHandle_t;

/**************** Functions prototypes ***********************/
/* Phofees PhofeesTask Control functions */
PhofeesTaskListHandle_t xPhofeesTaskListCreate( PhofeesTaskQueueSize_t uxNumOfItems, unsigned short uxItemSize );
PhofeesTaskHandle_t xPhofeesTaskCreate( PhofeesTaskListHandle_t xPhofeesTaskList, PhofeesTaskId_t xPhofeesTaskId, pdPHOFEESTASK_CODE xCodeFunction, unsigned short usContextSize, unsigned char ucExtTimerNbr);
void vPhofeesTaskStartScheduler( PhofeesTaskListHandle_t xPhofeesTaskList );
void vPhofeesIncrementTick( PhofeesTimer_t xTicks );
PhofeesTaskHandle_t xPhofeesTaskGetHandle( PhofeesTaskId_t xPhofeesTaskId );
PhofeesTaskContext_t *pxPhofeesTaskGetContext(PhofeesTaskHandle_t xPhofeesTask);

/* PhofeesTask Event functions */
void vPhofeesEventSet( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId );
void vPhofeesEventSetISR( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId );
void vPhofeesEventClear( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId );
void vPhofeesEventClearISR( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId );
signed portBASE_TYPE xPhofeesEventCheck( PhofeesTaskHandle_t xPhofeesTask , unsigned char cEventId );
signed portBASE_TYPE xPhofeesEventCheckAll( PhofeesTaskHandle_t xPhofeesTask );
#define vPhofeesEventSetId(xPhofeesTaskId,cEventId)			vPhofeesEventSet(xPhofeesTaskGetHandle(xPhofeesTaskId),cEventId)
#define vPhofeesEventSetIdISR(xPhofeesTaskId,cEventId)		vPhofeesEventSetISR(xPhofeesTaskGetHandle(xPhofeesTaskId),cEventId)
#define vPhofeesEventClearId(xPhofeesTaskId,cEventId)		vPhofeesEventClear(xPhofeesTaskGetHandle(xPhofeesTaskId),cEventId)
#define vPhofeesEventClearIdISR(xPhofeesTaskId,cEventId)	vPhofeesEventClearISR(xPhofeesTaskGetHandle(xPhofeesTaskId),cEventId)
#define xPhofeesEventCheckId(xPhofeesTaskId,cEventId)		xPhofeesEventCheck(xPhofeesTaskGetHandle(xPhofeesTaskId),cEventId)
#define xPhofeesEventCheckAllId(xPhofeesTaskId)				xPhofeesEventCheckAll(xPhofeesTaskGetHandle(xPhofeesTaskId))

/* PhofeesTask Timer functions */
void vPhofeesTimerSet( PhofeesTaskHandle_t xPhofeesTask , PhofeesTimer_t xTicks );
#define vPhofeesTimerSetId(xPhofeesTaskId,xTicks)			vPhofeesTimerSet(xPhofeesTaskGetHandle(xPhofeesTaskId),xTicks);

/* PhofeesTask Extended Timer functions */
void vPhofeesExtTimerUpdate( PhofeesTaskHandle_t xPhofeesTask );
void vPhofeesExtTimerSetGen( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId, PhofeesExtTimer_t xExtTicks , unsigned char ucFromISR);
signed portBASE_TYPE xPhofeesExtTimerCheck( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId);
signed portBASE_TYPE xPhofeesExtTimerIsStopped( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId);
#define vPhofeesExtTimerSet(xPhofeesTask,ucExtTimerId,xExtTicks) 			vPhofeesExtTimerSetGen( xPhofeesTask , ucExtTimerId, xExtTicks, 0)
#define vPhofeesExtTimerSetISR(xPhofeesTask,ucExtTimerId,xExtTicks) 		vPhofeesExtTimerSetGen( xPhofeesTask , ucExtTimerId, xExtTicks, 1)
#define vPhofeesExtTimerSetId(xPhofeesTaskId,ucExtTimerId,xExtTicks) 		vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId) , ucExtTimerId, xExtTicks, 0)
#define vPhofeesExtTimerSetIdISR(xPhofeesTaskId,ucExtTimerId,xExtTicks)		vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId) , ucExtTimerId, xExtTicks, 1)
#define vPhofeesExtTimerStop(xPhofeesTask,ucExtTimerId)		 				vPhofeesExtTimerSetGen( xPhofeesTask , ucExtTimerId, 0, 0)
#define vPhofeesExtTimerStopISR(xPhofeesTask,ucExtTimerId) 					vPhofeesExtTimerSetGen( xPhofeesTask , ucExtTimerId, 0, 1)
#define vPhofeesExtTimerStopId(xPhofeesTaskId,ucExtTimerId) 				vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId) , ucExtTimerId, 0, 0)
#define vPhofeesExtTimerStopIdISR(xPhofeesTaskId,ucExtTimerId)				vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId) , ucExtTimerId, 0, 1)

PhofeesExtTimer_t xPhofeesExtTimerChronReadGen( PhofeesTaskHandle_t xPhofeesTask , unsigned char ucExtTimerId , unsigned char ucFromISR);
#define vPhofeesExtTimerChronStart(xPhofeesTask,ucExtTimerId) 			vPhofeesExtTimerSetGen( xPhofeesTask, ucExtTimerId, PhofeesTask_TIMER_CHRON_START ,0)
#define vPhofeesExtTimerChronStop(xPhofeesTask,ucExtTimerId) 			vPhofeesExtTimerSetGen( xPhofeesTask, ucExtTimerId, 0, 0)
#define vPhofeesExtTimerChronStartId(xPhofeesTaskId,ucExtTimerId) 		vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , PhofeesTask_TIMER_CHRON_START ,0 )
#define vPhofeesExtTimerChronStopId(xPhofeesTaskId,ucExtTimerId) 		vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , 0 , 0)
#define vPhofeesExtTimerChronStartISR(xPhofeesTask,ucExtTimerId) 		vPhofeesExtTimerSetGen( xPhofeesTask, ucExtTimerId, PhofeesTask_TIMER_CHRON_START ,1)
#define vPhofeesExtTimerChronStopISR(xPhofeesTask,ucExtTimerId) 		vPhofeesExtTimerSetGen( xPhofeesTask, ucExtTimerId, 0, 1)
#define vPhofeesExtTimerChronStartIdISR(xPhofeesTaskId,ucExtTimerId) 	vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , PhofeesTask_TIMER_CHRON_START ,1 )
#define vPhofeesExtTimerChronStopIdISR(xPhofeesTaskId,ucExtTimerId) 	vPhofeesExtTimerSetGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , 0 , 1)
#define xPhofeesExtTimerChronRead(xPhofeesTask,ucExtTimerId) 			xPhofeesExtTimerChronReadGen( xPhofeesTask, ucExtTimerId , 0)
#define xPhofeesExtTimerChronReadId(xPhofeesTaskId,ucExtTimerId) 		xPhofeesExtTimerChronReadGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , 0)
#define xPhofeesExtTimerChronReadISR(xPhofeesTask,ucExtTimerId) 		xPhofeesExtTimerChronReadGen( xPhofeesTask, ucExtTimerId , 1)
#define xPhofeesExtTimerChronReadIdISR(xPhofeesTaskId,ucExtTimerId) 	xPhofeesExtTimerChronReadGen( xPhofeesTaskGetHandle(xPhofeesTaskId), ucExtTimerId , 1)

/* PhofeesTask Message functions */
signed portBASE_TYPE xPhofeesMsgGenericSend( PhofeesMsgHeader_t xMsgHeader, const void * const pvItemToQueue, portBASE_TYPE xCopyPosition );
PhofeesMsgHeader_t xPhofeesMsgReceive( PhofeesTaskHandle_t xPhofeesTask, void * const pvBuffer);
PhofeesMsgHeader_t xPhofeesMsgFillHeader( PhofeesTaskHandle_t xPhofeesTaskSource, PhofeesTaskId_t xPhofeesTaskDestId , PhofeesMsgId_t xMsgId );
#define xPhofeesMsgSend(xMsgHeader,pvItemToQueue)			xPhofeesMsgGenericSend (xMsgHeader,pvItemToQueue,queueSEND_TO_BACK)
#define xPhofeesMsgSendFront(xMsgHeader,pvItemToQueue)		xPhofeesMsgGenericSend (xMsgHeader,pvItemToQueue,queueSEND_TO_FRONT)

#endif /* __PHOFEES_H */
