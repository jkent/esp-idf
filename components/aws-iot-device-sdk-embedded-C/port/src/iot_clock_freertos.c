/*
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file iot_clock_posix.c
 * @brief Implementation of the functions in iot_clock.h for POSIX systems.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <stdlib.h>

#include "esp_log.h"

/* FreeRTOS includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/* POSIX includes. Allow the default POSIX headers to be overridden. */
#ifdef POSIX_TIME_HEADER
    #include POSIX_TIME_HEADER
#else
    #include <time.h>
#endif

/* Platform clock include. */
#include "platform/iot_clock.h"

/* Configure logs for the functions in this file. */
#ifdef IOT_LOG_LEVEL_PLATFORM
    #define LIBRARY_LOG_LEVEL        IOT_LOG_LEVEL_PLATFORM
#else
    #ifdef IOT_LOG_LEVEL_GLOBAL
        #define LIBRARY_LOG_LEVEL    IOT_LOG_LEVEL_GLOBAL
    #else
        #define LIBRARY_LOG_LEVEL    IOT_LOG_NONE
    #endif
#endif

#define LIBRARY_LOG_NAME    ( "CLOCK" )
#include "iot_logging_setup.h"

/* When building tests, the Unity framework's malloc overrides are used to track
 * calls to platform resource creation and destruction. This ensures that all
 * platform resources are destroyed before the tests finish. When not testing,
 * define the Unity malloc functions to nothing. */
#if IOT_BUILD_TESTS != 1
    #define UnityMalloc_AllocateResource()    true
    #define UnityMalloc_FreeResource()
#endif

/*-----------------------------------------------------------*/

/**
 * @brief Wraps an #IotThreadRoutine_t with a FreeRTOS-compliant one.
 *
 * @param[in] argument The value passed as `TimerHandle_t`.
 */
static void _timerExpirationWrapper( TimerHandle_t xTimer )
{
    IotTimer_t * pTimer = ( IotTimer_t * ) pvTimerGetTimerID( xTimer );

    /* Schedule another timer if period is > 0 */
    if( pTimer->periodMs > 0 )
    {
        xTimerChangePeriod( xTimer, pTimer->periodMs / portTICK_PERIOD_MS, portMAX_DELAY );
        xTimerStart( xTimer, portMAX_DELAY );
    }

    /* Call the wrapped thread routine. */
    pTimer->threadRoutine( pTimer->pArgument );
}

/*-----------------------------------------------------------*/

bool IotClock_GetTimestring( char * pBuffer,
                             size_t bufferSize,
                             size_t * pTimestringLength )
{
    uint32_t timestamp = esp_log_timestamp();

    *pTimestringLength = snprintf(pBuffer, bufferSize, "%u", timestamp);

    return true;
}

/*-----------------------------------------------------------*/

uint64_t IotClock_GetTimeMs( void )
{
    TickType_t ticks = xTaskGetTickCount();

    return ticks / portTICK_PERIOD_MS;
}

/*-----------------------------------------------------------*/

void IotClock_SleepMs( uint32_t sleepTimeMs )
{
    vTaskDelay( sleepTimeMs / portTICK_PERIOD_MS );
}

/*-----------------------------------------------------------*/

bool IotClock_TimerCreate( IotTimer_t * pNewTimer,
                           IotThreadRoutine_t expirationRoutine,
                           void * pArgument )
{
    bool status = UnityMalloc_AllocateResource();

    if( status == true )
    {
        IotLogDebug( "Creating new timer %p.", pNewTimer );

        /* Set the timer expiration routione and argument. */
        pNewTimer->threadRoutine = expirationRoutine;
        pNewTimer->pArgument = pArgument;

        pNewTimer->timer = xTimerCreate( "iot", 1000 / portTICK_PERIOD_MS, pdFALSE, pNewTimer, _timerExpirationWrapper );
        if( pNewTimer->timer == NULL )
        {
            IotLogError( "Failed to create new timer %p.", pNewTimer );
            UnityMalloc_FreeResource();
            status = false;
        }
    }

    return status;
}

/*-----------------------------------------------------------*/

void IotClock_TimerDestroy( IotTimer_t * pTimer )
{
    IotLogDebug( "Destroying timer %p.", pTimer );

    /* Decrement the number of platform resources in use. */
    UnityMalloc_FreeResource();

    if( xTimerDelete( pTimer->timer, portMAX_DELAY ) != pdPASS )
    {
        /* This block should not be reached; log an error and abort if it is. */
        IotLogError( "Failed to destroy timer %p.", pTimer );
        abort();
    }
}

/*-----------------------------------------------------------*/

bool IotClock_TimerArm( IotTimer_t * pTimer,
                        uint32_t relativeTimeoutMs,
                        uint32_t periodMs )
{
    bool status = true;

    IotLogDebug( "Arming timer %p with timeout %lu and period %lu.",
                 pTimer,
                 relativeTimeoutMs,
                 periodMs );

    pTimer->periodMs = periodMs;

    if( xTimerChangePeriod( pTimer->timer, relativeTimeoutMs / portTICK_PERIOD_MS, portMAX_DELAY ) != pdPASS )
    {
        IotLogError( "Unable to change timer period." );

        status = false;
    }

    if( status == true )
    {
        if( xTimerStart( pTimer->timer, portMAX_DELAY ) != pdPASS )
        {
            IotLogError( "Unable to start timer." );

            status = false;
        }
    }

    return status;
}

/*-----------------------------------------------------------*/
