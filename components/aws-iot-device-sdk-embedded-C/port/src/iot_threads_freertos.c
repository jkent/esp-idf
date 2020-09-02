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
 * @file iot_threads_posix.c
 * @brief Implementation of the functions in iot_threads.h for POSIX systems.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <stdlib.h>

/* FreeRTOS includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* Platform threads include. */
#include "platform/iot_threads.h"

/* Error handling include. */
#include "iot_error.h"

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

#define LIBRARY_LOG_NAME    ( "THREAD" )
#include "iot_logging_setup.h"

/*
 * Provide default values for undefined memory allocation functions.
 */
#ifndef IotThreads_Malloc
    #include <stdlib.h>

/**
 * @brief Memory allocation. This function should have the same signature
 * as [malloc](http://pubs.opengroup.org/onlinepubs/9699919799/functions/malloc.html).
 */
    #define IotThreads_Malloc    malloc
#endif
#ifndef IotThreads_Free
    #include <stdlib.h>

/**
 * @brief Free memory. This function should have the same signature as
 * [free](http://pubs.opengroup.org/onlinepubs/9699919799/functions/free.html).
 */
    #define IotThreads_Free    free
#endif

/* When building tests, the Unity framework's malloc overrides are used to track
 * calls to platform resource creation and destruction. This ensures that all
 * platform resources are destroyed before the tests finish. When not testing,
 * define the Unity malloc functions to nothing. */
#if IOT_BUILD_TESTS != 1
    #define UnityMalloc_AllocateResource()    true
    #define UnityMalloc_FreeResource()
#endif

/*-----------------------------------------------------------*/

bool Iot_CreateDetachedThread( IotThreadRoutine_t threadRoutine,
                               void * pArgument,
                               int32_t priority,
                               size_t stackSize )
{
    if( stackSize == IOT_THREAD_DEFAULT_STACK_SIZE || priority == IOT_THREAD_IGNORE_STACK_SIZE )
    {
        stackSize = 2048;
    }

    if( priority == IOT_THREAD_DEFAULT_PRIORITY || priority == IOT_THREAD_IGNORE_PRIORITY )
    {
        priority = tskIDLE_PRIORITY + 1;
    }

    bool status = xTaskCreate(threadRoutine, "iot", stackSize, pArgument,
                              priority, NULL) == pdTRUE;

    return status;
}

/*-----------------------------------------------------------*/

bool IotMutex_Create( IotMutex_t * pNewMutex,
                      bool recursive )
{
    IOT_FUNCTION_ENTRY( bool, true );

    /* Increment the number of platform resources in use. */
    status = UnityMalloc_AllocateResource();

    if( status == true )
    {
        pNewMutex = IotThreads_Malloc(sizeof(IotMutex_t));
        pNewMutex->recursive = recursive;

        if( recursive == true )
        {
            pNewMutex->mutex = xSemaphoreCreateRecursiveMutex();
        }
        else
        {
            pNewMutex->mutex = xSemaphoreCreateMutex();
        }

        if( pNewMutex->mutex == NULL ) {
            IotLogError( "Failed to initialize mutex." );

            UnityMalloc_FreeResource();
            IotThreads_Free( pNewMutex );
            IOT_SET_AND_GOTO_CLEANUP( false );
        }
    }

    IOT_FUNCTION_CLEANUP_BEGIN();
    IOT_FUNCTION_CLEANUP_END();
}

/*-----------------------------------------------------------*/

void IotMutex_Destroy( IotMutex_t * pMutex )
{
    /* Decrement the number of platform resources in use. */
    UnityMalloc_FreeResource();

    vSemaphoreDelete( pMutex->mutex );
    IotThreads_Free( pMutex );
}

/*-----------------------------------------------------------*/

void IotMutex_Lock( IotMutex_t * pMutex )
{
    if( pMutex->recursive )
    {
        xSemaphoreTakeRecursive( pMutex->mutex, portMAX_DELAY );
    }
    else
    {        
        xSemaphoreTake( pMutex->mutex, portMAX_DELAY );
    }
}

/*-----------------------------------------------------------*/

bool IotMutex_TryLock( IotMutex_t * pMutex )
{
    if( pMutex->recursive )
    {
        return xSemaphoreTakeRecursive( pMutex->mutex, 0 );
    }
    else
    {        
        return xSemaphoreTake( pMutex->mutex, 0 );
    }
}

/*-----------------------------------------------------------*/

void IotMutex_Unlock( IotMutex_t * pMutex )
{
    if( pMutex->recursive )
    {
        xSemaphoreGiveRecursive( pMutex->mutex );
    }
    else
    {
        xSemaphoreGive( pMutex->mutex );
    }
}

/*-----------------------------------------------------------*/

bool IotSemaphore_Create( IotSemaphore_t * pNewSemaphore,
                          uint32_t initialValue,
                          uint32_t maxValue )
{
    bool status = UnityMalloc_AllocateResource();

    if( status == true )
    {
        pNewSemaphore = xQueueCreateCountingSemaphore( maxValue, initialValue );

        if( pNewSemaphore == NULL )
        {
            IotLogError( "Failed to create new semaphore." );

            UnityMalloc_FreeResource();
            status = false;
        }

    }
    return status;
}

/*-----------------------------------------------------------*/

uint32_t IotSemaphore_GetCount( IotSemaphore_t * pSemaphore )
{
    return ( uint32_t ) uxSemaphoreGetCount( pSemaphore );
}

/*-----------------------------------------------------------*/

void IotSemaphore_Destroy( IotSemaphore_t * pSemaphore )
{
    /* Decrement the number of platform resources in use. */
    UnityMalloc_FreeResource();

    vSemaphoreDelete( pSemaphore );
}

/*-----------------------------------------------------------*/

void IotSemaphore_Wait( IotSemaphore_t * pSemaphore )
{
    xSemaphoreTake( pSemaphore, portMAX_DELAY );
}

/*-----------------------------------------------------------*/

bool IotSemaphore_TryWait( IotSemaphore_t * pSemaphore )
{
    return xSemaphoreTake( pSemaphore, portMAX_DELAY ) == pdTRUE;
}

/*-----------------------------------------------------------*/

bool IotSemaphore_TimedWait( IotSemaphore_t * pSemaphore,
                             uint32_t timeoutMs )
{
    return xSemaphoreTake( pSemaphore, timeoutMs / portTICK_PERIOD_MS ) == pdTRUE;
}

/*-----------------------------------------------------------*/

void IotSemaphore_Post( IotSemaphore_t * pSemaphore )
{
    xSemaphoreGive( pSemaphore );
}

/*-----------------------------------------------------------*/
