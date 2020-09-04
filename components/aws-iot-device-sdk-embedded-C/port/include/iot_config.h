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

#ifndef IOT_CONFIG_H_
#define IOT_CONFIG_H_

#define AWS_IOT_MQTT_ENABLE_METRICS ( 0 )

/* Enable asserts in the libraries. */
#define IOT_CONTAINERS_ENABLE_ASSERTS                                     ( 1 )
#define IOT_MQTT_ENABLE_ASSERTS                                           ( 1 )
#define IOT_TASKPOOL_ENABLE_ASSERTS                                       ( 1 )
#define AWS_IOT_SHADOW_ENABLE_ASSERTS                                     ( 1 )
#define AWS_IOT_DEFENDER_ENABLE_ASSERTS                                   ( 1 )
#define AWS_IOT_JOBS_ENABLE_ASSERTS                                       ( 1 )
#define AWS_IOT_PROVISIONING_ENABLE_ASSERTS                               ( 1 )

/* Library logging configuration. IOT_LOG_LEVEL_GLOBAL provides a global log
 * level for all libraries; the library-specific settings override the global
 * setting. If both the library-specific and global settings are undefined,
 * no logs will be printed. */
#define IOT_LOG_LEVEL_GLOBAL                                              IOT_LOG_INFO
#define IOT_LOG_LEVEL_DEMO                                                IOT_LOG_INFO
#define IOT_LOG_LEVEL_PLATFORM                                            IOT_LOG_DEBUG//IOT_LOG_NONE
#define IOT_LOG_LEVEL_NETWORK                                             IOT_LOG_INFO
#define IOT_LOG_LEVEL_TASKPOOL                                            IOT_LOG_NONE
#define IOT_LOG_LEVEL_MQTT                                                IOT_LOG_INFO
#define AWS_IOT_LOG_LEVEL_SHADOW                                          IOT_LOG_INFO
#define AWS_IOT_LOG_LEVEL_DEFENDER                                        IOT_LOG_INFO
#define AWS_IOT_LOG_LEVEL_JOBS                                            IOT_LOG_INFO
#define AWS_IOT_LOG_LEVEL_PROVISIONING                                    IOT_LOG_INFO

/* Default assert and memory allocation functions. */
#include <assert.h>
#include <stdlib.h>

#define Iot_DefaultAssert    assert
#define Iot_DefaultMalloc    malloc
#define Iot_DefaultFree      free

/* The build system will choose the appropriate system types file for the platform
 * layer based on the host operating system. */
#include "iot_platform_types_freertos.h"

#endif /* ifndef IOT_CONFIG_H_ */
