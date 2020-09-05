set(SDK_DIR aws-iot-device-sdk-embedded-C)

set(include_dirs ./port/include
                 ${SDK_DIR}/libraries
                 ${SDK_DIR}/libraries/aws/common/include
                 ${SDK_DIR}/libraries/aws/defender/include
                 ${SDK_DIR}/libraries/aws/jobs/include
                 ${SDK_DIR}/libraries/aws/provisioning/include
                 ${SDK_DIR}/libraries/aws/shadow/include
                 ${SDK_DIR}/libraries/platform
                 ${SDK_DIR}/libraries/standard/common/include
                 ${SDK_DIR}/libraries/standard/mqtt/include
                 ${SDK_DIR}/libraries/standard/serializer/include
                 ${SDK_DIR}/ports/common/include
)

set(priv_include_dirs ${SDK_DIR}/libraries/aws/defender/src/private
                      ${SDK_DIR}/libraries/aws/jobs/src/private
                      ${SDK_DIR}/libraries/aws/provisioning/src/private
                      ${SDK_DIR}/libraries/aws/shadow/src/private
                      ${SDK_DIR}/libraries/standard/common/src/private
                      ${SDK_DIR}/libraries/standard/mqtt/src/private
)

set(srcs
         ${SDK_DIR}/libraries/aws/common/src/aws_iot_doc_parser.c
         ${SDK_DIR}/libraries/aws/common/src/aws_iot_operation.c
         ${SDK_DIR}/libraries/aws/common/src/aws_iot_parser.c
         ${SDK_DIR}/libraries/aws/common/src/aws_iot_subscription.c
         ${SDK_DIR}/libraries/aws/common/src/aws_iot_validate.c

         ${SDK_DIR}/libraries/aws/defender/src/aws_iot_defender_api.c
         ${SDK_DIR}/libraries/aws/defender/src/aws_iot_defender_collector.c
         ${SDK_DIR}/libraries/aws/defender/src/aws_iot_defender_mqtt.c

         ${SDK_DIR}/libraries/aws/jobs/src/aws_iot_jobs_api.c
         ${SDK_DIR}/libraries/aws/jobs/src/aws_iot_jobs_operation.c
         ${SDK_DIR}/libraries/aws/jobs/src/aws_iot_jobs_serialize.c
         ${SDK_DIR}/libraries/aws/jobs/src/aws_iot_jobs_static_memory.c
         ${SDK_DIR}/libraries/aws/jobs/src/aws_iot_jobs_subscription.c
         
         ${SDK_DIR}/libraries/aws/provisioning/src/aws_iot_provisioning_api.c
         ${SDK_DIR}/libraries/aws/provisioning/src/aws_iot_provisioning_parser.c
         ${SDK_DIR}/libraries/aws/provisioning/src/aws_iot_provisioning_serializer.c
         
         ${SDK_DIR}/libraries/aws/shadow/src/aws_iot_shadow_api.c
         ${SDK_DIR}/libraries/aws/shadow/src/aws_iot_shadow_operation.c
         ${SDK_DIR}/libraries/aws/shadow/src/aws_iot_shadow_parser.c
         ${SDK_DIR}/libraries/aws/shadow/src/aws_iot_shadow_static_memory.c
         ${SDK_DIR}/libraries/aws/shadow/src/aws_iot_shadow_subscription.c

         ${SDK_DIR}/libraries/standard/common/src/iot_init.c
         ${SDK_DIR}/libraries/standard/common/src/iot_static_memory_common.c
         ${SDK_DIR}/libraries/standard/common/src/iot_taskpool.c
         ${SDK_DIR}/libraries/standard/common/src/iot_taskpool_static_memory.c

         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_api.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_helper.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_lightweight_api.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_network.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_operation.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_serialize.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_static_memory.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_subscription.c
         ${SDK_DIR}/libraries/standard/mqtt/src/iot_mqtt_validate.c

         ${SDK_DIR}/libraries/standard/serializer/src/cbor/iot_serializer_tinycbor_decoder.c
         ${SDK_DIR}/libraries/standard/serializer/src/cbor/iot_serializer_tinycbor_encoder.c
         ${SDK_DIR}/libraries/standard/serializer/src/iot_serializer_static_memory.c

         ${SDK_DIR}/ports/common/src/iot_network_mbedtls.c
         ${SDK_DIR}/ports/common/src/iot_network_metrics.c

         port/src/iot_clock_freertos.c
         port/src/iot_logging.c
         port/src/iot_threads_freertos.c
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_INCLUDE_DIRS ${priv_include_dirs}
                       REQUIRES cbor
                                mbedtls)
