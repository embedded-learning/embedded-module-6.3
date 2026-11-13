#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "soc/adc_channel.h"

typedef struct {
    adc_unit_t adcUnit;
    adc_channel_t adcChannel;
    adc_atten_t adcAttenuation;
    adc_bitwidth_t adcBitwidth;
    bool isInverted;
    int32_t percentMin;
    int32_t percentMax;
} LightSensorConfiguration;

typedef struct {
    int32_t celsiusMin;
    int32_t celsiusMax;
} TemperatureSensorConfiguration;

typedef struct {
    UBaseType_t length;
    uint32_t sendTimeoutMs;
} MeasurementQueueConfiguration;

typedef struct {
    uint32_t stackSizeBytes;
    UBaseType_t sensorTaskPriority;
    UBaseType_t printTaskPriority;
} TaskConfiguration;

typedef enum {
    SENSOR_KIND_LIGHT,
    SENSOR_KIND_TEMPERATURE,
    SENSOR_KIND_COUNT,
} SensorKind;

typedef struct {
    SensorKind sensorKind;
    int32_t value;
} Measurement;

typedef esp_err_t (*ReadSensorValueFunction)(int32_t *sensorValue);

typedef struct {
    const char *label;
    const char *unit;
    uint32_t pollPeriodMs;
    ReadSensorValueFunction readSensorValue;
} SensorDescription;

static esp_err_t readLightPercent(int32_t *lightPercent);
static esp_err_t generateTemperatureCelsius(int32_t *temperatureCelsius);

static const LightSensorConfiguration lightSensorConfiguration = {
    .adcUnit = ADC_UNIT_1,
    .adcChannel = ADC1_GPIO1_CHANNEL,
    .adcAttenuation = ADC_ATTEN_DB_12,
    .adcBitwidth = ADC_BITWIDTH_12,
    .isInverted = false,
    .percentMin = 0,
    .percentMax = 100,
};

static const TemperatureSensorConfiguration temperatureSensorConfiguration = {
    .celsiusMin = -30,
    .celsiusMax = 40,
};

static const MeasurementQueueConfiguration measurementQueueConfiguration = {
    .length = 8,
    .sendTimeoutMs = 100,
};

static const TaskConfiguration taskConfiguration = {
    .stackSizeBytes = 4096,
    .sensorTaskPriority = 5,
    .printTaskPriority = 6,
};

static const SensorDescription sensorDescriptions[SENSOR_KIND_COUNT] = {
    [SENSOR_KIND_LIGHT] = {
        .label = "light",
        .unit = "%",
        .pollPeriodMs = 1000,
        .readSensorValue = readLightPercent,
    },
    [SENSOR_KIND_TEMPERATURE] = {
        .label = "temperature",
        .unit = "°C",
        .pollPeriodMs = 2000,
        .readSensorValue = generateTemperatureCelsius,
    },
};

static const char *LOG_TAG = "main";

static adc_oneshot_unit_handle_t lightSensorAdcHandle;
static QueueHandle_t measurementQueue;

static int32_t calculateRawValueMax(void)
{
    return (1 << lightSensorConfiguration.adcBitwidth) - 1;
}

static int32_t convertRawValueToLightPercent(int rawValue)
{
    int32_t voltagePercent = rawValue * lightSensorConfiguration.percentMax / calculateRawValueMax();
    if (voltagePercent < lightSensorConfiguration.percentMin) {
        voltagePercent = lightSensorConfiguration.percentMin;
    }
    if (voltagePercent > lightSensorConfiguration.percentMax) {
        voltagePercent = lightSensorConfiguration.percentMax;
    }
    return lightSensorConfiguration.isInverted ? lightSensorConfiguration.percentMax - voltagePercent : voltagePercent;
}

static esp_err_t readLightPercent(int32_t *lightPercent)
{
    int rawValue = 0;
    esp_err_t readStatus = adc_oneshot_read(lightSensorAdcHandle, lightSensorConfiguration.adcChannel, &rawValue);
    if (readStatus == ESP_OK) {
        *lightPercent = convertRawValueToLightPercent(rawValue);
    }
    return readStatus;
}

static esp_err_t generateTemperatureCelsius(int32_t *temperatureCelsius)
{
    uint32_t temperatureValueCount = temperatureSensorConfiguration.celsiusMax - temperatureSensorConfiguration.celsiusMin + 1;
    *temperatureCelsius = (int32_t)(esp_random() % temperatureValueCount) + temperatureSensorConfiguration.celsiusMin;
    return ESP_OK;
}

static void sendMeasurement(const Measurement *measurement)
{
    if (xQueueSend(measurementQueue, measurement, pdMS_TO_TICKS(measurementQueueConfiguration.sendTimeoutMs)) != pdPASS) {
        ESP_LOGW(sensorDescriptions[measurement->sensorKind].label, "measurement queue is full, measurement dropped");
    }
}

static void printMeasurement(const Measurement *measurement)
{
    const SensorDescription *sensorDescription = &sensorDescriptions[measurement->sensorKind];
    ESP_LOGI(sensorDescription->label, "%" PRId32 " %s", measurement->value, sensorDescription->unit);
}

static void sensorTask(void *taskParameter)
{
    SensorKind sensorKind = (SensorKind)(intptr_t)taskParameter;
    const SensorDescription *sensorDescription = &sensorDescriptions[sensorKind];
    Measurement measurement = {.sensorKind = sensorKind};
    TickType_t lastWakeTick = xTaskGetTickCount();

    while (true) {
        esp_err_t readStatus = sensorDescription->readSensorValue(&measurement.value);
        if (readStatus == ESP_OK) {
            sendMeasurement(&measurement);
        } else {
            ESP_LOGW(sensorDescription->label, "sensor read failed: %s", esp_err_to_name(readStatus));
        }
        xTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(sensorDescription->pollPeriodMs));
    }
}

static void printTask(void *taskParameter)
{
    Measurement measurement;

    while (true) {
        if (xQueueReceive(measurementQueue, &measurement, portMAX_DELAY) == pdPASS) {
            printMeasurement(&measurement);
        }
    }
}

static void initializeLightSensorAdc(void)
{
    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = lightSensorConfiguration.adcUnit,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &lightSensorAdcHandle));

    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = lightSensorConfiguration.adcAttenuation,
        .bitwidth = lightSensorConfiguration.adcBitwidth,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(lightSensorAdcHandle, lightSensorConfiguration.adcChannel, &channelConfig));
}

static void createMeasurementQueue(void)
{
    measurementQueue = xQueueCreate(measurementQueueConfiguration.length, sizeof(Measurement));
    if (measurementQueue == NULL) {
        ESP_LOGE(LOG_TAG, "failed to create measurement queue");
        abort();
    }
}

static void createTask(TaskFunction_t taskFunction, const char *taskName, void *taskParameter, UBaseType_t taskPriority)
{
    if (xTaskCreate(taskFunction, taskName, taskConfiguration.stackSizeBytes, taskParameter, taskPriority, NULL) != pdPASS) {
        ESP_LOGE(LOG_TAG, "failed to create task %s", taskName);
        abort();
    }
}

static void createSensorTasks(void)
{
    for (SensorKind sensorKind = 0; sensorKind < SENSOR_KIND_COUNT; sensorKind++) {
        createTask(sensorTask, sensorDescriptions[sensorKind].label, (void *)(intptr_t)sensorKind, taskConfiguration.sensorTaskPriority);
    }
}

void app_main(void)
{
    initializeLightSensorAdc();
    createMeasurementQueue();
    createTask(printTask, "print", NULL, taskConfiguration.printTaskPriority);
    createSensorTasks();
}
