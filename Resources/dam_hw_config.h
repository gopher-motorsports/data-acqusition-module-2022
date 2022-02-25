// Autogenerated by SensorCannon
//
// TODO fix these defines in the auto gen to be more general
#ifndef DAM_HW_CONFIG_H
#define DAM_HW_CONFIG_H

#include "sensor_hal.h"


#define NUM_ADC1_PARAMS 2
#define NUM_ADC2_PARAMS 0
#define NUM_ADC3_PARAMS 0

// analog params must be in channel order
extern ANALOG_SENSOR_PARAM adc1_sensor_params[NUM_ADC1_PARAMS];
extern ANALOG_SENSOR_PARAM adc2_sensor_params[NUM_ADC2_PARAMS];
extern ANALOG_SENSOR_PARAM adc3_sensor_params[NUM_ADC3_PARAMS];

#define NUM_CAN_SENSOR_PARAMS 4
extern CAN_SENSOR_PARAM can_sensor_params[NUM_CAN_SENSOR_PARAMS];

#define NUM_BUCKETS 3
extern BUCKET buckets[NUM_BUCKETS];

void init_adc1_params(void);
void init_adc2_params(void);
void init_adc3_params(void);
void init_can_params(void);
void init_buckets(void);

#endif // DAM_HW_CONFIG_H_H
// End autogenerated file