// Autogenerated by SensorCannon
#include "dam_hw_config.h" // TODO fix this in auto gen to be general
#include "gopher_sense.h"
#include "GopherCAN.h"




// BEGIN ADC1 PARAMS

// give memory to the buffers


#define ADC1_PARAM1_BUF_SIZE 100
U32 adc1_param1_buffer_mem[ADC1_PARAM1_BUF_SIZE];

// Fill out the array with all of the sensor params
ANALOG_SENSOR_PARAM adc1_sensor_params[NUM_ADC1_PARAMS] = 
{
  
  
  // adc1_param1
  {
    .analog_param =
    {
      .can_param = &float_tester,
      .status = CLEAN,
    },
    .analog_sensor = &basic_voltage_sensor_for_testing,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = adc1_param1_buffer_mem,
      .buffer_size = ADC1_PARAM1_BUF_SIZE,
      .fill_level = 0
    }
  },
  #undef ADC1_PARAM1_BUF_SIZE
  
};

// ***************************************************************************



// BEGIN ADC2 PARAMS

// give memory to the buffers


#define ADC2_PARAM1_BUF_SIZE 100
U32 adc2_param1_buffer_mem[ADC2_PARAM1_BUF_SIZE];

// Fill out the array with all of the sensor params
ANALOG_SENSOR_PARAM adc2_sensor_params[NUM_ADC2_PARAMS] = 
{
  
  
  // adc2_param1
  {
    .analog_param =
    {
      .can_param = &float_tester_2,
      .status = CLEAN,
    },
    .analog_sensor = &basic_resistive_sensor_for_testing,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = adc2_param1_buffer_mem,
      .buffer_size = ADC2_PARAM1_BUF_SIZE,
      .fill_level = 0
    }
  },
  #undef ADC2_PARAM1_BUF_SIZE
  
};

// ***************************************************************************



// No parameters on ADC3

// ***************************************************************************



// BEGIN CAN PARAMS
// give memory to the buffers


#define CAN_PARAM1_BUF_SIZE 1
U32 can_param1_buffer_mem[CAN_PARAM1_BUF_SIZE];


#define CAN_PARAM2_BUF_SIZE 1
U32 can_param2_buffer_mem[CAN_PARAM2_BUF_SIZE];


#define CAN_PARAM3_BUF_SIZE 1
U32 can_param3_buffer_mem[CAN_PARAM3_BUF_SIZE];


#define CAN_PARAM4_BUF_SIZE 1
U32 can_param4_buffer_mem[CAN_PARAM4_BUF_SIZE];

// Fill out the array with all of the sensor params
CAN_SENSOR_PARAM can_sensor_params[NUM_CAN_SENSOR_PARAMS] =
{
  
  
  
  // can_param1
  {

    .can_param =
    {
      .can_param = &float_tester_3,
      .status = CLEAN,
    },
    .can_sensor = &can_sensor_for_testing,
    .message_idx = 0,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = can_param1_buffer_mem,
      .buffer_size = CAN_PARAM1_BUF_SIZE,
      .fill_level = 0
    }

  },
  #undef CAN_PARAM1_BUF_SIZE
  
  
  
  // can_param2
  {

    .can_param =
    {
      .can_param = &float_tester_4,
      .status = CLEAN,
    },
    .can_sensor = &can_sensor_for_testing,
    .message_idx = 1,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = can_param2_buffer_mem,
      .buffer_size = CAN_PARAM2_BUF_SIZE,
      .fill_level = 0
    }

  },
  #undef CAN_PARAM2_BUF_SIZE
  
  
  
  // can_param3
  {

    .can_param =
    {
      .can_param = &float_tester_5,
      .status = CLEAN,
    },
    .can_sensor = &can_sensor_for_testing,
    .message_idx = 2,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = can_param3_buffer_mem,
      .buffer_size = CAN_PARAM3_BUF_SIZE,
      .fill_level = 0
    }

  },
  #undef CAN_PARAM3_BUF_SIZE
  
  
  
  // can_param4
  {

    .can_param =
    {
      .can_param = &float_tester_6,
      .status = CLEAN,
    },
    .can_sensor = &can_sensor_for_testing,
    .message_idx = 3,
    .num_filtered_subparams = 0, // not yet implemented
    .filtered_subparams = NULL, // not yet implemented
    .buffer =
    {
      .buffer = can_param4_buffer_mem,
      .buffer_size = CAN_PARAM4_BUF_SIZE,
      .fill_level = 0
    }

  },
  #undef CAN_PARAM4_BUF_SIZE
  
};

// ***************************************************************************

// BUCKETS
// setup the param lists for each bucket

GENERAL_PARAMETER bucket_1_bucket_general_param_list[2] =
{
  {
    .can_param = &float_tester,
    .status = CLEAN
  },
  {
    .can_param = &float_tester_2,
    .status = CLEAN
  },
  
};

GENERAL_PARAMETER bucket_2_bucket_general_param_list[2] =
{
  {
    .can_param = &float_tester_3,
    .status = CLEAN
  },
  {
    .can_param = &float_tester_4,
    .status = CLEAN
  },
  
};

GENERAL_PARAMETER bucket_3_bucket_general_param_list[2] =
{
  {
    .can_param = &float_tester_5,
    .status = CLEAN
  },
  {
    .can_param = &float_tester_6,
    .status = CLEAN
  },
  
};


// initalize the bucket structs
BUCKET bucket_list[NUM_BUCKETS] =
{
  
  // Define bucket bucket_1
  {
    .bucket_id = 1,
    .frequency = 10,
    .state = BUCKET_INIT,
    .param_list = 
    {
      .len = 2,
      .list = bucket_1_bucket_general_param_list
    }
  },
  
  // Define bucket bucket_2
  {
    .bucket_id = 2,
    .frequency = 50,
    .state = BUCKET_INIT,
    .param_list = 
    {
      .len = 2,
      .list = bucket_2_bucket_general_param_list
    }
  },
  
  // Define bucket bucket_3
  {
    .bucket_id = 3,
    .frequency = 1,
    .state = BUCKET_INIT,
    .param_list = 
    {
      .len = 2,
      .list = bucket_3_bucket_general_param_list
    }
  },
  
};


//END autogenerted file