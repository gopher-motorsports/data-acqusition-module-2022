/*
 * DAM.c
 *
 *  Created on: Jun 9, 2021
 *      Author: ian
 */


#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "main.h"
#include "stm32f7xx_hal_can.h"
#include "DAM.h"
#include "sensor_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
//DMA_HandleTypeDef hdma_adc1;
//DMA_HandleTypeDef hdma_adc2;
//DMA_HandleTypeDef hdma_adc3;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim14;

#define THIS_DAM_ID DAM_FR_ID

#define GOPHERCAN_HANDLE hcan1
#define SENSORCAN_HANDLE hcan2

#define TIMER_PSC 16
// Move this to a config file
#define ADC1_SCHEDULING_FREQUENCY_HZ 100
#define ADC2_SCHEDULING_FREQUENCY_HZ 100
#define ADC3_SCHEDULING_FREQUENCY_HZ 100

#define BUCKET_SEND_TASK_STACK_SIZE 128
#define BUCKET_TASK_NAME_BASE "send_bucket_task_"
#define BUCKET_SEND_MAX_ATTEMPTS 5
#define INITIAL_DATA 0xAA
#define DATA_CONV_FAILURE_REPLACEMENT -1


typedef enum {
    WAITING = 0,
    CONFIG = 1,
    NORMAL = 2
} DAM_STATE;

//extern U8* parameter_data_types;

static U64 error_count;
static DAM_STATE dam_state = WAITING;
static DAM_ERROR_STATE latched_error_state = NO_ERRORS;
static boolean hasInitialized = FALSE;


//void change_led_state(U8 sender, void* parameter, U8 remote_param, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3)
//{
//    // this function will set the LED to high or low, depending on remote_param
//    // the LED to change is dependent on the parameter stored on this module (*((U16*)parameter))
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//}

// TODO: Error state behavior
// TODO: pitch Error scheme to HOC
void handle_DAM_error (DAM_ERROR_STATE error_state) {
    latched_error_state = error_state;
    switch (error_state) {
        // log the error?
        case INITIALIZATION_ERROR:
        {
            NVIC_SystemReset();
            break;
        }
        case CRITICAL_ERROR:
        {
            NVIC_SystemReset();
            break;
        }
        case CONVERSION_ERROR:
        {
        	//send can error???
        	break;
        }
        default:
        {
            error_count++;
            break;
        }
    }
}

void DAM_init(void) {
    if (!hasInitialized) {
        // Run once initialization code

        if (init_can(&GOPHERCAN_HANDLE, THIS_DAM_ID, BXTYPE_MASTER)) {
            handle_DAM_error(INITIALIZATION_ERROR);
        }
        set_all_params_state(TRUE);
        init_sensor_hal();

        add_custom_can_func(SEND_BUCKET_PARAMS, &send_bucket_params, TRUE, NULL);
        add_custom_can_func(BUCKET_OK, &bucket_ok, TRUE, NULL);
        add_custom_can_func(REQUEST_BUCKET, &bucket_requested, TRUE, NULL);

        configLibADC(&hadc1, &hadc2, &hadc3);
        configLibTIM(&htim10, ADC1_SCHEDULING_FREQUENCY_HZ,
                     &htim11, ADC2_SCHEDULING_FREQUENCY_HZ,
                     &htim14, ADC3_SCHEDULING_FREQUENCY_HZ, TIMER_PSC);
    }

    // All code needed for DLM-DAM reset goes here
    stopTimers();
    // Reset all of the buffers
    for (U8 i = 0; i < NUM_ADC1_PARAMS; i++) {
        reset_buffer(&adc1_sensor_params[i].buffer);
    }
    for (U8 i = 0; i < NUM_ADC2_PARAMS; i++) {
        reset_buffer(&adc2_sensor_params[i].buffer);
    }
    for (U8 i = 0; i < NUM_ADC3_PARAMS; i++) {
        reset_buffer(&adc3_sensor_params[i].buffer);
    }
    for (U8 i = 0; i < NUM_CAN_SENSOR_PARAMS; i++) {
        reset_buffer(&can_sensor_params[i].buffer);
    }


    // enable all bucket params, set status to clean
    for (U8 i = 0; i < NUM_BUCKETS; i++) {
        BUCKET* bucket = &buckets[i];
        bucket->state = BUCKET_INIT;


        for (U16 n = 0; n < bucket->bucket.len; n++) {
            GENERAL_PARAMETER* param = &bucket->bucket.list[n];
            param->status = CLEAN;
            param->param.float_struct.update_enabled = TRUE;
            param->param.float_struct.data = INITIAL_DATA; // Set some initial value

        }
        if (!hasInitialized) {
			//create bucket tasks
			char name_buf[30];
			sprintf(name_buf, "%s%d", BUCKET_TASK_NAME_BASE, bucket->bucket_id);
			if (xTaskCreate(send_bucket_task, name_buf, BUCKET_SEND_TASK_STACK_SIZE,
							(void*) bucket, osPriorityNormal, NULL) != pdPASS) {
				// set error state
				handle_DAM_error(INITIALIZATION_ERROR);
			}
		}
    }


    hasInitialized = TRUE;

}


void complete_DLM_handshake (void) {

    boolean all_buckets_ok = FALSE;
    while (!all_buckets_ok) {
        boolean check_buckets_ok = TRUE;

        for (U8 i = 0; i < NUM_BUCKETS; i++) {
            BUCKET* bucket = &buckets[i];
            if (bucket->state == BUCKET_DLM_OK) {
                continue; // if bucket ok dont do anything
            }
            check_buckets_ok = FALSE;
            send_can_command(PRIO_HIGH, DLM_ID, SET_BUCKET_SIZE, bucket->bucket_id, (U8)bucket->bucket.len, 0, 0);

            for (U16 n = 0; n < bucket->bucket.len; n++) {
                GENERAL_PARAMETER param = bucket->bucket.list[n];
                send_can_command(PRIO_HIGH, DLM_ID, ADD_PARAM_TO_BUCKET, bucket->bucket_id,
                                 GET_U16_MSB(param.param.float_struct.param_id), GET_U16_LSB(param.param.float_struct.param_id), 0);
            }
            osDelay(10); // Delay to avoid flooding the TX_queue
        }
        all_buckets_ok = check_buckets_ok;
    }

    // Assign the buckets to the correct frequencies after all buckets OK
    for (U8 i = 0; i < NUM_BUCKETS; i++) {
        BUCKET* bucket = &buckets[i];
        send_can_command(PRIO_HIGH, DLM_ID, ASSIGN_BUCKET_TO_FRQ,
                         bucket->bucket_id, GET_U16_MSB(bucket->frequency), GET_U16_LSB(bucket->frequency), 0);
        bucket->state = BUCKET_GETTING_DATA;
    }

    startTimers(); // Start the data acquisition process
    dam_state = NORMAL;
}



BUCKET* get_bucket_by_id (U8 bucket_id) {
    for (U8 i = 0; i < NUM_BUCKETS; i++) {
        if (buckets[i].bucket_id == bucket_id) {
            return &buckets[i];
        }
    }
    return NULL;
}



void ADC_sensor_service (void) {
    for (U8 i = 0; i < NUM_ADC1_PARAMS; i++) {
        ANALOG_SENSOR_PARAM* param = &adc1_sensor_params[i];
        if (buffer_full(&param->buffer)) {
            U16 avg;
            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
                continue;
            }
            float data_in = avg;
            float converted;
            if (apply_analog_sensor_conversion(&param->analog_sensor, data_in, &converted)
                 != BUFFER_SUCCESS) {
                handle_DAM_error(CONVERSION_ERROR);
            }
            // No data cast as we assume params are set up correctly
            param->analog_param.param.float_struct.data = converted;
            param->analog_param.status = DIRTY;
            fill_analog_subparams(param, converted);
        }
    }

    for (U8 i = 0; i < NUM_ADC2_PARAMS; i++) {
        ANALOG_SENSOR_PARAM* param = &adc2_sensor_params[i];
        if (buffer_full(&param->buffer)) {
            U16 avg;
            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
                continue;
            }
            float data_in = avg;
            float converted;
            if (apply_analog_sensor_conversion(&param->analog_sensor, data_in, &converted)
                 != BUFFER_SUCCESS) {
            	 handle_DAM_error(CONVERSION_ERROR);
            }
            // No data cast as we assume params are set up correctly
            param->analog_param.param.float_struct.data = converted;
            param->analog_param.status = DIRTY;
            fill_analog_subparams(param, converted);
        }
    }

    for (U8 i = 0; i < NUM_ADC3_PARAMS; i++) {
        ANALOG_SENSOR_PARAM* param = &adc3_sensor_params[i];
        if (buffer_full(&param->buffer)) {
            U16 avg;
            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
                continue;
            }
            float data_in = avg;
            float converted;
            if (apply_analog_sensor_conversion(&param->analog_sensor, data_in, &converted)
                 != BUFFER_SUCCESS) {
            	handle_DAM_error(CONVERSION_ERROR);
            }
            // No data cast as we assume params are set up correctly
            param->analog_param.param.float_struct.data = converted;
            param->analog_param.status = DIRTY;
            fill_analog_subparams(param, converted);
        }
    }
}


void sensorCAN_service (void) {
    for (U8 i = 0; i < NUM_CAN_SENSOR_PARAMS; i++) {
        CAN_SENSOR_PARAM* param = &can_sensor_params[i];
        if (buffer_full(&param->buffer)) {
            U16 avg;
            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
                continue;
            }
            float data_in = avg;
            float converted;
            if (apply_can_sensor_conversion(&param->can_sensor, param->message_idx, data_in, &converted)
                 != BUFFER_SUCCESS) {
            	handle_DAM_error(CONVERSION_ERROR);
            }
            // No data cast as we assume params are set up correctly
            param->can_param.param.float_struct.data = converted; // fill the data
            param->can_param.status = DIRTY;
            fill_can_subparams(param, converted);
        }
    }
}

// THis has to do with filtering, so do this whomever implements that
void fill_can_subparams (CAN_SENSOR_PARAM* param, float newdata) {
    for (U8 i = 0; i < param->num_filtered_params; i++) {
//        U16_BUFFER temp = param->buffer;
//        apply_filter(&temp, &param->filtered_subparams[i]);
//        U16 avg;
//        if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
//            continue;
//        }
//        float data_in = avg;
//        float converted;
//        if (apply_can_sensor_conversion(&param->can_sensor, param->message_idx, data_in, &converted)
//             != BUFFER_SUCCESS) {
//            converted = DATA_CONV_FAILURE_REPLACEMENT;
//        }
        // No data cast as we assume params are set up correctly
        param->filtered_subparams[i].filtered_param.param.float_struct.data = newdata; //fill the data
        param->filtered_subparams[i].filtered_param.status = DIRTY;
    }
}


// THis has to do with filtering, so do this whomever implements that
void fill_analog_subparams (ANALOG_SENSOR_PARAM* param, float newdata) {
    for (U8 i = 0; i < param->num_filtered_subparams; i++) {
//            U16_BUFFER temp = param->buffer;
//            apply_filter(&temp, &param->filtered_subparams[i]);
//            U16 avg;
//            if (average_buffer(&param->buffer, &avg) != BUFFER_SUCCESS) {
//                continue;
//            }
//            float data_in = avg;
//            float converted;
//            if (apply_analog_sensor_conversion(&param->analog_sensor, data_in, &converted) != BUFFER_SUCCESS) {
//                converted = DATA_CONV_FAILURE_REPLACEMENT;
//            }
            // No data cast as we assume params are set up correctly
            param->filtered_subparams[i].filtered_param.param.float_struct.data = newdata; //fill the data
            param->filtered_subparams[i].filtered_param.status = DIRTY;
    }
}



/* send_bucket_task
 * Task created when BucketRequest gopherCAN command is recieved
 * sends all parameters in the provided bucket, deletes upon completion
 */
void send_bucket_task (void* pvParameters) {
    BUCKET* bucket = (BUCKET*) pvParameters;

    while(1){
    	while (bucket->state != BUCKET_REQUESTED){
    		taskYIELD();
    	}

		// send the bucket parameters
		for (U8 i = 0; i < bucket->bucket.len; i++) {
			GENERAL_PARAMETER* param = &bucket->bucket.list[i];
			if (param->status < DIRTY) {
				U16 err_count = 0;
				while (send_parameter(PRIO_HIGH, DLM_ID, param->param.float_struct.param_id) != CAN_SUCCESS) {
					if (++err_count > BUCKET_SEND_MAX_ATTEMPTS) {
						//Todo set error state behavior
						handle_DAM_error(TBD_ERROR);
						break;
					}
					osDelay(1); // Delay due to error
				}
				param->status = CLEAN;
		   }

		}

		bucket->state = BUCKET_GETTING_DATA;
    }
    vTaskDelete(NULL);
}


/* DAM_main_task
 * Main task state machine
 */
void DAM_main_task (void) {
    // Must have started buffer service task and tx task
    while (1) {
        switch (dam_state) {
            case WAITING:
            {
            	osDelay(1);
            	break;
            }
            case CONFIG:
            {
                DAM_init(); // reset the DAM process if the DLM sends this request
                complete_DLM_handshake();
                break;
            }
            case NORMAL:
            {
                ADC_sensor_service();
                sensorCAN_service();
                break;
            }
            default:
            {
                handle_DAM_error(TBD_ERROR);
            }
        }

    }
}


//*************** GopherCAN callbacks *****************
/* send_bucket_params
 * Handler for the SEND_BUCKET_PARAMS gopherCAN command
 * sets the DAM into configuration state
 */
void send_bucket_params (U8 sender, void* param, U8 U1, U8 U2, U8 U3, U8 U4) {
    if (sender != DLM_ID) return;

    dam_state = CONFIG;
}


/* bucket_ok
 * Handler for the BUCKET_OK gopherCAN command
 * sets the passed bucket state to OK
 */
void bucket_ok(MODULE_ID sender, void* parameter,
               U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3) {

    if (sender != DLM_ID) return;

    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket != NULL && bucket->state <= BUCKET_DLM_OK ) { // dont reset the state of bucket getting data/sending
        bucket->state = BUCKET_DLM_OK;
    }
    else {
        // todo set error states
        handle_DAM_error(TBD_ERROR);
    }
}


/* bucket_requested
 * Handler for the RequestBucket gopherCAN command
 * Creates rtos task to handle sending the bucket
 */
void bucket_requested (MODULE_ID sender, void* parameter,
                       U8 bucket_id, U8 UNUSED1, U8 UNUSED2, U8 UNUSED3) {

    BUCKET* bucket = get_bucket_by_id(bucket_id);
    if (bucket == NULL || bucket->state < BUCKET_GETTING_DATA) {
        // todo set error states
        handle_DAM_error(TBD_ERROR);
        return;
    }

    bucket->state = BUCKET_REQUESTED;

}


/* custom_service_can_rx_hardware
 * Definition for the ISR called on CAN message reception
*/
void custom_service_can_rx_hardware(CAN_HandleTypeDef* hcan, U32 rx_mailbox)
{
   if (hcan == &GOPHERCAN_HANDLE) {
       service_can_rx_hardware(hcan, rx_mailbox);
   }

   else if (hcan == &SENSORCAN_HANDLE)
   {
       sensor_can_message_handle(hcan, rx_mailbox);
   }

   else handle_DAM_error(TBD_ERROR); // This case shouldnt happen
}


//*************** GopherCAN tasks *****************
// Service the GopherCAN tx buffer
void gopherCAN_tx_service_task (void) {
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1) {
        service_can_tx_hardware(&GOPHERCAN_HANDLE);
        osDelay(1);
    }
}

// Service the GopherCAN rx buffer
void gopherCAN_rx_buffer_service_task (void) {
    while(!hasInitialized) osDelay(1); // wait for initialization

    while (1) {
        if (service_can_rx_buffer()) {
            handle_DAM_error(TBD_ERROR);
        }
        osDelay(1);
    }
}


// TODO: add timer interrupt to ensure IDLEtask runs??



