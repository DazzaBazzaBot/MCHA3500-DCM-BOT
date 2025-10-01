#ifndef MOD_ADC_H
#define MOD_ADC_H

void mod_adc_init(void);
void mod_adc_task(void *argument);
void mod_adc_configure_hardware(void);
void mod_adc_set_calibration(float offset, float scale);

void mod_adc_update_readings(float *voltage_1, float *voltage_2);
void mod_adc_update_currents(float *current_1, float *current_2);
void mod_adc_update_kalman_left(float new_current_left);
void mod_adc_update_kalman_right(float new_current_right);

float mod_adc_get_current_left(void);
float mod_adc_get_current_right(void);


void mod_adc_stop_task(void);
void mod_adc_start_task(void);
void mod_adc_get_currents(float *current_1, float *current_2);

#endif