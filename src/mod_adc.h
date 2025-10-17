#ifndef MOD_ADC_H
#define MOD_ADC_H

void mod_adc_update_readings(float*, float*);
void mod_adc_configure_hardware(void);

float mod_adc_get_voltageA(void);
float mod_adc_get_voltageB(void);


#endif