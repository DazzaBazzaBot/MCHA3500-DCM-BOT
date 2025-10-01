#ifndef MOD_ENC_H
#define MOD_ENC_H

void mod_enc_init(void);
void mod_enc_task(void *argument);
void mod_enc_configure_hardware(void);

void mod_enc_update_readings(void);
void mod_enc_reset_counts(void);
void mod_get_speed_left(void);
void mod_get_speed_right(void);

void mod_enc_stop_task(void);
void mod_enc_start_task(void);

#endif