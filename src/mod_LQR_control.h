#ifndef MOD_LQR_CONTROL_H
#define MOD_LQR_CONTROL_H

void mod_LQR_init(void);

void mod_LQR_set_states(float[]);
void mod_LQR_set_x1(float);
void mod_LQR_set_x2(float);
void mod_LQR_set_x3(float);
void mod_LQR_set_x4(float);

void mod_LQR_update(void);

float mod_LQR_get_control(void);

#endif