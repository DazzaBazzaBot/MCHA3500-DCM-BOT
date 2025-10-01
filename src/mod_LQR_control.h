#ifndef MOD_LQR_CONTROL_H
#define MOD_LQR_CONTROL_H

void ctrl_init(void);

void ctrl_set_x1(float);
void ctrl_set_x2(float);
void ctrl_set_x3(float);
void ctrl_set_x4(float);

float getControl(void);

void ctrl_update(void);

#endif