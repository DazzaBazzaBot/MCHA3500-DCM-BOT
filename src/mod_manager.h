#ifndef MOD_MANAGER_H
#define MOD_MANAGER_H

enum
{
    MOD_MANAGER_PID,
    MOD_MANAGER_LQR,
    MOD_MANAGER_MPC
} ModManagerControlAlgorithm;

void mod_manager_init(void);
void mod_manager_start(int8_t);
void mod_manager_stop(void);
void mod_manager_task(void *argument);

#endif