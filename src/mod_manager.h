#ifndef MOD_MANAGER_H
#define MOD_MANAGER_H

enum
{
    MOD_MANAGER_PID,
    MOD_MANAGER_LQR,
    MOD_MANAGER_MPC,
    MOD_MANAGER_ASS
} ModManagerControlAlgorithm;

void mod_manager_init(void);
void mod_manager_start(int8_t);
void mod_manager_stop(void);
void mod_manager_task(void *argument);

// state getter
float mod_manager_get_phi(void);
float mod_manager_get_theta(void);
float mod_manager_get_dphi(void);
float mod_manager_get_dtheta(void);

#endif