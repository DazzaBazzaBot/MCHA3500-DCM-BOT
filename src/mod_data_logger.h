#ifndef MOD_DATA_LOGGER_H
#define MOD_DATA_LOGGER_H

enum {
    LOG_IMU = 0,
    LOG_INERTIA = 1,
    LOG_MOTOR = 2,
    LOG_FREEWHEEL = 3,
    LOG_KALMAN = 4,
};

void mod_log_init(void);
void mod_log_task(void *argument);
void mod_log_start(uint8_t);
void mod_log_stop(void);

#endif