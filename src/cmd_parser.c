// these guys were here so here they stay
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#include <inttypes.h> // For PRIxx and SCNxx macros
#include "stm32f4xx_hal.h"
#include "cmd_line_buffer.h"
#include "cmd_parser.h"
#include "heartbeat_task.h"
#include "cmsis_os2.h"

// my cool shit
#include "mod_mpu6050.h"
#include "mod_data_logger.h"
#include "mod_manager.h"
#include "mod_dcm_driver.h"
#include "mod_adc.h"
#include "mod_enc.h"
#include "mod_LQR_control.h"

// Type for each command table entry
typedef struct
{
    void (*func)(int argc, char *argv[]); // Command function pointer
    const char *cmd;                      // Command name
    const char *args;                     // Command arguments syntax
    const char *help;                     // Command description
} CMD_T;

// Command function declarations
static void _cmd_help(int, char *[]);
static void _cmd_heartbeat(int, char *[]);
static void _cmd_reset(int, char *[]);

// mod manager commands
static void _cmd_manager_start(int, char *[]);
static void _cmd_manager_stop(int, char *[]);

// data logging commands
static void _cmd_log_motor_data(int, char *[]);
static void _cmd_log_freewheel_data(int, char *[]);
static void _cmd_log_inertia_data(int, char *[]);
static void _cmd_log_imu_data(int, char *[]);
static void _cmd_log_kalman_data(int, char *[]);

// dcm commands
static void _cmd_dcm_set_pwm(int, char *[]);
static void _cmd_dcm_set_voltage(int, char *[]);

// dcm + enc + adc commands
static void _cmd_dcm_left_voltage(int, char *[]);
static void _cmd_dcm_right_voltage(int, char *[]);

// LQR commands
static void _cmd_lqr_update(int, char *[]);
static void _cmd_lqr_set_states(int, char *[]);
static void _cmd_lqr_get_control(int, char *[]);
static void _cmd_lqr_set_y_ref(int, char *[]);

// MPU BULLSHIT
static void _cmd_mpu_accel_offset(int, char *[]);

// Assesment
// set motor voltage -12 to 12
static void _cmd_ass_set_voltage(int, char *[]);

// get wheel position in rads
static void _cmd_ass_get_phi(int, char *[]);

// get chasis angle in rads from kalman
static void _cmd_ass_get_theta(int, char *[]);

// get wheel velocity
static void _cmd_ass_get_dphi(int, char *[]);

// non zero velocity ref
static void _cmd_ass_set_ds_yref(int, char *[]);

// zero velocity ref
////////////////////////////////////// PIL
// probs ass_set_state
//       ass_update_control
//       ass_get_control

// Command table
static CMD_T cmd_table[] =
    {
        {_cmd_help, "help", "", "Displays this help message"},
        {_cmd_heartbeat, "heartbeat", "[start|stop]", "Start/Stop heartbeat task"},
        {_cmd_reset, "reset", "", "Restarts the system.\n"},

        // manager
        {_cmd_manager_start, "manager_start", "[pid|lqr|mpc|ass]", "Starts the module manager with the specified control method, or assessment mode"},
        {_cmd_manager_stop, "manager_stop", "", "Stops the module manager\n"},

        // data logging
        {_cmd_log_motor_data, "log_motor_data", "", "Logs motor data"},
        {_cmd_log_freewheel_data, "log_freewheel_data", "", "Logs freewheel motor data"},
        {_cmd_log_inertia_data, "log_inertia_data", "", "Logs inertia data"},
        {_cmd_log_imu_data, "log_imu_data", "", "Logs IMU data"},
        {_cmd_log_kalman_data, "log_kalman_data", "", "Logs Kalman output\n"},

        // dcm
        {_cmd_dcm_set_pwm, "dcm_set_pwm", "<left_pwm> <right_pwm>", "Sets the PWM duty cycle for the left and right DC motors (-100 to 100)"},
        {_cmd_dcm_set_voltage, "dcm_set_voltage", "<left_voltage> <right_voltage>", "Sets the voltage for the left and right DC motors (-12.0 to 12.0)\n"},

        // dcm + enc + adc
        {_cmd_dcm_left_voltage, "dcm_left_voltage", "<left_voltage>", "Sets the voltage for the left DC motor (-12.0 to 12.0)"},
        {_cmd_dcm_right_voltage, "dcm_right_voltage", "<right_voltage>", "Sets the voltage for the right DC motor (-12.0 to 12.0)\n"},

        // LQR
        {_cmd_lqr_update, "lqr_run", "<x1> <x2> <x3> <x4>", "Runs the LQR control and outputs the control signal"},
        {_cmd_lqr_set_states, "lqr_set_states", "<x1> <x2> <x3> <x4>", "Sets the LQR state vector"},
        {_cmd_lqr_get_control, "lqr_get_control", "", "Gets the current LQR control output"},
        {_cmd_lqr_set_y_ref, "lqr_set_y_ref", "<y_ref>", "Sets the LQR reference output value\n"},

        // stupid mpu stupid stuff
        {_cmd_mpu_accel_offset, "mpu_accel_offset", "", "Calculates the accel offset when level\n"},

        // assesment stuff
        {_cmd_ass_set_voltage, "ass_set_voltage", "<v_left> <v_right>", "Sets the voltage for the motors in the range of -12V to 12V"},
        {_cmd_ass_get_phi, "ass_get_phi", "", "Gets the position of the wheel in radians"},
        {_cmd_ass_get_theta, "ass_get_theta", "", "Gets the angle of the chassis in radians"},
        {_cmd_ass_get_dphi, "ass_get_dphi", "", "Gets the velocity of the wheels in radians per second"},
        {_cmd_ass_set_ds_yref, "ass_set_ds_yref", "<rads>", "Sets the wheel velocity reference in the range of -5 to 5 radians per second\n"},
};

enum
{
    CMD_TABLE_SIZE = sizeof(cmd_table) / sizeof(CMD_T)
};

enum
{
    CMD_MAX_TOKENS = 5
}; // Maximum number of tokens to process (command + arguments)

void _cmd_reset(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    // Reset the system
    HAL_NVIC_SystemReset();
}

void _cmd_heartbeat(int argc, char *argv[])
{
    if (argc <= 1)
    {
        if (heartbeat_task_is_running())
            printf("Heartbeat is currently running\n");
        else
            printf("Heartbeat is not currently running\n");
    }
    else
    {
        if (strcmp(argv[1], "start") == 0)
        {
            heartbeat_task_start();
            printf("Heartbeat has started\n");
        }
        else if (strcmp(argv[1], "stop") == 0)
        {
            heartbeat_task_stop();
            printf("Heartbeat has stopped\n");
        }
        else
        {
            printf("%s: invalid argument \"%s\", syntax is: %s [start|stop]\n", argv[0], argv[1], argv[0]);
        }
    }
}

/**********************************************
*********** MODULE MANAGER COMMANDS ***********
**********************************************/
void _cmd_manager_start(int argc, char *argv[])
{
    if (argc <= 1)
    {
        printf("Please specify a control method: pid, lqr, or mpc\n");
    }
    else
    {
        if (strcmp(argv[1], "pid") == 0)
        {
            mod_manager_start(MOD_MANAGER_PID);
            printf("Module manager started with PID control\n");
        }
        else if (strcmp(argv[1], "lqr") == 0)
        {
            mod_manager_start(MOD_MANAGER_LQR);
            printf("Module manager started with LQR control\n");
        }
        else if (strcmp(argv[1], "mpc") == 0)
        {
            mod_manager_start(MOD_MANAGER_MPC);
            printf("Module manager started with MPC control\n");
        }
        else if (strcmp(argv[1], "ass") == 0)
        {
            mod_manager_start(MOD_MANAGER_ASS);
            printf("Module manager started assessment mode\n");
        }
        else
        {
            printf("%s: invalid argument \"%s\", syntax is: %s [pid|lqr|mpc|ass]\n", argv[0], argv[1], argv[0]);
        }
    }
}

void _cmd_manager_stop(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_manager_stop();
    printf("Module manager stopped\n");
}

/********************************************
*********** DATA LOGGING COMMANDS ***********
********************************************/
void _cmd_log_motor_data(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_enc_reset_countA();
    mod_enc_reset_countB();
    mod_log_start(LOG_MOTOR);
}

void _cmd_log_freewheel_data(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_enc_reset_countA();
    mod_enc_reset_countB();
    mod_log_start(LOG_FREEWHEEL);
}

void _cmd_log_inertia_data(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_enc_reset_countA();
    mod_enc_reset_countB();
    mod_log_start(LOG_INERTIA);
}

void _cmd_log_imu_data(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_enc_reset_countA();
    mod_enc_reset_countB();
    mod_log_start(LOG_IMU);
}

void _cmd_log_kalman_data(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    mod_enc_reset_countA();
    mod_enc_reset_countB();
    mod_log_start(LOG_KALMAN);
}

/******************************************
*********** DCM DRIVER COMMANDS ***********
******************************************/
void _cmd_dcm_set_pwm(int argc, char *argv[])
{
    if (argc < 3)
    {
        printf("Usage: %s <left_pwm> <right_pwm>\n", argv[0]);
        printf("  left_pwm and right_pwm should be in the range -100 to 100\n");
        return;
    }

    int left_pwm = atoi(argv[1]);
    int right_pwm = atoi(argv[2]);

    if (left_pwm < -100 || left_pwm > 100 || right_pwm < -100 || right_pwm > 100)
    {
        printf("Error: PWM values must be between -100 and 100\n");
        return;
    }

    mod_dcm_set_PWM((float)left_pwm, (float)right_pwm);
    printf("Set left PWM to %d%% and right PWM to %d%%\n", left_pwm, right_pwm);
}

void _cmd_dcm_set_voltage(int argc, char *argv[])
{
    if (argc < 3)
    {
        printf("Usage: %s <left_voltage> <right_voltage>\n", argv[0]);
        printf("  left_voltage and right_voltage should be in the range -12.0 to 12.0\n");
        return;
    }

    float left_voltage = atof(argv[1]);
    float right_voltage = atof(argv[2]);

    if (left_voltage < -12.0f || left_voltage > 12.0f || right_voltage < -12.0f || right_voltage > 12.0f)
    {
        printf("Error: Voltage values must be between -12.0 and 12.0\n");
        return;
    }

    mod_dcm_set_voltage(left_voltage, right_voltage);
    printf("Set left voltage to %.2fV and right voltage to %.2fV\n", left_voltage, right_voltage);
}

/******************************************
*********** DCM + MORE COMMANDS ***********
******************************************/
void _cmd_dcm_left_voltage(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s <left_voltage>\n", argv[0]);
        printf("  left_voltage should be in the range -12.0 to 12.0\n");
        return;
    }

    float left_voltage = atof(argv[1]);

    if (left_voltage < -12.0f || left_voltage > 12.0f)
    {
        printf("Error: Voltage value must be between -12.0 and 12.0\n");
        return;
    }

    mod_dcm_set_voltageLeft(left_voltage);
    int32_t left_enc_count = mod_enc_get_countB();
    float left_adc_voltage = mod_adc_get_voltageB();

    // prent voltage, enc count, adc voltage
    printf("Set left voltage to %.2fV | Left Encoder Count: %" PRId32 " | Left ADC Voltage: %.2fV\n", left_voltage, left_enc_count, left_adc_voltage);
}

void _cmd_dcm_right_voltage(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s <right_voltage>\n", argv[0]);
        printf("  right_voltage should be in the range -12.0 to 12.0\n");
        return;
    }

    float right_voltage = atof(argv[1]);

    if (right_voltage < -12.0f || right_voltage > 12.0f)
    {
        printf("Error: Voltage value must be between -12.0 and 12.0\n");
        return;
    }

    mod_dcm_set_voltageRight(right_voltage);
    int32_t right_enc_count = mod_enc_get_countA();
    float right_adc_voltage = mod_adc_get_voltageA();

    // prent voltage, enc count, adc voltage
    printf("Set right voltage to %.2fV | Right Encoder Count: %" PRId32 " | Right ADC Voltage: %.2fV\n", right_voltage, right_enc_count, right_adc_voltage);
}

/*******************************************
*********** LQR CONTROK COMMANDS ***********
*******************************************/
void _cmd_lqr_update(int argc, char *argv[])
{
    if (argc < 5)
    {
        printf("Usage: %s <x1> <x2> <x3> <x4>\n", argv[0]);
        printf("  x1, x2, x3, x4 are the LQR state variables\n");
        return;
    }

    float x1 = atof(argv[1]);
    float x2 = atof(argv[2]);
    float x3 = atof(argv[3]);
    float x4 = atof(argv[4]);

    mod_LQR_set_x1(x1);
    mod_LQR_set_x2(x2);
    mod_LQR_set_x3(x3);
    mod_LQR_set_x4(x4);

    mod_LQR_update();

    printf("%.4f\n", mod_LQR_get_control());
}

void _cmd_lqr_set_states(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
}

void _cmd_lqr_get_control(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);
}

void _cmd_lqr_set_y_ref(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s <y_ref>\n", argv[0]);
        printf("  y_ref is the LQR reference output value\n");
        return;
    }

    float y_ref = atof(argv[1]);

    mod_LQR_set_y_ref(y_ref);

    printf("LQR reference output set to %.4f\n", y_ref);
}

/***********************************
*********** IMU COMMANDS ***********
***********************************/
void _cmd_mpu_accel_offset(int argc, char *argv[])
{
    printf("start calc (thats short for calculator chat)\n");
    float sum = 0.0f;
    int16_t ax, ay, az;

    for (int i = 0; i < 200; i++)
    {
        mod_mpu_read_raw_accel(&ax, &ay, &az);
        float pitch = mod_mpu_update_pitch(ax, ay, az);
        sum += pitch;
        osDelay(5); // Small delay between samples
    }

    float avg_off = sum / 200;

    printf("Accel offset: %.4f\n", avg_off);
}

/******************************************
*********** ASSESSMENT COMMANDS ***********
******************************************/

static void _cmd_ass_set_voltage(int argc, char *argv[])
{
    UNUSED(argv);

    // Check arg coung
    if (argc < 3)
    {
        printf("Usage: %s <left_voltage> <right_voltage>\n", argv[0]);
        printf("Both should be in the range -12.0 to 12.0\n");
        return;
    }

    // Make the number number
    float left_voltage = atof(argv[1]);
    float right_voltage = atof(argv[2]);

    // CHecl range
    if (fabs(left_voltage) > 12.0f || fabs(right_voltage) > 12.0f)
    {
        printf("Error: Voltage must be between -12.0 and 12.0\n");
        return;
    }

    // Set voltage
    mod_dcm_set_voltageLeft(left_voltage);
    mod_dcm_set_voltageRight(right_voltage);
}

static void _cmd_ass_get_phi(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    // get from mod man
    float state_phi = mod_manager_get_phi();
    printf("Wheel position: %0.3fradians\n", state_phi);
}

static void _cmd_ass_get_theta(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    // get from mod man
    float state_theta = mod_manager_get_theta();
    printf("Chassis angle: %0.3fradians\n", state_theta);
}

static void _cmd_ass_get_dphi(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    // get from mod man
    float state_dphi = mod_manager_get_dphi();
    printf("Wheel position: %0.3fradians/second\n", state_dphi);
}

static void _cmd_ass_set_ds_yref(int argc, char *argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    // set lqr
    if (argc < 2)
    {
        printf("Usage: %s <y_ref>\n", argv[0]);
        printf("  y_ref is the LQR reference output value\n");
        return;
    }

    float y_ref = atof(argv[1]);

    mod_LQR_set_y_ref(y_ref);
}

/*******************************************/
static void _print_chip_pinout(void);

void _cmd_help(int argc, char *argv[])
{
    UNUSED(argv); // Avoid compiler warning about unused variable
    printf(
        "\n"
        "\n");

    _print_chip_pinout();

    printf("\n");

    switch (argc)
    {
    case 1:
        printf(
            "Command Arguments                         Description             \n"
            "------------------------------------------------------------------\n");
        for (int i = 0; i < CMD_TABLE_SIZE; i++)
        {
            printf("%-25s %-15s %-s\n", cmd_table[i].cmd, cmd_table[i].args, cmd_table[i].help);
        }
        // printf("\nFor more information, enter help followed by the command name\n\n");
        break;
        // case 2:
        //     printf("To be implemented...\n\n");
        // TODO: Scan command table, and lookup extended help string.
        break;
        // default:
        // printf("help is expecting zero or one argument.\n\n");
    }
}

void _print_chip_pinout(void)
{
    printf(
        "Pin configuration:                                     \n"
        "                                                       \n"
        "       .---------------------------------------.       \n"
        " PC10--|  1  2 --PC11              PC9--  1  2 |--PC8  \n"
        " PC12--|  3  4 --PD2               PB8--  3  4 |--PC6  \n"
        "  VDD--|  5  6 --E5V               PB9--  5  6 |--PC5  \n"
        "BOOT0--|  7  8 --GND              AVDD--  7  8 |--U5V  \n"
        "   NC--|  9 10 --NC                GND--  9 10 |--NC   \n"
        "   NC--| 11 12 --IOREF             PA5-- 11 12 |--PA12 \n"
        " PA13--| 13 14 --RESET             PA6-- 13 14 |--PA11 \n"
        " PA14--| 15 16 --+3v3              PA7-- 15 16 |--PB12 \n"
        " PA15--| 17 18 --+5v               PB6-- 17 18 |--NC   \n"
        "  GND--| 19 20 --GND               PC7-- 19 20 |--GND  \n"
        "  PB7--| 21 22 --GND               PA9-- 21 22 |--PB2  \n"
        " PC13--| 23 24 --VIN               PA8-- 23 24 |--PB1  \n"
        " PC14--| 25 26 --NC               PB10-- 25 26 |--PB15 \n"
        " PC15--| 27 28 --PA0               PB4-- 27 28 |--PB14 \n"
        "  PH0--| 29 30 --PA1               PB5-- 29 30 |--PB13 \n"
        "  PH1--| 31 32 --PA4               PB3-- 31 32 |--AGND \n"
        " VBAT--| 33 34 --PB0              PA10-- 33 34 |--PC4  \n"
        "  PC2--| 35 36 --PC1               PA2-- 35 36 |--NC   \n"
        "  PC3--| 37 38 --PC0               PA3-- 37 38 |--NC   \n"
        "       |________                   ____________|       \n"
        "                \\_________________/                   \n");
}

// Command parser
static int _makeargv(char *s, char *argv[], int argvsize);

#ifdef NO_LD_WRAP
void cmd_parse(char *) __asm__("___real_cmd_parse");
#endif

void cmd_parse(char *cmd)
{
    if (cmd == NULL)
    {
        printf("ERROR: Tried to parse NULL command pointer\n");
        return;
    }
    else if (*cmd == '\0') // Empty command string
    {
        return;
    }

    // Tokenise command string
    char *argv[CMD_MAX_TOKENS];
    int argc = _makeargv(cmd, argv, CMD_MAX_TOKENS);

    // Execute corresponding command function
    for (int i = 0; i < CMD_TABLE_SIZE; i++)
    {
        if (strcmp(argv[0], cmd_table[i].cmd) == 0)
        {
            cmd_table[i].func(argc, argv);
            return;
        }
    }

    // Command not found
    printf("Unknown command: \"%s\"\n", argv[0]);
    // if (argc > 1)
    // {
    //     printf("    with arguments:\n");
    //     for (int i = 1; i < argc; i++)
    //     {
    //         printf("        %d: %s\n", i, argv[i]);
    //     }
    // }
    // printf_P(PSTR("\nEnter \"help\" for a list of commands.\n\n"));
}

int _makeargv(char *s, char *argv[], int argvsize)
{
    char *p = s;
    int argc = 0;

    for (int i = 0; i < argvsize; ++i)
    {
        // skip leading whitespace
        while (isspace(*p))
            p++;

        if (*p != '\0')
            argv[argc++] = p;
        else
        {
            argv[argc] = NULL;
            break;
        }

        // scan over arg
        while (*p != '\0' && !isspace(*p))
            p++;

        // terminate arg
        if (*p != '\0' && i < argvsize - 1)
            *p++ = '\0';
    }

    return argc;
}