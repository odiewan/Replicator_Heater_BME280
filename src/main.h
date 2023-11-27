#ifndef __main_h__
#define __main_h__

#include <Arduino.h>
#include <String.h>
#include <serialPrint.h>
#include <Adafruit_ADT7410.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <u8TextBox.h>
#include <u8BarGraph.h>
#include <u8g2Disp.h>
#include <ioChan.h>
#include <ioBtn.h>

#define STAT_OUT_PIN LED_BUILTIN /* 13 */

#define KNOB_IN_PIN A0     /* 14 */
#define NTC_IN_PIN A1      /* 15 */
#define AUX_TEMP_IN_PIN A2 /* 16 */
#define I_IN_PIN A3        /* 17 */
#define V_IN_PIN A4        /* 18 */

#define PUMP_OUT_PIN        2
#define G_LED_OUT_PIN       4
#define Y_LED_OUT_PIN       5
#define R_LED_OUT_PIN       6
#define HEAT00_OUT_PIN      7
#define HEAT01_OUT_PIN      8
#define HEAT02_OUT_PIN      9
#define BTN00_IN_PIN        12
#define BTN01_IN_PIN        11

#define MAX_TEMP_MAN        100
#define MAX_TEMP_MAX        120
#define MAX_PLAUSIBLE_TEMP  120
#define MIN_PLAUSIBLE_TEMP  10

#define MAX_PCOMP           200
#define MAX_ICOMP           200

#define KP 8
#define KI 2
#define DEF_SP 15

#define TEMP_HYST 2

#define SAMPLE_CNT 10

//---Non HW defines---
#define SER_DELAY 3
#define SER_FREQ 3
#define LOOP_DLY 5
#define BTN_THRES 50
#define KNOB_MAX 200
#define KNOB_MIN 0
#define FRAME_WIDTH 50 / LOOP_DLY
#define NUM_BLINK_FRAMES 5
#define BLINK_PERCENT 10

#define U8D_ROW_00 16
#define U8D_ROW_01 32

#define BAR_GRAPH_BOOT_X 88
#define BAR_GRAPH_BOOT_Y 20
#define BAR_GRAPH_BOOT_H 12
#define BAR_GRAPH_BOOT_W 36

#define BAR_GRAPH_CV_X 88
#define BAR_GRAPH_CV_Y 20
#define BAR_GRAPH_CV_H 12
#define BAR_GRAPH_CV_W 36
#define SEALEVELPRESSURE_HPA (1013.25)

enum TaskType
{
    TASK_STAT_LED,
    TASK_POWER_UP,
    TASK_SAWTOOTH,
    TASK_VOLTAGE_MON,
    TASK_BTN_READ,
    TASK_OP_MODE,
    TASK_HEAT_CTRL,
    TASK_PMP_CTRL,
    TASK_SERIAL,
    TASK_IMU,
    TASK_OLED,
    TASK_NTC,
    TASK_AUX_IN,
    TASK_I2C_TEMP_IN,

    NUM_TASKS
};

String taskStr[NUM_TASKS] = {
    "TASK_STAT_LED",
    "TASK_POWER_UP",
    "TASK_SAWTOOTH",
    "TASK_VOLTAGE_MON",
    "TASK_BTN_READ",
    "TASK_OP_MODE",
    "TASK_PMP_CTRL",
    "TASK_HEAT_CTRL",
    "TASK_SERIAL",
    "TASK_IMU",
    "TASK_OLED",
    "TASK_NTC",
    "TASK_AUX_IN",
    "TASK_I2C_TEMP_IN",

};

enum opModes
{
    OP_MD_NA,
    OP_MD_BOOT,
    OP_MD_PWRUP,
    OP_MD_RESET,
    OP_MD_IDLE,
    OP_MD_SP_SET,
    OP_MD_TEMP_HOLD,
    OP_MD_IND_MAN,
    OP_MD_RESET_MIN_MAX,
    OP_MD_OLED_SETTINGS,
    OP_MD_SERIAL_SETTINGS,
    NUM_OP_MODES
};

String opMdStr[NUM_OP_MODES] = {
    "<     NA      >",
    "<    Boot     >",
    "<   Powerup   >",
    "<    Reset    >",
    "<    Idle     >",
    "<   SP Adj    >",
    "<  Temp Hold  >",
    "<   Ind Man   >",
    "<Reset Min/max>",
    "<   OLED Opt  >",
    "<   SER Opt   >",
};

String opMdOledStr[NUM_OP_MODES] = {
    "NA",
    "Boot",
    "Powerup",
    "Reset",
    "Idle",
    "SP",
    "Hold",
    "Ind",
    "Rst Min/max",
    "OLED Opt",
    "Ser Opt"};

enum displayModeStr
{
    DM_DEFALT,
    DM_TEMP_CL,
    DM_ATMO,
    DM_HEAT_MAN,
    DM_SETTINGS,
    DM_SER_SETTINGS,
    DM_OLED_SETTINGS,
    NUM_DM_MODES
};

String displayModeStr[NUM_DM_MODES] = {
    "DM DEFALT",
    "DM TEMP_CL",
    "DM ATMO",
    "DM HEAT_MAN",
    "DM SER SETTINGS"
    "DM OLED SETTINGS"};

String displayModeStrhortStr[NUM_DM_MODES] = {
    "DM_DEF",
    "DM_CL",
    "DM_ATMO",
    "DM_MAN",
    "DM_SER",
    "DM_OLED"};

enum powerUpSteps
{
    PWRUP_NA,
    PWRUP_0,    //---All LEDs off
    PWRUP_1,    //---LF on
    PWRUP_2,    //---LF, RF on
    PWRUP_3,    //---LF, RF, RR on
    PWRUP_4,    //---LF, RF, RR, LR on
    PWRUP_5,    //---LF, RF, RR on
    PWRUP_6,    //---LF, RF on
    PWRUP_7,    //---LF, on
    PWRUP_DONE, //---All LEDs off
    NUM_PWRUP_STEPS
};

String pwrUpStr[NUM_PWRUP_STEPS] = {
    "PWRUP_NA",
    "PWRUP_0",
    "PWRUP_1",
    "PWRUP_2",
    "PWRUP_3",
    "PWRUP_4",
    "PWRUP_5",
    "PWRUP_6",
    "PWRUP_7",
    "PWRUP_DONE"};

enum errorStates
{
    ERR_ST_NA,
    ERR_ST_NO_ERR,
    ERR_ST_I2C_ADT_FAIL,
    ERR_ST_I2C_BME_FAIL,
    ERR_ST_I2C_OLED_FAIL,
    ERR_ST_OVERTEMP,
    NUM_ERR_STATES
};

String errStStr[NUM_ERR_STATES] = {
    "ERR_ST_NA",
    "ERR_ST_NO_ERR",
    "ERR_ST_I2C_ADT_FAIL",
    "ERR_ST_I2C_BME_FAIL",
    "ERR_ST_I2C_OLED_FAIL",
    "ERR_ST_OVERTEMP"};

#endif