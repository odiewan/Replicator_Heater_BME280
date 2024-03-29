#include "main.h"

int opMode;
int dispModeOLED;
int dispModeSer;
int pwrUpStep;
int statLedState;
int rLedState;
int yLedState;
int gLedState;

int iCount;

float tempADTVar;
float tempADTVarShadow;
float tempBMEVar;
float tempBMEVarShadow;
float altVar;
float prsVar;
float humVar;

int vInVar;
ioChannel vInChan;

int iInVar;
ioChannel iInChan;

int knobInVar;
ioChannel knobInChan;

int heatVar00;
ioChannel heatChan00;

int heatVar01;
ioChannel heatChan01;

int heatVar02;
ioChannel heatChan02;

int btn00Var;
ioBtn btn00Chan;

int btn01Var;
ioBtn btn01Chan;

int errorState;

String ModeStr;
String AppTitle;
String AppVersion;

int tempLimit;
float tempSP;
float tempErr;
float tempErrOvr;
float tempCV;
float tempPV;
float tempPVShadow;
float tempKI;
float tempKP;
float tempPComp;
float tempIComp;
float dT;

float deltaT;   //Time rate of change of the Heater plate in *C/min
float deltaErr; //Time rate of chane of the PID error in *C/min

int minT; // Typ: 25c -> 533
int maxT; // Typ: ?? -> 850
int rangeT;

Adafruit_ADT7410 *adt01;
Adafruit_BME280 *bme01;
u8g2Disp *u8disp00;
u8g2TextBox *utbTMP;
u8g2TextBox *utbSP;
// u8g2TextBox * utbHCmd;
u8g2BarGraph *ubgCtrlVar;
int bme280_ok;
int adt7410_OK;

//-----------------------------------------------------------------------------
void setStatLEDs()
{
  digitalWrite(STAT_OUT_PIN, statLedState);
  analogWrite(R_LED_OUT_PIN, rLedState);
  digitalWrite(Y_LED_OUT_PIN, yLedState);
  digitalWrite(G_LED_OUT_PIN, gLedState);
}

//-----------------------------------------------------------------------------
void statLEDsON()
{
  statLedState = true;
  rLedState = 250;
  yLedState = true;
  gLedState = true;
}

//-----------------------------------------------------------------------------
void statLEDsOFF()
{
  statLedState = false;
  rLedState = 0;
  yLedState = false;
  gLedState = false;
}

//-----------------------------------------------------------------------------
void statLEDsToggle()
{
  statLedState = !statLedState;
  rLedState == 250 ? 0 : 250;
  yLedState = !yLedState;
  gLedState = !gLedState;
}

//-----------------------------------------------------------------------------
void setupeTFT()
{

  Serial.println("Init u8g2Disp...");
  u8disp00 = new u8g2Disp();
  Serial.println("Init u8g2Disp done");

  statLEDsToggle();
  errorState = ERR_ST_NO_ERR;

  Serial.println("Init u8g2 Graphic Elements.");
  ubgCtrlVar = new u8g2BarGraph(u8disp00, BAR_GRAPH_CV_X, BAR_GRAPH_CV_Y, BAR_GRAPH_CV_H, BAR_GRAPH_CV_W, 255);

  statLEDsToggle();

  Serial.println("Init u8g2 Graphic Elements..");
  utbTMP = new u8g2TextBox(85, U8D_ROW_00, "T:XX ", u8g2_font_t0_17b_tf);

  statLEDsToggle();

  Serial.println("Init u8g2 Graphic Elements...");
  utbSP = new u8g2TextBox(8, U8D_ROW_01, "S:XX ", u8g2_font_t0_17b_tf);

  statLEDsToggle();

  // utbHCmd = new u8g2TextBox(85, U8D_ROW_01, "C:XX ", u8g2_font_t0_17b_tf);
  Serial.println("Init u8g2 Graphic Elements: DONE");

  statLEDsToggle();
}

//-----------------------------------------------------------------------------
void setup()
{
  int tmr = 0;
  int toggle = 0;
  int serialOk = 0;
  adt7410_OK = 0;
  bme280_ok = 0;
  iCount = 0;

  rLedState = 0;
  yLedState = 0;
  gLedState = 0;

  AppTitle = "Replicator_Heater_BME280";
  AppVersion = "v0.0.0";

  pinMode(STAT_OUT_PIN, OUTPUT);
  // pinMode(R_LED_OUT_PIN, OUTPUT);
  pinMode(Y_LED_OUT_PIN, OUTPUT);
  pinMode(G_LED_OUT_PIN, OUTPUT);

  pinMode(PUMP_OUT_PIN, OUTPUT);

  statLEDsToggle();

  Serial.begin(9600);

  bme01 = new Adafruit_BME280();
  adt01 = new Adafruit_ADT7410();

  statLEDsToggle();

  tmr = SER_DELAY;
  // u8disp00->writeTxt_clr("Init serial...");
  while (tmr > 0 && serialOk == 0)
  {

    if (Serial)
      serialOk = 1;
    delay(1000);

    toggle = !toggle;

    digitalWrite(STAT_OUT_PIN, toggle);
    tmr--;
  }

  statLEDsToggle();
  Serial.println("Serial Ready");

  Serial.println(AppTitle);
  Serial.println(AppVersion);

  statLEDsToggle();
  Wire.begin();
  statLEDsToggle();

  setupeTFT();

  Serial.println("Init BME280 Temp sensor...");
  bme280_ok = bme01->begin();

  statLEDsToggle();

  if (bme280_ok)
  {
    Serial.println("Init BME280 Temp sensor OK: done");
  }
  else
  {
    Serial.println("Init BME280 Temp sensor Failed");
    errorState = ERR_ST_I2C_BME_FAIL;
  }

  statLEDsToggle();

  adt7410_OK = adt01->begin(0x49);

  if (adt7410_OK)
  {
    Serial.println("Init ADT7410 Temp sensor OK: done");
  }
  else
  {
    Serial.println("Init ADT7410 Temp sensor Failed");
    errorState = ERR_ST_I2C_ADT_FAIL;
  }

  statLEDsToggle();

  opMode = OP_MD_NA;

  pwrUpStep = 0;
  knobInVar = SERVO_POS_MID;

  statLEDsToggle();

  tempLimit = MAX_TEMP_MAN;
  tempErr = 0;    //---PID Loop Err
  tempErrOvr = 0; //---PID Loop err override
  tempPV = 0;     //---PID Loop process Var
  tempPVShadow = 0;
  tempCV = 0;      //---PID Loop control Var
  tempPComp = 0;   //---PID Loop proportional control component
  tempIComp = 0;   //---PID Loop integral control component
  tempKI = KI;     //---PID Loop integral gain
  tempKP = KP;     //---PID Loop proportional gain
  tempSP = DEF_SP; //---PID Loop set point
  minT = 1023;     // Typ: 25c -> 533
  maxT = 0;        // Typ: ?? -> 850
  rangeT = 0;      //
  dT = LOOP_DLY;   //
  deltaT = 0;
  deltaErr = 0;

  statLEDsToggle();
  Serial.println(AppTitle);
  Serial.println(AppVersion);

  statLEDsToggle();
  Serial.println("Init ioChannels.");
  u8disp00->writeTxt_clr("Init ioChannels.");
  iInChan = ioChannel(IO_TYPE_AIN_NORM, I_IN_PIN, &iInVar);
  iInChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  statLEDsToggle();
  Serial.println("Init ioChannels..");
  u8disp00->writeTxt_clr("Init ioChannels..");
  knobInChan = ioChannel(IO_TYPE_AIN_3V3_255, KNOB_IN_PIN, &knobInVar);
  knobInChan.ioFilter = IO_FILT_WEIGHTED_AVG;

  statLEDsToggle();
  Serial.println("Init ioChannels...");
  u8disp00->writeTxt_clr("Init ioChannels...");
  btn00Chan = ioBtn(BTN_TYPE_MOM_ACTIVE_LOW, BTN00_IN_PIN, &btn00Var);
  u8disp00->writeTxt_clr("Init ioChannels....");
  btn01Chan = ioBtn(BTN_TYPE_MOM_ACTIVE_LOW, BTN01_IN_PIN, &btn01Var);

  statLEDsToggle();
  Serial.println("Init ioChannels....");
  u8disp00->writeTxt_clr("Init ioChannels.....");
  heatChan00 = ioChannel(IO_TYPE_DOUT_PWM, HEAT00_OUT_PIN, &heatVar00);
  heatChan01 = ioChannel(IO_TYPE_DOUT_PWM, HEAT01_OUT_PIN, &heatVar01);
  heatChan02 = ioChannel(IO_TYPE_DOUT_PWM, HEAT02_OUT_PIN, &heatVar02);

  statLEDsToggle();
  Serial.println("Init ioChannels: DONE");
  u8disp00->writeTxt_clr("Init ioChannels: DONE");

  statLEDsToggle();
}

//-----------------------------------------------------------------
float avg(int input)
{
  static int ary[SAMPLE_CNT] = {};

  float avg = 0;

  for (int x = 0; x < SAMPLE_CNT - 1; x++)
  {
    ary[x] = ary[x + 1];
    avg += ary[x];
  }

  ary[SAMPLE_CNT - 1] = input;
  avg += ary[SAMPLE_CNT - 1];
  avg /= SAMPLE_CNT;
  return avg;
}

//-----------------------------------------------------------------
void handleModeBtn(void)
{
  static char btn00Shadow = 0;
  if (btn00Var != btn00Shadow && !btn00Var)
    opMode++;

  if (opMode >= NUM_OP_MODES)
    opMode = OP_MD_IDLE;
  btn00Shadow = btn00Var;
}

//-----------------------------------------------------------------
void handleSelBtn(void)
{
  static char btn01Shadow = 0;
  if (btn00Var != btn01Shadow && !btn00Var)
    opMode++;

  if (opMode >= NUM_OP_MODES)
    opMode = OP_MD_IDLE;
  btn01Shadow = btn01Var;
}

//-----------------------------------------------------------------
void resetPID()
{
  heatVar00 = 0;
  heatVar01 = 0;

  tempErr = 0;

  tempPComp = 0;
  tempCV = 0;
}

//-----------------------------------------------------------------
void calcClosedLoopControl()
{
  tempErr = tempSP - tempPV;
  tempPComp = tempKP * tempErr;
  tempIComp += tempKI * tempErr;

  if (tempIComp > MAX_ICOMP)
    tempIComp = MAX_ICOMP;

  if (tempIComp < -MAX_ICOMP)
    tempIComp = -MAX_ICOMP;

  tempCV = tempPComp + tempIComp;

  if (tempCV > 240.0)
    tempCV = 240.0f;
  else if (tempCV < 0.0)
  {
    tempCV = 0.0f;
  }

  heatVar00 = tempCV;
}

//-----------------------------------------------------------------
int monitorPlateTemp()
{
  if (tempBMEVar < MIN_PLAUSIBLE_TEMP || tempBMEVar > MAX_PLAUSIBLE_TEMP)
  {
    opMode = OP_MD_IDLE;
    return 0;
  }
  else
    return 1;
}

//-----------------------------------------------------------------
int taskPowerUp(void)
{
  static int pwrUpCnt = 0;

  pwrUpCnt++;

  switch (pwrUpStep)
  {
  default:
  case PWRUP_NA:
    pwrUpStep = PWRUP_0;
    break;

  case PWRUP_0:
    pwrUpStep = PWRUP_DONE;
    break;

  case PWRUP_DONE:
    break;
  }

  return pwrUpStep;
}

//-----------------------------------------------------------------
void taskStatLED(void)
{

  switch (opMode)
  {
  default:
  case OP_MD_NA:
    break;

  case OP_MD_BOOT:
    statLedState = !statLedState;
    break;

  case OP_MD_PWRUP:
    statLedState = 0;

    break;

  case OP_MD_RESET:
    statLedState = 0;

    break;

  case OP_MD_IDLE:
    statLedState = 0;
    break;

  case OP_MD_SP_SET:
  case OP_MD_IND_MAN:
  case OP_MD_TEMP_HOLD:
    statLedState = 1;
    break;

  case OP_MD_RESET_MIN_MAX:
    statLedState = 1;
    break;

  case OP_MD_OLED_SETTINGS:
    statLedState = 1;
    break;

  case OP_MD_SERIAL_SETTINGS:
    statLedState = 1;
    break;
  }
}

//-----------------------------------------------------------------
void taskOLED()
{
  utbTMP->utbText = "T:" + String(tempBMEVar, 2);
  // utbHCmd->utbText = "C:" + String(heatVar00);
  utbSP->utbText = "SP:" + String(tempSP);
  static int oledCnt = 0;

  if (!(oledCnt % 2))
  {
    switch (opMode)
    {
    default:
    case OP_MD_NA:
      break;

    case OP_MD_BOOT:
      u8disp00->writeTxt_clr("Booting");
      break;

    case OP_MD_PWRUP:
      u8disp00->writeTxt_clr("Powerwup");
      break;

    case OP_MD_RESET:
      u8disp00->writeTxt_clr("Reset");
      break;

    case OP_MD_IDLE:
    case OP_MD_SP_SET:
    case OP_MD_IND_MAN:
    case OP_MD_TEMP_HOLD:
    case OP_MD_OLED_SETTINGS:
    case OP_MD_SERIAL_SETTINGS:
      u8disp00->clearBufferU8D();
      u8disp00->writeTxt(opMdOledStr[opMode]);
      u8disp00->writeTxt(utbSP);
      ubgCtrlVar->writeBuffer(heatVar00);
      u8disp00->writeTxt(utbTMP);
      u8disp00->sendBufferU8D();
      break;

    case OP_MD_RESET_MIN_MAX:

      break;
    }
  }

  oledCnt++;
}

//-----------------------------------------------------------------
void serPrintOpMode(void)
{
  //Serial.print(" OpMd:");
  Serial.print(opMode);
  Serial.print(":");
  Serial.print(opMdStr[opMode]);
}

//-----------------------------------------------------------------
void serPrintOledMode(void)
{
  Serial.print("OLED Md:");
  Serial.print(dispModeOLED);
  Serial.print(":");
  Serial.print(displayModeStr[dispModeOLED]);
}

//-----------------------------------------------------------------
void serPrintSerMode(void)
{
  Serial.print("Serial Md:");
  Serial.print(dispModeSer);
  Serial.print(":");
  Serial.print(displayModeStr[dispModeSer]);
}

//-----------------------------------------------------------------
void serOledSettingDisp()
{
  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintOledMode();

    serPrintInt("btn00", btn00Var);
    serPrintInt("btn01", btn01Var);
    serPrintInt("K", knobInVar);

    //Serial.print(" err:");
    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}

//-----------------------------------------------------------------
void serSerSettingDisp()
{
  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintSerMode();

    serPrintInt(" btn00", btn00Var);
    serPrintInt(" btn01", btn01Var);
    serPrintInt(" K", knobInVar);

    //Serial.print(" err:");
    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}

//-----------------------------------------------------------------
void serHeatManDisp()
{

  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintOpMode();

    serPrintFlt("Tb:", tempBMEVar);
    serPrintFlt("Ta:", tempADTVar);

    serPrintInt("K", knobInVar);

    serPrintInt("h0", heatVar00);
    serPrintInt("h1", heatVar01);
    serPrintInt("h2", heatVar02);

    //Serial.print(" err:");
    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}

//-----------------------------------------------------------------
void serCLDisp()
{

  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintOpMode();

    serPrintFlt("Tb:", tempBMEVar);
    serPrintFlt("Ta:", tempADTVar);
    serPrintFlt("dT:", deltaT);

    // serPrintInt("K", knobInVar);
    serPrintFlt("Err", tempErr);

    serPrintFlt("SP", tempSP);
    serPrintFlt("CV", tempCV);
    // serPrintFlt("cP", tempPComp);
    // serPrintFlt("cI", tempIComp);

    serPrintInt("h0", heatVar00);
    serPrintInt("h1", heatVar01);
    // serPrintInt("h2", heatVar02);

    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}
//-----------------------------------------------------------------
void serDefDisp()
{

  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintOpMode();
    // serPrintInt("maxT", maxT);
    // serPrintInt("minT", minT);
    // serPrintInt("rangeT", rangeT);
    // serPrintInt("ic", iCount);

    serPrintFlt("Tb:", tempBMEVar);
    serPrintFlt("Ta:", tempADTVar);
    serPrintFlt("dT:", deltaT);

    // serPrintFlt("P:", prsVar);
    // serPrintFlt("A:", altVar);
    serPrintFlt("H:", humVar);
    serPrintInt("K", knobInVar);
    serPrintFlt("Err", tempErr);

    serPrintFlt("SP", tempSP);
    serPrintFlt("CV", tempCV);
    // serPrintFlt("cP", tempPComp);
    // serPrintFlt("cI", tempIComp);

    serPrintInt("h0", heatVar00);
    serPrintInt("h1", heatVar01);
    serPrintInt("h2", heatVar02);

    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}

//-----------------------------------------------------------------
void serAtmoDisp()
{

  static int serCnt = 0;

  if (!(serCnt % 3))
  {
    serPrintOpMode();
    // serPrintInt("maxT", maxT);
    // serPrintInt("minT", minT);
    // serPrintInt("rangeT", rangeT);

    // serPrintFlt("Sel:", temp);
    serPrintFlt("Tb:", tempBMEVar);
    // serPrintFlt("Ta:", tempADTVar);
    serPrintFlt("P:", prsVar);
    serPrintFlt("A:", altVar);
    serPrintFlt("H:", humVar);

    //Serial.print(" err:");
    Serial.print(" ");
    Serial.print(errStStr[errorState]);

    Serial.print("\n");
  }

  serCnt++;
}

//-----------------------------------------------------------------
void taskSerial()
{

  switch (opMode)
  {
  default:
  case OP_MD_NA:
  case OP_MD_BOOT:
  case OP_MD_PWRUP:
  case OP_MD_RESET:
  case OP_MD_IDLE:
  case OP_MD_TEMP_HOLD:
  case OP_MD_RESET_MIN_MAX:
    serAtmoDisp();
    break;

  case OP_MD_SP_SET:
    serCLDisp();
    break;

  case OP_MD_IND_MAN:
    serHeatManDisp();
    break;

  case OP_MD_OLED_SETTINGS:
    serOledSettingDisp();
    break;

  case OP_MD_SERIAL_SETTINGS:
    serSerSettingDisp();
    break;
  }
}

//-----------------------------------------------------------------
void taskOpMode(void)
{

  switch (opMode)
  {
  default:
  case OP_MD_NA:
    //--Go directly to BOOT mode---
    opMode = OP_MD_BOOT;
    break;

  case OP_MD_BOOT:
    //---Stay in BOOT mode for 100 ticks---
    if (iCount < 50)
      opMode = OP_MD_BOOT;
    else
      opMode = OP_MD_PWRUP;
    break;

  case OP_MD_PWRUP:
    //---Stay in PWRUP mode until it's done---
    if (taskPowerUp() == PWRUP_DONE)
      opMode = OP_MD_IDLE;
    else
      opMode = OP_MD_PWRUP;
    handleModeBtn();
    handleSelBtn();
    break;

  case OP_MD_RESET:
  case OP_MD_IDLE:
  case OP_MD_SP_SET:
  case OP_MD_TEMP_HOLD:
  case OP_MD_IND_MAN:

    handleModeBtn();
    handleSelBtn();
    break;

  case OP_MD_RESET_MIN_MAX:
    minT = 1023;
    maxT = 0;
    opMode = OP_MD_OLED_SETTINGS;
    handleModeBtn();
    handleSelBtn();
    break;

  case OP_MD_OLED_SETTINGS:
  case OP_MD_SERIAL_SETTINGS:
    handleModeBtn();
    handleSelBtn();
    break;
  }
}

//-----------------------------------------------------------------
void taskHeat()
{
  static int knobProxy = 0;
  static int opModeShadow = opMode;
  knobInVar -= 7;
  knobProxy = constrain(knobInVar, KNOB_MIN, KNOB_MAX);
  heatVar02 = 0;

  if (tempPV > maxT)
    maxT = tempPV;

  if (tempPV < minT)
    minT = tempPV;

  rangeT = maxT - minT;

  deltaT = tempPV - tempPVShadow;

  switch (opMode)
  {
  default:
    resetPID();
    break;

  case OP_MD_NA:
  case OP_MD_BOOT:
  case OP_MD_PWRUP:
  case OP_MD_RESET:
    resetPID();
    break;

  case OP_MD_IDLE:
    resetPID();
    break;

  case OP_MD_SP_SET:
    tempSP = (knobProxy / 2.0) + 20;
    calcClosedLoopControl();
    monitorPlateTemp();
    break;

  case OP_MD_TEMP_HOLD:
    if (opMode != opModeShadow)
      tempSP = tempPV;
    calcClosedLoopControl();
    monitorPlateTemp();
    break;

  case OP_MD_IND_MAN:
    resetPID();
    monitorPlateTemp();

    heatVar02 = knobProxy;
    break;

  case OP_MD_RESET_MIN_MAX:
    resetPID();
    break;
  }
  heatVar01 = 255 - heatVar00;
  opModeShadow = opMode;

  tempPVShadow = tempPV;
  tempADTVarShadow = tempADTVar;
  tempBMEVarShadow = tempBMEVar;
}

//-----------------------------------------------------------------
void tempCtrl(void)
{
  // if (errorState == ERR_ST_NO_ERR) {

  //   if (tempPV > tempLimit) {
  //     errorState = ERR_ST_OVERTEMP;
  //   }
  //   else if (tempPV < (tempLimit - TEMP_HYST))
  //     errorState = ERR_ST_NO_ERR;
  // }
  //   heatVar00 = 0;
}

//-----------------------------------------------------------------
void taskManager(void)
{

  static int taskIdx = 0;

  switch (taskIdx)
  {

  default:
  case TASK_SAWTOOTH:
  case TASK_VOLTAGE_MON:
    break;

  case TASK_STAT_LED:
    taskStatLED();
    break;

  //---Analog to momentary switch
  case TASK_BTN_READ:
    //---TODO: generalize to pass button structure/class as arg---
    // taskBtn();
    break;

  //---Op mode state engine task---
  case TASK_OP_MODE:
    taskOpMode();
    break;

  case TASK_HEAT_CTRL:
    taskHeat();
    break;

  case TASK_SERIAL:
    taskSerial();
    break;

  case TASK_OLED:
    taskOLED();
    break;
  }

  taskIdx++;
  if (taskIdx > NUM_TASKS)
    taskIdx = TASK_STAT_LED;
}

//-----------------------------------------------------------------
void loop(void)
{
  knobInChan.procInChan();
  iInChan.procInChan();
  btn00Chan.procInBtn();
  btn01Chan.procInBtn();

  if (adt7410_OK) {
    tempADTVar = adt01->readTempC();
  }
  else {
    tempADTVar = 0;
    tempPV = 0;
  }

  if (bme280_ok)  {
    tempBMEVar = bme01->readTemperature();
    tempPV = tempBMEVar;
    prsVar = bme01->readPressure() / 100.0F;
    altVar = bme01->readAltitude(SEALEVELPRESSURE_HPA);
    humVar = bme01->readHumidity();
  }
  else {

    tempBMEVar = 0;
    tempPV = 0;
    prsVar = 0;
    altVar = 0;
    humVar = 0;
  }

  taskManager();

  tempCtrl();
  heatChan00.procOutChan();
  heatChan01.procOutChan();
  heatChan02.procOutChan();
  analogWrite(Y_LED_OUT_PIN, 255 - heatVar02);
  analogWrite(R_LED_OUT_PIN, 255 - heatVar00);
  digitalWrite(STAT_OUT_PIN, statLedState);

  iCount++;
  delay(LOOP_DLY);
}
