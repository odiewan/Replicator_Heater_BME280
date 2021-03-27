#include "main.h"

int opMode;
int pwrUpStep;
int statLedState;
int rLedState;
int yLedState;
int gLedState;

int iCount;

float tempVar;
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

int btnVar;
ioBtn btnChan;

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
float tempKI;
float tempKP;
float tempPComp;
float tempIComp;
u_int16_t dT;


float deltaT; //Time rate of change of the Heater plate in *C/min
float deltaErr; //Time rate of chane of the PID error in *C/min


int minT; // Typ: 25c -> 533
int maxT;    // Typ: ?? -> 850
int rangeT;


Adafruit_BME280 *bme01;
u8g2Disp * u8disp00;
u8g2TextBox * utbTMP;
u8g2TextBox * utbSP;
// u8g2TextBox * utbHCmd;
u8g2BarGraph * ubgCtrlVar;
int bme280_ok;


//-----------------------------------------------------------------------------
void statLEDsON() {
  digitalWrite(STAT_OUT_PIN, LOW);
  analogWrite(R_LED_OUT_PIN, 0);
  digitalWrite(Y_LED_OUT_PIN, LOW);
  digitalWrite(G_LED_OUT_PIN, LOW);
}

//-----------------------------------------------------------------------------
void statLEDsOFF() {
  digitalWrite(STAT_OUT_PIN, HIGH);
  analogWrite(R_LED_OUT_PIN, 250);
  digitalWrite(Y_LED_OUT_PIN, HIGH);
  digitalWrite(G_LED_OUT_PIN, HIGH);
}


//-----------------------------------------------------------------------------
void setup() {
  int tmr = 0;
  int toggle = 0;
  int serialOk = 0;
  bme280_ok = 0;
  iCount = 0;


  AppTitle = "Replicator_Heater_BME280";
  AppVersion = "v0.0.0";

  pinMode(STAT_OUT_PIN, OUTPUT);
  // pinMode(R_LED_OUT_PIN, OUTPUT);
  pinMode(Y_LED_OUT_PIN, OUTPUT);
  pinMode(G_LED_OUT_PIN, OUTPUT);

  pinMode(PUMP_OUT_PIN, OUTPUT);

  statLEDsON();


  Serial.begin(9600);

  bme01 = new Adafruit_BME280();

  statLEDsON();

  tmr = SER_DELAY;
  // u8disp00->writeTxt_clr("Init serial...");
  while (tmr > 0 && serialOk == 0) {

    if (Serial)
      serialOk = 1;
    delay(1000);

    toggle = !toggle;

    digitalWrite(STAT_OUT_PIN, toggle);
    tmr--;
  }

  statLEDsOFF();
  Serial.println("Serial Ready");

  Serial.println(AppTitle);
  Serial.println(AppVersion);

  statLEDsON();
  Wire.begin();
  statLEDsOFF();

  Serial.println("Init u8g2Disp...");
  u8disp00 = new u8g2Disp();
  Serial.println("Init u8g2Disp done");

  statLEDsON();

  Serial.println("Init u8g2 Graphic Elements.");
  ubgCtrlVar = new u8g2BarGraph(u8disp00, BAR_GRAPH_CV_X, BAR_GRAPH_CV_Y, BAR_GRAPH_CV_H, BAR_GRAPH_CV_W, 255);

  statLEDsOFF();

  Serial.println("Init u8g2 Graphic Elements..");
  utbTMP = new u8g2TextBox(85, U8D_ROW_00, "T:XX ", u8g2_font_t0_17b_tf);

  statLEDsON();

  Serial.println("Init u8g2 Graphic Elements...");
  utbSP = new u8g2TextBox(8, U8D_ROW_01, "S:XX ", u8g2_font_t0_17b_tf);

  statLEDsOFF();

  // utbHCmd = new u8g2TextBox(85, U8D_ROW_01, "C:XX ", u8g2_font_t0_17b_tf);
  Serial.println("Init u8g2 Graphic Elements: DONE");

  statLEDsON();

  Serial.println("Init BME280 Temp sensor...");
  bme280_ok = bme01->begin();

  statLEDsOFF();

  if(bme280_ok)
    Serial.println("Init BME280 Temp sensor OK: done");
  else
    Serial.println("Init BME280 Temp sensor Failed");

  statLEDsON();

  opMode = OP_MD_NA;
  errorState = ERR_ST_NO_ERR;
  pwrUpStep = 0;
  knobInVar = SERVO_POS_MID;


  statLEDsOFF();


  tempLimit = MAX_TEMP_MAN;
  tempErr = 0;              //---PID Loop Err
  tempErrOvr = 0;           //---PID Loop err override
  tempPV = 0;               //---PID Loop process Var
  tempCV = 0;               //---PID Loop control Var
  tempPComp = 0;            //---PID Loop proportional control component
  tempIComp = 0;            //---PID Loop integral control component
  tempKI = KI;              //---PID Loop integral gain
  tempKP = KP;              //---PID Loop proportional gain
  tempSP = DEF_SP;          //---PID Loop set point
  minT = 1023;              // Typ: 25c -> 533
  maxT = 0;                 // Typ: ?? -> 850
  rangeT = 0;               //
  dT = LOOP_DLY;            //
  deltaT = 0;
  deltaErr = 0;


  statLEDsON();
  Serial.println(AppTitle);
  Serial.println(AppVersion);

  statLEDsOFF();
  Serial.println("Init ioChannels.");
  u8disp00->writeTxt_clr("Init ioChannels.");
  iInChan = ioChannel(IO_TYPE_AIN_NORM, I_IN_PIN, &iInVar);
  iInChan.ioFilter = IO_FILT_WEIGHTED_AVG;


  statLEDsON();
  Serial.println("Init ioChannels..");
  u8disp00->writeTxt_clr("Init ioChannels..");
  knobInChan = ioChannel(IO_TYPE_AIN_3V3_255, KNOB_IN_PIN, &knobInVar);
  knobInChan.ioFilter = IO_FILT_WEIGHTED_AVG;


  statLEDsOFF();
  Serial.println("Init ioChannels...");
  u8disp00->writeTxt_clr("Init ioChannels...");
  btnChan = ioBtn(BTN_TYPE_MOM_ACTIVE_LOW, BTN_IN_PIN, &btnVar);

  statLEDsON();
  Serial.println("Init ioChannels....");
  u8disp00->writeTxt_clr("Init ioChannels....");
  heatChan00 = ioChannel(IO_TYPE_DOUT_PWM, HEAT00_OUT_PIN, &heatVar00);
  heatChan01 = ioChannel(IO_TYPE_DOUT_PWM, HEAT01_OUT_PIN, &heatVar01);
  heatChan02 = ioChannel(IO_TYPE_DOUT_PWM, HEAT02_OUT_PIN, &heatVar02);

  statLEDsOFF();
  Serial.println("Init ioChannels: DONE");
  u8disp00->writeTxt_clr("Init ioChannels: DONE");

  statLEDsON();
}


//-----------------------------------------------------------------
void serPrintOpMode(void) {
  //Serial.print(" OpMd:");
  Serial.print(opMode);
  Serial.print(":");
  Serial.print(opMdStr[opMode]);
}

//-----------------------------------------------------------------
float avg(int input) {
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
void handleModeBtn(void) {
  static char btnShadow = 0;
  if (btnVar != btnShadow && !btnVar)
    opMode++;

  if (opMode > OP_MD_IND_MAN)
    opMode = OP_MD_IDLE;
  btnShadow = btnVar;
}

//-----------------------------------------------------------------
void resetPID() {
  heatVar00 = 0;
  heatVar01 = 0;

  tempErr = 0;

  tempPComp = 0;
  tempCV = 0;
}

//-----------------------------------------------------------------
void calcClosedLoopControl() {
  tempErr = tempSP - tempVar;
  tempPComp = tempKP * tempErr;
  tempIComp += tempKI * tempErr;

  if (tempIComp > MAX_ICOMP)
    tempIComp = MAX_ICOMP;

  if (tempIComp < -MAX_ICOMP)
    tempIComp = -MAX_ICOMP;


  tempCV = tempPComp + tempIComp;

  if (tempCV > 240.0)
    tempCV = 240.0;
  else if (tempCV < 0.0) {
    tempCV = 0.0;
  }
  heatVar00 = tempCV;
}

//-----------------------------------------------------------------
int monitorPlateTemp() {
  if(tempVar < MIN_PLAUSIBLE_TEMP || tempVar > MAX_PLAUSIBLE_TEMP) {
    opMode = OP_MD_IDLE;
    return 0;
  }
  else
    return 1;
}


//-----------------------------------------------------------------
int taskPowerUp(void) {
  static int pwrUpCnt = 0;

  pwrUpCnt++;

  switch (pwrUpStep) {
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
void taskStatLED(void) {

  switch (opMode)  {
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
  }
}

//-----------------------------------------------------------------
void taskOLED() {
  utbTMP->utbText = "T:" + String(tempVar, 2);
  // utbHCmd->utbText = "C:" + String(heatVar00);
  utbSP->utbText = "SP:" + String(tempSP);
  static int oledCnt = 0;

  if (!(oledCnt % 2)) {
    switch (opMode) {
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
void taskSerial() {
  static int serCnt = 0;

  if(!(serCnt % 3)) {
      serPrintOpMode();
      // Serial.print(" u8dDefTB->utbX:" + u8disp00->getX());
      // Serial.print(" u8dDefTB->utbY:" + u8disp00->getY());
      // serPrintInt("maxT", maxT);
      // serPrintInt("minT", minT);
      // serPrintInt("rangeT", rangeT);
      // serPrintInt("ic", iCount);
      tempVar = bme01->readTemperature();
      prsVar = bme01->readPressure() / 100.0F;
      altVar = bme01->readAltitude(SEALEVELPRESSURE_HPA);
      humVar = bme01->readHumidity();

      serPrintFlt("T:", tempVar);
      serPrintFlt("P:", prsVar);
      serPrintFlt("A:", altVar);
      serPrintFlt("H:", humVar);

      // serPrintInt("vInVar", vInVar);
      // serPrintInt("iInVar", iInVar);
      // serPrintInt(":", iInChan.ioRawVal);

      // tmpFloat = avg(iInChan.ioRawVal);
      // serPrintFlt("iInVar a:", tmpFloat);
      // serPrintInt("btn", btnVar);

      // serPrintInt("ledSt", statLedState);
      // serPrintInt("blinkS", blinkS);

      serPrintInt("K", knobInVar);
      // serPrintInt("KnobR", knobInChan.ioRawVal);


      // serPrintInt("heatR", heatVar);
      serPrintFlt("Err", tempErr);

      serPrintFlt("SP", tempSP);
      serPrintFlt("CV", tempCV);
      // serPrintFlt("cP", tempPComp);
      // serPrintFlt("cI", tempIComp);

      // Serial.print(" w:");
      // Serial.print(ubgCtrlVar->ubgWCurr);
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
void taskOpMode(void) {


  switch (opMode) {
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
      break;

    case OP_MD_RESET:
    case OP_MD_IDLE:
    case OP_MD_TEMP_HOLD:
    case OP_MD_IND_MAN:

      handleModeBtn();
      break;


    case OP_MD_RESET_MIN_MAX:
      minT = 1023;
      maxT = 0;
      opMode = OP_MD_IDLE;
      handleModeBtn();
      break;

    case OP_MD_SP_SET:

      handleModeBtn();
      break;
  }

}


//-----------------------------------------------------------------
void taskHeat() {
  static int knobProxy = 0;
  static int opModeShadow = opMode;
  knobInVar -= 7;
  knobProxy = constrain(knobInVar, KNOB_MIN, KNOB_MAX);
  heatVar02 = 0;

  if (tempVar > maxT)
    maxT = tempVar;

  if (tempVar < minT)
    minT = tempVar;

  rangeT = maxT - minT;

  switch (opMode) {
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
        tempSP = tempVar;
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
  heatVar01 = heatVar00;
  opModeShadow = opMode;
}

//-----------------------------------------------------------------
void tempCtrl(void) {
  if (tempVar > tempLimit) {
    errorState = ERR_ST_OVERTEMP;
  }
  else if (tempVar < (tempLimit - TEMP_HYST))
    errorState = ERR_ST_NO_ERR;

  if (errorState == ERR_ST_OVERTEMP)
    heatVar00 = 0;

}

//-----------------------------------------------------------------
void taskManager(void) {

  static int taskIdx = 0;


  switch (taskIdx) {

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
void loop(void) {
  knobInChan.procInChan();
  iInChan.procInChan();
  btnChan.procInBtn();

  taskManager();

  tempCtrl();
  heatChan00.procOutChan();
  heatChan01.procOutChan();
  heatChan02.procOutChan();
  analogWrite(Y_LED_OUT_PIN, 255-heatVar02);
  analogWrite(R_LED_OUT_PIN, 255-heatVar00);
  digitalWrite(STAT_OUT_PIN, statLedState);

  iCount++;
  delay(LOOP_DLY);
}
