/* Standard Includes */
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

/* LTC Includes */
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"

/* Custom Includes */
#include "pinModes.h"
#include <mcp2518fd_can.h>
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* LTC Defines */
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/* User Defines */
#define CCS 10                    // MCP chip select pin
#define TOTAL_IC 18               // Total number of IC's/Total number of stacks
#define TOTAL_CELL 6              // Total number of cells per stack => Used for reading voltage values
#define TEMPS 4                   // Total number of cells per stack => Used for reading temperature values
#define UV 30000                  // Under voltage limit
#define OV 41000                  // Over voltage limit
#define UT 10                     // Under temperature limit
#define OT 45                     // Over temperature limit
#define MEASUREMENT_LOOP_TIME 20  // Sampling period per iteration for mainLoop()
#define progLED1 42               // Pin number for programmable LED 1
#define progLED2 43               // Pin number for programmable LED 2
#define progLED3 44               // Pin number for programmable LED 3
#define progLED4 45               // Pin number for programmable LED 4
#define CSOUTtemp A15
#define cartId_pin 9
#define chargerAux_pin 6
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Global Variables */
// ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED;          // ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;  // ADC Mode
const uint8_t ADC_DCP = DCP_ENABLED;               // Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;    // Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;      // Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;    // Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;               // Register Selection
const uint8_t SEL_REG_A = REG_1;                   // Register Selection
const uint8_t SEL_REG_B = REG_2;                   // Register Selection

// Loop Measurement Setup
const uint8_t WRITE_CONFIG = ENABLED;  // This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = ENABLED;   // This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED;  // This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = ENABLED;   // This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = ENABLED;  // This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = ENABLED;     // This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* User Variables */
// Iterators
uint8_t actual_index = 0;  // Iterator for actual data array
uint8_t can_index = 0;     // Iterator for can data array

// Booleans
bool voltfaultStatus[TOTAL_IC][TOTAL_CELL];  // Fault status variable => Will trigger AMS Fault
bool tempfaultStatus[TOTAL_IC][TEMPS];       // Fault status variable => Will trigger AMS Fault
bool ntcfaultStatus[TOTAL_IC][TEMPS];
bool canSend_status;
bool chargerCAN_status;

// Counters
uint8_t mainLoopCounter = 0;  // Counter variable to check whether main loop has run 10 times

// Timers
unsigned long current_loopTime;           // Timer variable to hold current loop time
static uint32_t next_loopTime;            // Timer vairable to hold prev loop time
unsigned long start_timer, current_time;  // Timer for SoC Estimation

// Readings
uint16_t avg_cellVoltage, avg_cellTemperature;  // Variable to hold average of 10 readings of cell voltage and temperature
double minCV, maxCV;                            // Variables for min and max cell voltage
double minCT, maxCT;                            // Variables for min and max cell temperature
float soc = 100.0;                              // SoC Variable
uint8_t VfaultIc, VfaultCell;
uint8_t TfaultIc, TfaultCell;

// Constants
const double r_inf = 10000 * exp(-3435 / 298.15);  // Variable for temperature conversion
const double csFault_value = 10.00;                // Variable for current sensor fault check => Change based on CS testing.
const float maxCapacity = 4.5;                     // Maximum Capacity of MOLICEL INR-21700-P45B
const int period = 200;                            // To calculate SoC every 200ms
const float sensorSensitivity = 0.0057;            // Sensor sensitivity in V/A
const float vRef = 2.4929;                         // Reference voltage
const float analogResolution = 5.0 / 1023.0;

// Variables
double currentSensor;                                         // Variable to store current sensor reading
double R;                                                     // Variable for temperature conversion
double vRef2;                                                 // Reference voltage for temperature conversions
float pcrDone = 0, airP = 0, airN = 0, pcr5v = 0, vsBat = 0,  // Variables for lvData
  vCheck = 0, tCheck = 0, vsBatin = 0, vsHVin = 0,
      amsFaultout = 0, k2 = 0, k1 = 0, ssFinal = 0, grnIn = 0,
      grnOut = 0, auxP = 0, auxN = 0, pcrAuxin = 0;
double vsBATvoltage, vsHVvoltage;
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* User Defined Arrays */
// Readings
uint32_t cellVoltage[TOTAL_IC][TOTAL_CELL];                          // Array to hold voltage values for each cell
uint32_t cellTemperature[TOTAL_IC][TEMPS];                           // Array to hold temperature values for each cell
uint8_t canData[2 * (TOTAL_IC * (TEMPS + TOTAL_CELL) + 4)] = { 0 };  // Array to hold the lowByte and highByte of every value in actual_data with 1 cs, 2 vs, 1 soc
uint16_t allData[TOTAL_IC * (TEMPS + TOTAL_CELL) + 4] = { 0 };       // Array to hold the values of voltage and Temperature readings
float stack_voltage[TOTAL_IC];                                       // Array to hold stack voltage
uint16_t sensorValue[TOTAL_IC][TOTAL_CELL] = { 0 };
uint16_t deltaVoltage[TOTAL_IC][TOTAL_CELL] = { 0 };  // Array to hold cell number with delta voltage > 20mV

// Counters
uint16_t volt_faultCounter[TOTAL_IC][TOTAL_CELL] = { 0 };  // Array to hold voltage fault count for each cell
uint16_t temps_faultCounter[TOTAL_IC][TEMPS] = { 0 };      // Array to hold temperature fault count for each cell
uint16_t ntc_faultCounter[TOTAL_IC][TEMPS] = { 0 };

// Constants
const byte can_ids[46] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                           0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
                           0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46 };  // CAN IDs for each Message

unsigned char BMSA[8] = { 0x11, 0xB8, 0x00, 0x64, 0x00, 0x00, 0xFF, 0xFF };  // Byte 0 and 1 -> Max TS Voltage. Byte 2 and 3 -> Max Charging current.
                                                                             // Byte 4 and 5 -> Charger related settings. Byte 6 and 7 -> Retain.
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Struct initialization */
cell_asic BMS_IC[TOTAL_IC];  // Initializing main struct to hold cell voltage, temperature values and stack voltage value.
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Configuration bit setup */
bool REFON = true;                                       // Reference Powered Up Bit
bool ADCOPT = false;                                     // ADC Mode option bit
bool gpioBits_a[5] = { true, true, true, true, false };  // GPIO Pin Control => Gpio {1,2,3,4,5} /* CHANGE GPIO 5 TO TRUE AND CHECK. IT CAN BE CONFIGURED AS SPI MASTER -> Didn't make a difference*/
bool dccBits_a[12] = { false, false, false, false,
                       false, false, false, false,
                       false, false, false, false };  // Discharge cell switch => Dcc {1,2,3,4,5,6,7,8,9,10,11,12}
bool dctoBits[4] = { false, false, false, false };    // Discharge time value => Dcto {0,1,2,3}. Programed for 4 min
/* !!Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet!! */
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
// User Defined Functions
void initializeLTC();
void initializeMCP();
void measurementLoop();
void print_wrconfig(void);
void print_rxconfig(void);
void check_error(int);
void serialPrintHex(uint8_t);
void printStackVoltage(void);
void print(uint8_t, uint8_t, uint16_t, bool, bool);
void faultCounter(uint16_t, uint16_t, bool, uint8_t, uint8_t);
void BMSData(void);
double tempCalc(uint8_t, uint8_t);
void AvgBMSData(void);
void faultCheck(void);
double minmaxCV(void);
void minmaxCT(void);
float readCurrent(void);
void SoC(void);
void csFault_check(void);
void charging(void);
void lvData(void);
void canSend(void);
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
//MCP_CAN CAN(CCS); // CAN object for MCP2515
mcp2518fd CAN(CCS);  // CAN object for MCP2518FD
int mainFaultStatus = 0;
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
void setup() {
  Serial.begin(115200);  // Setting baud rate for Serial Monitor
  while (!Serial);
  pinMode(16, OUTPUT);  // isoSPI LED
  pinMode(10, OUTPUT);  //MCP CS

  initializeMCP();  // Initialize MCP -> For both MCP2515 and MCP2518FD
  initializeLTC();  // Initialize LTC

  int8_t error;

  if (WRITE_CONFIG == ENABLED) {
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, BMS_IC);
    print_wrconfig();
  }
  if (READ_CONFIG == ENABLED) {
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    print_rxconfig();
  }
  digitalWrite(VCHECK, HIGH);
}

void loop() {
  int cartId = digitalRead(cartId);
  int chargerAux = digitalRead(chargerAux);
  if (millis() - next_loopTime >= MEASUREMENT_LOOP_TIME) {
  bool voltageFault = voltage_faultCheck();
  bool tempFault = temperature_faultCheck();
    measurementLoop();
    BMSData();
    if (cartId == 1) {
      if (chargerAux == 1) {
        charging();
      } else {
        Serial.println("Not Charging");
      }
    }
    mainLoopCounter += 1;
    if (mainLoopCounter >= 10) {
      faultCheck();
      csFault_check();
      Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
      AvgBMSData();
      minmaxCV();
      minmaxCT();
      canSend();
      if (canSend_status == true) {
        Serial.println("DATA SENT SUCCESSFULLY");
      }
      if (chargerCAN_status == true) {
        Serial.println("Recieved Correct CAN ID from charger.");
      } else {
        Serial.println("(CHARGER): No CAN id message recieved");
      }
      Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
      lvData();
      mainLoopCounter = 0;
    }
    // if ((readCurrent() < csFault_value || readCurrent() > -400) && (BMS_IC[VfaultIc].cells.c_codes[VfaultCell] < OV || BMS_IC[VfaultIc].cells.c_codes[VfaultCell] > UV) && (tempCalc(TfaultIc, TfaultCell) > UT || tempCalc(TfaultIc, TfaultCell) < OT)) {
    //   digitalWrite(AMS_FAULT_SDC, HIGH);
    //   digitalWrite(AMS_FAULT_PIN, LOW);
    // }
    spi_enable(SPI_CLOCK_DIV16);
    next_loopTime += MEASUREMENT_LOOP_TIME;
  }
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* Initialization Functions */
void initializeLTC() {
  spi_enable(SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
  LTC6811_init_cfg(TOTAL_IC, BMS_IC);
  for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    LTC6811_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, gpioBits_a, dccBits_a, dctoBits, UV, OV);
  }
  LTC6811_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6811_init_reg_limits(TOTAL_IC, BMS_IC);
}

void initializeMCP() {
  Serial.println("CAN Initializing....");
  // if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  // {
  //   Serial.println("CAN SUCCESS");
  // }
  // else if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_FAILINIT)
  // {
  //   Serial.println("CAN INITIALIZATION FAILED");
  // }
  // else
  // {
  //   Serial.println("MCP FAILED");
  // }

  // while(!Serial);

  CAN.setMode(CAN_CLASSIC_MODE);

  while (CAN_OK != CAN.begin(CAN20_500KBPS)) {  // init can bus : baudrate = 1000k
    Serial.println("CAN INITIALIZATION FAILED");
  }
  Serial.println("CAN SUCCESS");
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* LTC Defined Functions */
void measurementLoop() {
  int8_t error;

  if (MEASURE_CELL == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }

  if (MEASURE_AUX == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adax(SEL_ALL_REG, AUX_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }

  if (MEASURE_STAT == ENABLED) {
    wakeup_idle(TOTAL_IC);
    LTC6811_adstat(SEL_ALL_REG, STAT_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC);
    check_error(error);
  }
}

void check_error(int error) {
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

void print_wrconfig(void) {
  Serial.println(F("Written Config:"));
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    Serial.print(F("CFGA IC "));
    Serial.print(currentIc + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serialPrintHex(BMS_IC[currentIc].config.tx_data[i]);
    }
    Serial.print(F(", Calculated PEC: 0x"));
    int cfg_pec = pec15_calc(6, &BMS_IC[currentIc].config.tx_data[0]);
    serialPrintHex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serialPrintHex((uint8_t)(cfg_pec));
    Serial.println();
  }
}

void print_rxconfig(void) {
  Serial.println(F("Received Config:"));
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    Serial.print(F("CGFA IC "));
    Serial.print(currentIc + 1, DEC);
    for (int i = 0; i < 6; i++) {
      Serial.print(F(", 0x"));
      serialPrintHex(BMS_IC[currentIc].config.rx_data[i]);
    }
    Serial.print(F(", Received PEC: 0x"));
    serialPrintHex(BMS_IC[currentIc].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serialPrintHex(BMS_IC[currentIc].config.rx_data[7]);
    Serial.println();
  }
}

void printStackVoltage() {
  double accuVoltage = 0;
  const uint8_t printOrder[3][6] = {
    { 18, 13, 12, 7, 6, 1 },
    { 17, 14, 11, 8, 5, 2 },
    { 16, 15, 10, 9, 4, 3 }
  };

  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 6; col++) {
      uint8_t ic = printOrder[row][col] - 1;

      if (ic >= TOTAL_IC) {
        // Print stack number with alignment
        Serial.print("S");
        if (printOrder[row][col] < 10) Serial.print("0");  // pad single digit stack numbers
        Serial.print(printOrder[row][col]);
        Serial.print(": ");

        // Error message
        Serial.print("NotConn ");
      } else {
        float voltage = BMS_IC[ic].stat.stat_codes[0] * 0.0001 * 20;
        // Print stack number with alignment
        Serial.print("S");
        if (printOrder[row][col] < 10) Serial.print("0");  // pad single digit stack numbers
        Serial.print(printOrder[row][col]);
        Serial.print(": ");

        // Voltage always between 0.0000 and 25.2000, so fixed width
        Serial.print(voltage, 4);
      }

      if (col < 5) Serial.print(" | ");
    }
    Serial.println();
  }
}

void serialPrintHex(uint8_t data) {
  if (data < 16) {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  } else {
    Serial.print((byte)data, HEX);
  }
}

void print_conv_time(uint32_t conv_time) {
  uint16_t m_factor = 1000;  // to print in ms

  Serial.print(F("Conversion completed in:"));
  Serial.print(((float)conv_time / m_factor), 1);
  Serial.println(F("ms \n"));
}
/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* User defined functions */
void print(uint8_t ic, uint8_t cell, uint16_t value, bool temp = false, bool fault = false) {
  Serial.print("S");
  Serial.print(ic + 1, DEC);
  Serial.print("C");
  Serial.print(cell + 1, DEC);
  Serial.print(": ");

  if (temp == false) {
    Serial.print(value * 0.0001, 4);
  } else {
    Serial.print(value);
  }
  if (fault) {
    Serial.print(" !FAULT! ");
  }
  Serial.print(", ");
}

void faultCounter(uint16_t value, bool isTemp, uint8_t ic, uint8_t cellno) {
  bool ntcFault;
  if (!isTemp) {
    if (value > OV || value < UV) {
      volt_faultCounter[ic][cellno] += 1;
    } else {
      volt_faultCounter[ic][cellno] = 0;
    }
  } else {
    if (value > OT || value < UT) {
      temps_faultCounter[ic][cellno] += 1;
    } else {
      temps_faultCounter[ic][cellno] = 0;
    }
    if (value > 65000) {
      digitalWrite(AMS_FAULT_SDC, LOW);   // AMS FAULT PIN on CART
      //digitalWrite(AMS_FAULT_PIN, HIGH);  // AMS FAULT PIN on CAR
      digitalWrite(progLED2, HIGH);
      Serial.println("NTC DISCONNECTED");
    }
    
  }
}

double tempCalc(uint8_t ic, uint8_t temp) {
  vRef2 = BMS_IC[ic].aux.a_codes[5] * 0.0001;
  R = BMS_IC[ic].aux.a_codes[temp] / (vRef2 - (BMS_IC[ic].aux.a_codes[temp] * 0.0001));
  return ((3435 / log(R / r_inf)) - 273.15);
}

void BMSData() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    bool isTemp = false;
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      cellVoltage[currentIc][cell] += BMS_IC[currentIc].cells.c_codes[cell];
      faultCounter(BMS_IC[currentIc].cells.c_codes[cell], isTemp, currentIc, cell);
    }
    isTemp = true;
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      cellTemperature[currentIc][temp] += tempCalc(currentIc, temp);
      faultCounter(tempCalc(currentIc, temp), isTemp, currentIc, temp);
    }
  }
}

bool voltage_faultCheck() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      if (volt_faultCounter[currentIc][cell] == 10) {
          voltfaultStatus[currentIc][cell] = true;
          volt_faultCounter[currentIc][cell] = 0;
          return true;
      }
      else {
          voltfaultStatus[currentIc][cell] = false;
          return false;
      }
    }
  }
}

bool temperature_faultCheck() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      if (volt_faultCounter[currentIc][temp] == 10) {
          voltfaultStatus[currentIc][temp] = true;
          volt_faultCounter[currentIc][temp] = 0;
          return true;
      }
      else {
          voltfaultStatus[currentIc][temp] = false;
          return false;
      }
    }
  }
}

void faultCheck() {
  bool fault;
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      if (cell < TEMPS) {
        if ((temps_faultCounter[currentIc][cell] == 10 || volt_faultCounter[currentIc][cell] == 10) ) {
          tempfaultStatus[currentIc][cell] = true;
          temps_faultCounter[currentIc][cell] = 0;
          voltfaultStatus[currentIc][cell] = true;
          volt_faultCounter[currentIc][cell] = 0;
          digitalWrite(AMS_FAULT_SDC, LOW);   // AMS FAULT PIN on CART
          //digitalWrite(AMS_FAULT_PIN, HIGH);  // AMS FAULT PIN on CAR
          digitalWrite(progLED2, HIGH);
          digitalWrite(progLED1, HIGH);
        } else {
          tempfaultStatus[currentIc][cell] = false;
          voltfaultStatus[currentIc][cell] = false;
          digitalWrite(AMS_FAULT_SDC, HIGH);  // AMS FAULT PIN on CART
          //digitalWrite(AMS_FAULT_PIN, LOW);   // AMS FAULT PIN on CAR
          digitalWrite(progLED2, LOW);
          digitalWrite(progLED1, LOW);
        }
      } else {
        if (volt_faultCounter[currentIc][cell] >= 10) {
          voltfaultStatus[currentIc][cell] = true;
          volt_faultCounter[currentIc][cell] = 0;
          digitalWrite(AMS_FAULT_SDC, LOW);   // AMS FAULT PIN on CART
          //digitalWrite(AMS_FAULT_PIN, HIGH);  // AMS FAULT PIN on CAR
          digitalWrite(progLED1, HIGH);
          
        } else {
          voltfaultStatus[currentIc][cell] = false;
          digitalWrite(AMS_FAULT_SDC, HIGH);  // AMS FAULT PIN on CART
          //digitalWrite(AMS_FAULT_PIN, LOW);   // AMS FAULT PIN on CAR
          digitalWrite(progLED1, LOW);
        }
      }
    }
  }
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

void AvgBMSData() {
  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    bool isTemp = false;
    Serial.println("Voltage Readings");
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      avg_cellVoltage = cellVoltage[currentIc][cell] / 10;
      print(currentIc, cell, avg_cellVoltage, isTemp, voltfaultStatus[currentIc][cell]);
      cellVoltage[currentIc][cell] = 0;
    }
    Serial.println();
    Serial.println("Temperature Readings");
    isTemp = true;
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      avg_cellTemperature = cellTemperature[currentIc][temp] / 10;
      print(currentIc, temp, avg_cellTemperature, isTemp, tempfaultStatus[currentIc][temp]);
      cellTemperature[currentIc][temp] = 0;
    }
    Serial.println();
    Serial.println();
  }
  Serial.println("Stack Voltage");
  printStackVoltage();
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

double minmaxCV() {
  minCV = BMS_IC[0].cells.c_codes[0];
  maxCV = minCV;

  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      uint16_t voltage = BMS_IC[currentIc].cells.c_codes[cell];
      minCV = (voltage < minCV) ? voltage : minCV;
      maxCV = (voltage > maxCV) ? voltage : maxCV;
    }
  }
  Serial.print("Maximum Cell Voltage: ");
  Serial.print(maxCV * 0.0001, 4);
  Serial.println(" V");
  Serial.print("Minimum Cell Voltage: ");
  Serial.print(minCV * 0.0001, 4);
  Serial.println(" V");
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");

  return minCV;
}

void minmaxCT() {
  uint8_t currentIc;

  minCT = maxCT = tempCalc(0, 0);
  for (currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      uint16_t temperature = tempCalc(currentIc, temp);
      minCT = (temperature < minCT) ? temperature : minCT;
      maxCT = (temperature > maxCT) ? temperature : maxCT;
    }
  }
  Serial.print("Maximum Cell Temperature: ");
  Serial.print(maxCT);
  Serial.println(" C");
  Serial.print("Minimum Cell Temperature: ");
  Serial.print(minCT);
  Serial.println(" C");
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}

void canSend() {
  uint8_t canMsg[8];
  uint8_t canMsg_index = 0;
  uint8_t canIDindex = 0;
  uint16_t index = 0;
  bool isSent;

  for (uint8_t currentIc = 0; currentIc < TOTAL_IC; currentIc++) {
    for (uint8_t cell = 0; cell < TOTAL_CELL; cell++) {
      allData[index++] = BMS_IC[currentIc].cells.c_codes[cell];
    }

    for (uint8_t temp = 0; temp < TEMPS; temp++) {
      allData[index++] = BMS_IC[currentIc].aux.a_codes[temp];
    }
  }

  double currentReading = analogRead(CSOUTtemp);

  allData[index++] = currentReading;
  allData[index++] = vsHVin;
  allData[index++] = vsBatin;
  allData[index++] = soc;

  for (uint16_t i = 0; i < 184; i++) {
    canMsg[canMsg_index++] = lowByte(allData[i]);
    canMsg[canMsg_index++] = highByte(allData[i]);

    if (canMsg_index == 8) {
      if (canIDindex < 46) {
        if (CAN.sendMsgBuf(can_ids[canIDindex], 0, 8, canMsg) == CAN_OK) {
          isSent = true;
        } else {
          isSent = false;
        }
        canIDindex++;
      }
      canMsg_index = 0;
    }
  }
  canSend_status = isSent;
}

float readCurrent() {
  int currentValue = analogRead(CSOUTtemp);
  float voltage = currentValue * analogResolution;       // Converts to Voltage
  float current = (voltage - vRef) / sensorSensitivity;  // Calculates Current
  // Serial.print("Voltage: ");
  // Serial.println(voltage);
  return current;
}

void SoC() {
  float totalCoulombs;
  current_time = millis();  // Get the current time

  // Check if the time interval has elapsed
  if (current_time - start_timer >= period) {
    // Measure the current
    float current = readCurrent();

    // Calculate charge in Coulombs (Current (A) * Time (s))
    float deltaTime = period / 1000.0;        // Convert milliseconds to seconds
    float chargeDelta = current * deltaTime;  // Charge change in Coulombs

    // Update total charge
    totalCoulombs += chargeDelta;

    // Calculate State of Charge (SoC)
    float totalAh = totalCoulombs / 3600.0;         // Convert Coulombs to Ah
    soc = 100.0 * (1.0 - (totalAh / maxCapacity));  // Remaining charge as percentage
    soc = constrain(soc, 0.0, 100.0);               // Constrain SoC to valid range

    Serial.print("State of Charge (SoC): ");
    Serial.print(int(soc));
    Serial.println(" %");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
    // Update start time
    start_timer = current_time;
  }
}

void csFault_check() {
  currentSensor = readCurrent();
  Serial.print("Current sensor value:");
  Serial.print(currentSensor);
  Serial.print(" Amps.");
  Serial.println();
  if (currentSensor > csFault_value || currentSensor < -400) {
    digitalWrite(AMS_FAULT_SDC, LOW);
    digitalWrite(AMS_FAULT_PIN, HIGH);
    digitalWrite(progLED3, HIGH);
    Serial.println("Current sensor fault");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
  } else {
    digitalWrite(AMS_FAULT_SDC, HIGH);
    digitalWrite(AMS_FAULT_PIN, LOW);
    digitalWrite(progLED3, LOW);
  }
}

void charging() {
  bool chargerStatus;
  uint32_t buf2[8];
  buf2[2] << 8;
  CAN.sendMsgBuf(0x1806E5F4, 1, 8, BMSA);
  uint16_t chargerCANrx = CAN.getCanId();
  if (chargerCANrx == 0x18FF50E5) {
    chargerCAN_status = true;

  } else {
    chargerCAN_status = false;
  }

  chargerStatus = chargerCAN_status;
  // float currsensor = readCurrent();
  // Serial.print("Current Value: ");
  // Serial.println(currsensor);
  // float chargercurr = ((buf2[2] + (buf2[3] + 0.2)) / 10) - 0.46;
  // Serial.print("Current Value: ");
  // Serial.println(chargercurr); // Prints charger current.
}

// void charging() {
//   uint32_t len2;
//   uint32_t buf2[8];
//   byte abcd[32];
//   float x_cur;
//   Serial.println("***********************Sending CAN*****************************");
//   CAN.sendMsgBuf(0x1806E5F4, 1, 8, BMSA);  // REPLACE THE ACTUAL REQUIRED CANMSG HERE ACCORDING TO SETUP VARIABLES ABOVE
//   //CAN.sendMsgBuf(0x1806E5F4, 1, 8, canMsg7);  // REPLACE THE ACTUAL REQUIRED CANMSG HERE ACCORDING TO SETUP VARIABLES ABOVE
//   uint16_t canId2;
//   canId2 = CAN.getCanId(); //for some reason
//   Serial.println("Charger CAN ID::: ");
//   Serial.print(canId2;
//   Serial.println();
//   //CAN.readMsgBuf(&len2, buf2);

//   Serial.println("len2=====");
//   Serial.print(len2);
//   Serial.println("buf2=====");
//   Serial.print(buf2[0]);
//   Serial.println();
//   if (canId2 == 0x18FF50E5) {
//     Serial.println("-----Recieving Correct ID from charger-----");
//     for (int i = 0; i < len2; i++) {
//       Serial.print("Recieving MSGs.....");
//       abcd[i] = buf2[i];
//     }
//   }
//   //Serial.print("CAN ID: ");
//   //Serial.println(canId2,HEX);
//   //Serial.print("data len = ");
//   //Serial.println(len2);
//   //Serial.print(buf2[0]);
//   //for(int i = 0; i<len2; i++){
//   //Serial.print(buf2[i]);
//   //Serial.print("\t");
//   //}
//   //Serial.println();
//   //for(int i=0;i<16;i++){Serial.print(abcd[i]); Serial.print(" ");
//   //}
//   //Serial.println("****************************************");
//   //digitalWrite(28,HIGH);
//   //Serial.print("-");
//   //delay(1000);
//   // if (count > 10) {
//   //   //Serial.println(buf2[3]+ 0.2);
//   //   count++;
//   // } else {
//   //   //Serial.println(buf2[3]+ 0.5);
//   // }
//   buf2[2] << 8;
//   x_cur = ((buf2[2] + (buf2[3] + 0.2)) / 10) - 0.46;
//   Serial.print("Current Value: ");
//   Serial.println(x_cur);
// }

void lvData() {

  pcrDone = digitalRead(PCRDONE);
  airP = digitalRead(AIRP);
  airN = digitalRead(AIRN);
  pcr5v = digitalRead(PCR5V);
  vsBat = digitalRead(VSBAT);
  tCheck = digitalRead(TCHECK);
  vsBatin = analogRead(VS_BAT_IN);
  vsHVin = analogRead(VS_HV_IN);
  amsFaultout = digitalRead(AMS_FAULT_SDC);
  k1 = digitalRead(K1);
  k2 = digitalRead(K2);
  ssFinal = digitalRead(SSFINAL);
  grnIn = digitalRead(GRNIN);
  grnOut = digitalRead(GRNOUT);
  auxP = digitalRead(AUXP);
  auxN = digitalRead(AUXN);
  pcrAuxin = digitalRead(PCRAUXIN);
  vCheck = digitalRead(VCHECK);

  double vsHVop = vsHVin * analogResolution;
  vsHVvoltage = (vsHVop - 2.529) / 0.001326;

  double vsBATop = vsBatin * analogResolution;
  vsBATvoltage = (vsBATop - 2.529) / 0.001326;

  if (vsBatin >= 0.95 * vsHVin) {
    digitalWrite(VCHECK, HIGH);
  }

  Serial.print("AIR +ve: ");
  Serial.print(airP);
  Serial.print(" || AUX +ve: ");
  Serial.print(auxP);
  Serial.print(" || AIR -ve: ");
  Serial.print(airN);
  Serial.print(" || AUX -ve: ");
  Serial.print(auxN);
  Serial.print(" || PCR: ");
  Serial.print(pcr5v);
  Serial.print(" || PCR AUX: ");
  Serial.print(pcrAuxin);
  Serial.print(" || VSBAT: ");
  Serial.print(vsBat);
  Serial.println();

  Serial.print("ssfinal: ");
  Serial.print(ssFinal);
  Serial.print(" || grnin: ");
  Serial.print(grnIn);
  Serial.print(" || grnout: ");
  Serial.print(grnOut);
  Serial.print(" || VSBatIn: ");
  Serial.print(vsBATvoltage);
  Serial.print(" || VSHVIn: ");
  Serial.print(vsHVvoltage);
  // Serial.print(" || vshvin difference: ");
  // Serial.print(abs(vsHVin - vsBatin));
  Serial.println();

  Serial.print("PCRDONE: ");
  Serial.print(pcrDone);
  Serial.print(" || Vcheck: ");
  Serial.print(vCheck);
  Serial.print(" || TCheck: ");
  Serial.print(tCheck);
  Serial.print(" || K2: ");
  Serial.print(k2);
  Serial.print(" || K1: ");
  Serial.print(k1);
  Serial.println();
  Serial.println("----------------------------------------------------------------------------------------------------------------------------------------------------------");
}