/************************************************************************************* 
**Author::: ITTALO PEZZOTTI ESCOBAR **************************************************
**Date::::: 26/04/2023- 26/05/2025****************************************************
**Project:: CALCULATION AND OBTAINING OF PARAMETERS TO GENERATE THERMAL CYCLES USING** 
**:::::::::PID CONTROL IN A NON-PRESSURIZED CLOSED SYSTEM FOR MATERIALS TEST**********
**Prototypes:::: CACOTE AND COPAGETEC*************************************************
**NAME PROGRAM::1_Copagetec_Temperature_Chamber_PID_V015.INO**************************
**************************************************************************************/
/* ==================================================
  ::::::: ANALOG INPUTS ::::::::::::::::::::::::
  :::::::ARDUINO MEGA PIN-OUT:::::::::::::::::::
  ==================================================
  ==================================================
  :::::::  OUTPUT ACTUATORS :::::::::::::::::::::
  :::::::ARDUINO MEGA PIN-OUT:::::::::::::::::::
  ==================================================
  :::::::::::: PELTIERS  :::::::::::::::::::::::
  PIN_PELTIER B1        PIN_51  12 VDC CON_NO
  PIN_PELTIER B2        PIN_49  12 VDC CON_NO
  PIN_PELTIER B3        PIN_50  12 VDC CON_NO
  PIN_PELTIER B4        PIN_48  12 VDC CON_NO
  :::::::::::: HEATERS  :::::::::::::::::::::::
  ==================================================
  PIN_HEATER HE1        PIN_53  12 VDC CON_OK
  PIN_HEATER HE2        PIN_52  12 VDC CON_OK
  ::::: SSR RELE OUTPUTHEATERS :::::::::::::::::::
  ==================================================
  PIN_HEATER_RESISTENCE  PIN_36  RELE 12  VDC  CON_OK
  PIN_RESISTENCE_1300    PIN_5   RELE 220 VAC  CON_OK
  ==================================================
  ::::::: ACTIVATION FANS ::::::::::::::
  ==================================================
  :::::::: CONTROL SIGNAL FANS ::::::::::::::
  Pin_Fan_Con_Heater     PIN_46 CON_OK
  Pin_Fan_Con_Recircu    PIN_44 CON_OK
  Pin_Fan_Con_AUX_1      PIN_42 CON_OK
  Pin_Fan_Con_AUX_2      PIN_40 CON_OK
  Pin_Fan_Con_Peltier_B1 PIN_38 CON_OK
  Pin_Fan_Con_Peltier_B2 PIN_36 CON_K
  Pin_Fan_Con_Peltier_B3 PIN_34 CON_OK
  Pin_Fan_Con_Peltier_B4 PIN_32 CON_OK
  :::::::::::: SENSOR RPM FANS ::::::::::::::
  Pin_Fan_RPM_Heater     PIN_47 CON_OK
  Pin_Fan_RPM_Recircu    PIN_45 CON_OK
  Pin_Fan_RPM_AUX_1      PIN_43 CON_OK
  Pin_Fan_RPM_AUX_2      PIN_41 CON_OK
  Pin_Fan_RPM_Peltier_B1 PIN_39 CON_OK
  Pin_Fan_RPM_Peltier_B2 PIN_37 CON_OK
  Pin_Fan_RPM_Peltier_B3 PIN_35 CON_OK
  Pin_Fan_RPM_Peltier_B4 PIN_33 CON_OK
  ||||||||||||||||||||||||||||||||||||||||||||||||||
  =======================================
  :::::::::: OTHER DEVICES   :::::::::::
  ::::::: ACTIVATION ELECTROVALVES :::::
  ::::::: TIP 121-TIP 122 ::::::::::::::
  =====================================
  RELAY_Valve_Output_Air PIN A10
  WATER_PUMP_ASPIRATION  PIN_4 D4
  WATER_PUMP_FLOW_INPUT  PIN_5 D5
  ||||||||||||||||||||||||||||||||||||||
  =====================================
  :::::: SENSOR TEMPERATURA  ::::::::::
  ::::: ANALOGS INPUTS MCP9700 ::::::::
  ====================================
  A14:  TA MCP9700
  A13: TB MCP9700
  A12: TC MCP9700
  A11: TD MCP9700
  A10: TE  MCP9700
  ||||||||||||||||||||||||||||||||||||||
  =====================================
  :::::: SENSOR TEMPERATURA  ::::::::::
  ::::  TERMOCUPLA 2 - DS18B20 ::::::::::
  =====================================
  SENSOR TEMP DS18B20 VAPOR TVAP
  D13: TERMOCUPLA SCK
  D12: TERMOCUPLA SD
  D10: TERMOCUPLA CS
  SENSOR TEMP: DS18B20 SECADOR TSEC
  D13: TERMOCUPLA SCK
  D12: TERMOCUPLA SD
  D10: TERMOCUPLA CS
  ||||||||||||||||||||||||||||||||||||||
  A14 PIN SENSOR CURRENT
  ||||||||||||||||||||||||||||||||||||||
  =====================================
  ::::::::: SENSOR HUMIDITY  ::::::::::
  :::::::::::  DHT22   :::::::::::::::
  =====================================
  D8:  DTH22 HUMIDITY SENSOR INTERNAL
  D9:  DTH22 HUMIDITY SENSOR EXTERNAL
  ||||||||||||||||||||||||||||||||||||||
  =====================================
  ::::::::::I2C BUS CONTROL DEVICES::::
  =====================================
  ::::::::: REAL TIME RELOJ  ::::::::::
  :::::::::::  DS1307RTC     ::::::::::
  PIN 20 SDA
  PIN 21 SCL
  ||||||||||||||||||||||||||||||||||||||
  =====================================
  :::::: OTHERS DEVICES  ::::::::::::::
  ::::: ANALOGS INPUTS MCP9700 ::::::::
  ====================================
  ||||||||||||||||||||||||||||||||||||||
  ==============================================
LED indication
1) Fan on (red)
2) Internal resistor on (yellow)
3) Water vapor on (yellow)
4) Peltier on (yellow)
5) Recirculation fan (red)
  ==============================================*/
#include <Arduino.h>
#include <SerialCommUtil.h>
#include <SoftwareSerial.h>
/***********************************************************************
**** CLAMP CURRENT SENSOR
**** SCT013 30A/V FINE TO 100 A SENSOR
*************************************************************************/
#include "EmonLib.h"  /*||CURRENT CLAMP LIBRARY
  /*****************************************************************
 **** VAPOR CONTAINER PROBE TEMPERATURE SENSOR
**** MAX6675 SENSOR USED WITH SPI PORT
 **********************************************************************/
#include <Wire.h>     /*||REQUIRED FOR USE OF THE DALLAS DS18B20 TEMPERATURE SENSOR*/
#include "max6675.h"  /*||REQUIRED FOR USE OF CHIP WITH THERMOCOUPLE*/
/****************************************************************************************
*****************************************************************************************
  PROGRAM:   |1_Copagetec_Temperature_Chamber_PID_V71_26052025.ino
  SUBPROGRAM:|Control_Menu_serial.ino
  SUBPROGRAM:|Menu_Serial.ino || Menu_Serial_Temp
  FOLDER:    |D:\1_0_1_Prototipos_CNR_STIIMA\2_CACOTE\32_Firmware_Cacote_Copagetec_2025
*****************************************************************************************/
#include <Keypad_I2C.h> /*||REQUIRED FOR USE OF KEYBOARD I2C*/
#include <SPI.h>    /*||REQUIRED FOR USE OF DRIVERS AND SENSORS*/
#include <Keypad.h> /*||REQUIRED FOR USE OF KEYBOARD*/
#include <LiquidCrystal.h>/*||REQUIRED FOR USE OF DISPLAY*/
/******************************************************************
TO INCLUDE TIME AND CLOCK FUNCTIONS: <DS1307RTC.h> <Wire.h>
THE TIME AND WIRE LIBRARIES MUST BE ADDED Time.h TimeLib.h
 ******************************************************************/
#include <Time.h>   /*||REQUIRED FOR USE OF TIME RTC*/
#include <TimeLib.h>/*||REQUIRED FOR USE OF TIME RTC*/
#include <DS1307RTC.h>/*||REQUIRED FOR USE OF TIME RTC*/
#include <RTClib.h>   /*||REQUIRED FOR USE OF TIME RTC*/
#include <DHT.h>    /*||REQUIRED FOR USE OF humidity sensor*/
/******************************************************************
CONTROL ALGORITHMS FOR TEMPERATURE, HUMIDITY, AND AIR PRESSURE
**** LIBRARIES REQUIRED FOR PID CONTROL*************************
 ******************************************************************/
#include <PID_v1.h>           /*|| PID CONTROL ALGORITHM*/
#include <OneWire.h>          /*|| SERIAL COMMUNICATION*/
#include <DallasTemperature.h>/*|| TEMPERATURE SENSOR BY PROBE*/
#define pinData A5           /*||DATA PIN AND CONTROL HANDLING*/
#define SSR 5               /** SSR ACTIVATION RESISTOR 1300*/
#define Pin_Heat_R1300  5 /*||SSR ACTIVATION CEMENT RESISTOR Relay Solid State 220VAC **/
#define led 22               /* #define LED_01 22 *****/
#define tiempoCiclo 1000
/***************************************************************************
Ioexpander drives the 23016 chip with the mux control outputs
   //I2C device found at address 0x21  !MCP23016
   //I2C device found at address 0x3C  ! Keyboard
   //I2C device found at address 0x50  !RTC1
   //I2C device found at address 0x68  !RTC2
 ****************************************************************************/
#include <IOexpander.h>
/***************************************************************************
   CONFIG PIN RELATIVE CLOCK AND FRECUENCY
   IO EXPANDER  ADDRESS
***************************************************************************/
#define _100_KHZ 100000     /*|| IOEXPANDER CHIP 23016*/
#define _10_KHZ 10000       /*|| IOEXPANDER CHIP 23016*/
#define RTC_ADDRESS 0x68    /*|| IOEXPANDER CHIP 23016*/
#define SECONDS_ADDRESS 0x00/*|| IOEXPANDER CHIP 23016*/
IOexpander e(0x21);         /*|| IOEXPANDER CHIP 23016*/
/***************************************************************************
CONFIG PIN RELATIVE HUMIDITY AND HEAT CALCS relative humidity 
***************************************************************************/
#define DHTPIN_H1 6    /*||D8:  DTH22 HUMIDITY SENSOR INTERNAL *************/
#define DHTPIN_H2 7    /*||D9:  DTH22 HUMIDITY SENSOR EXTERNAL *************/
#define DHTTYPE DHT11  /*|| Dependiendo del tipo de sensor*****************/
/******** START DHT11 SENSOR PARAMETERS ***************************/
DHT dht_H1(DHTPIN_H1, DHTTYPE); /*|| DHT dht(DHTPIN, DHTTYPE);*************/
DHT dht_H2(DHTPIN_H2, DHTTYPE); /*|| DHT dht(DHTPIN, DHTTYPE);*************/
/******************************************************************************/
#define LED 13    /*|| Pin 13 is connected to the LED*/    
#define F1 190    /*|| KEYBOARD FUNCTION F1*/
#define F2 222    /*|| KEYBOARD FUNCTION F2*/
#define Prog 238  /*|| KEYBOARD FUNCTION PROG*/
#define Esc 246   /*|| KEYBOARD FUNCTION ESC*/
#define Ent 250   /*|| KEYBOARD FUNCTION ENTER*/
#define Menu 252  /*|| KEYBOARD FUNCTION MENU*/
#define TempPin 1 /*|| PIN FUNCTION TEMP TEST*/
/******************************************************************
  ::::::::::::::::::: DEFINE COPAGEITEC :::::::::::::::::::::::::::
******************************************************************/
///:::::::: PELTIER PIN A CONECTOR CARD PCB_SSEC DEFINITION ::::::
#define Pin_Peltier_PE1 46  /*||BANK_B1_Pe1 || PCB_SSEC_17::SL1_PIN_1 *********************/
#define Pin_Peltier_PE2 44  /*||BANK_B2_Pe2 || PCB_SSEC_17::SL1_PIN_6  *********************/
#define Pin_Peltier_PE3 40  /*||BANK_B3_Pe3 || PCB_SSEC_17::SL1_PIN_5  *********************/
#define Pin_Peltier_PE4 42  /*||BANK_B4_Pe4 || PCB_SSEC_17::SL1_PIN_2  *********************/
/**:::::: RESISTENCE RELE PIN A CONECTOR CARD PCB_SSEC DEFINITION::*************************/
#define Pin_Heater 36     /*||HEATER ppal || PCB_SSEC_::PIN_53  12 VDC****************/
/*SSR SOLID STATE RELAY FOR PLATE TYPE ACTUATORS +WATER RESISTANCE************************/
#define Pin_Heat_R1300  5 /*||*SSR SOLID STATE RELAY FOR PLATE TYPE ACTUATORS 220VAC ******/
/*******************************************************************
  ::::::::::::::::::: FAN CONTROL SYSTEM :::::::::::::::::::::::::::::
*************************************************************************/
/* THE CONTROL SYSTEM IS DONE THROUGH THE CIRCUIT WITH THE TIP121********
WITH THE JP9 CONNECTOR, THROUGH PIN D10, HOWEVER THERE ARE 2 PINS********
SIGNALS D11 AND D12 CONNECTED, CONTROL AND RPM OF THE FAN E1, SSEC CARD**
*************************************************************************/
#define Pin_Fan_Heater 37     /*|| CONTROL FAN SIGNAL D4|D11 PIN:40**/
#define Pin_Fan_Heater_RPM 48 /*|| CONTROL FAN SIGNAL D4|D12 PIN:42**/
/*************************************************************************/
/* THE CONTROL SYSTEM IS DONE THROUGH THE CIRCUIT WITH THE TIP122**********
**WITH THE JP9 CONNECTOR, THROUGH PIN D4, HOWEVER THERE ARE 2 PINS*********
**CONNECTED AS SIGNALS D5 AND D6 CONTROL AND RPM OF THE FAN C2 SSEC CARD***
**************************************************************************/
#define Pin_Fan_Recircu 35      /*|| CONTROL FAN SIGNAL D4|D5 PIN:47**/
#define Pin_Fan_Recircu_RPM 45 /*|| CONTROL FAN SIGNAL D4|D6 PIN:45**/
/********************************************************************/
/* THE CONTROL SYSTEM IS DONE THROUGH THE CIRCUIT WITH THE TIP122**********
**WITH THE JP8 CONNECTOR, THROUGH PIN D10, HOWEVER THERE ARE 2 PINS********
**CONNECTED AS SIGNALS D2 AND D3 CONTROL AND RPM OF THE FAN C1 SSEC CARD***
***************************************************************************/
#define Pin_Fan_Peltier_B1 43     /*|| CONTROL FAN SIGNAL D10|D2 PIN:44**/
#define Pin_Fan_Peltier_B2 41     /*|| CONTROL FAN SIGNAL D09|D2 PIN:41**/
#define Pin_Fan_Peltier_B3 39     /*|| CONTROL FAN SIGNAL D08|D2 PIN:39**/
/********************************************************************/
/* THE CONTROL SYSTEM IS DONE THROUGH THE CIRCUIT WITH THE TIP122*****************
**WITH THE JP9 CONNECTOR, THROUGH PIN D9, HOWEVER THERE ARE 2 PINS****************
**CONNECTED AS SIGNALS D8 AND D9 CONTROL AND RPM OF THE E2 FAN ON THE SSEC CARD***
**********************************************************************************/
#define Pin_Fan_Propaga 33      /*|| CONTROL FAN SIGNAL D9|D8 PIN:41**/
/********************************************************************/
/* THE CONTROL SYSTEM IS DONE THROUGH THE CIRCUIT WITH THE TIP122******************
**WITH THE JP10 CONNECTOR, THROUGH PIN D13, HOWEVER THERE ARE 2 PINS***************
**CONNECTED AS SIGNALS D14 AND D15 CONTROL AND RPM OF THE FAN E2 SSEC CARD*********
***********************************************************************************/
#define Pin_Fan_Other 36      /*|| CONTROL FAN SIGNAL D13|D14 PIN:36**/
#define Pin_Fan_Other_RPM 38  /*|| CONTROL FAN SIGNAL D13|D15 PIN:38**/
/********************************************************************/
/****************************************************************************/
/*::::::: PUMP DEFINITION ::::::::::::::::::::::*****************************/
/****************************************************************************/
#define Rele7_Valve_Output_Air 48 //NC
#define Pump_Water_Flow 9
/****BANK_COOLER_EX_S1*******/
#define B1_FAN_EX_S1 45
#define B1_FAN_EX_S2 31
/***:::::::: TERMOCUPLE I AND II DEFINITION ::::::::::::::::*/
int thermoCS = 10;
int thermoCLK = 13;
int thermoDO1 = 30;
int thermoDO2 = 32;
int thermoDO3 = 34;

float Temp_Heat_1300 = 0; /***thermocouple res 130 watt***********/
float Temp_Sec_Air = 0; /**thermocouple sec air heat 2000 watt****/

MAX6675 thermocouple1(thermoCLK, thermoCS, thermoDO1);// first thermocouple
MAX6675 thermocouple2(thermoCLK, thermoCS, thermoDO2);// 2nd thermocouple
MAX6675 thermocouple3(thermoCLK, thermoCS, thermoDO3);// 3nd thermocouple

/***:::::::: SENSOR TEMP DEFINITION FOR DS18B20 ::::::::::::::::***/
int Array_Sensors_DS18B20 = 8;
OneWire oneWireBus(Array_Sensors_DS18B20);
DallasTemperature sensors(&oneWireBus);

DeviceAddress TTUB_Thermometer = { 0x28, 0xE5, 0x07, 0xA8, 0x00, 0x00, 0x00, 0x38 };
DeviceAddress T_IAH_Thermometer = { 0x28, 0x97, 0xFD, 0xA8, 0x00, 0x00, 0x00, 0xD5 };
DeviceAddress TF_Thermometer = { 0x28, 0x97, 0xFD, 0xA8, 0x00, 0x00, 0x00, 0x27 };

/***reset arduino mega**/
#define COMMAND_RESET "reset"

/***Bluetooth 1 (Tablet) software serial pins**/
#define MEGATX_BT1RX_PIN 47
#define MEGARX_BT1TX_PIN A8

/**Bluetooth 2 (Smartphone) software serial pins**/
#define MEGATX_BT2RX_PIN 49
#define MEGARX_BT2TX_PIN A15

/***GPS software serial pins**/
#define MEGATX_GPSRX_PIN 10
#define MEGARX_GPSTX_PIN 11
/******USE_DHT22*********/
#define DHT_PIN 4
/*** Buzzer sound system***/
#define BUZZER_PIN 38
/**LED system indicator**/
#define LED_01 22
#define LED_02 24
#define LED_03 26
#define LED_04 28
#define LED_05 30
#define LED_06 23
#define LED_07 25 /***PID Copagetec_PID_Ra**/  
#define LED_08 27 /***PID Copagetec_PID_Rb**/     
#define LED_09 29 /***PID Copagetec_PID_RC**/  
/****************************************************************************/
/*::::::: FREQUENCE DEFINITION :::::::::::::::::*****************************/
/****************************************************************************/
int pinzumbador = 22;    // pin del zumbador
int frecuencia_SJ = 500;  // job start frequency (Star Job)
int frecuencia_EJ = 5320; // frequency to complete a job(End Job)
int frecuencia_TD = 80;   // frequency for testing devices (Test Device)
int frecuencia_V = 40;    // frequency to turn on the 1300 SSR resistor
int frecuencia_P = 700;   // frequency to turn on the Peltier cooling system
/*:::::::::::::::: TEMPERATURE DEFINITION ::::::::::::::::::::::**********/
float TA, TB, TC, TC1, TC2, TC3, TD, TE, TI, TIC, TCL, TF, T_IAH, TTUB=0;
float Ta, Tb, Tc, Tc1, Tc2, Tc3, Td, Te, Ti, Tic, Tcl, Tf, Ttub = 0;
/*::::::::::::::::: TENSION ANALOG INPUT::::::::::::::::::::::***********/
int TensionAnalog_TA, TensionAnalog_TB, TensionAnalog_TC, TensionAnalog_TC1, TensionAnalog_TC2, TensionAnalog_TC3, TensionAnalog_TD =0;
int TensionAnalog_TE, TensionAnalog_TI, TensionAnalog_TIC, TensionAnalog_TF, TensionAnalog_Ttub = 0;
int TensionAnalog_TIAH,TensionAnalog_TCL  = 0;
/******************************************************************
 *Clamp Meter Current Sensor  SENSOR SCT013 30A/V FINO A 100 A*****
 ******************************************************************/
const int sensorpin_Find = A4;
/**********************************************
******FUNCTION ARROW DISPLAY KEYBOARD**********
***** ARROW CHARACTER  ***********************/
byte flecha[8] = {
  0b00000,
  0b00100,
  0b00110,
  0b11111,
  0b11111,
  0b00110,
  0b00100,
  0b00000
};
/***********************************************************
****** VARIABILES FOR THE MUX CONTROL CHPS******************
************ variabile global used :::**********************
***********************************************************/
long du, di;
float Tempc, T1, T2, T3, T4, T5, T6, T7, T8, D1, D2;
float AnalogGlobal;
const int KeypapAddress = 0x38; /*Adress I2C for the keyboard*/
byte pulsacion = 0;
int z = 0; /********:: Cunters::z = 0; *************/
int x = 0; /********:: Cunters::x = 0; *************/
int d = 0; /********:: Cunters::d = 0; *************/
int c = 0; /********:: Cunters::c = 0; *************/
int input;
LiquidCrystal lcd(10);  /* ACTIV increment A 4% Y 8%*****/
/*************************************************************************
**************  M41T11  **************************************************
*************************************************************************/
byte dummy;         /* ACTIV increment A 4% Y 8%*****/
byte second_itt;    /* ACTIV increment A 4% Y 8%*****/
byte minute_itt;    /* ACTIV increment A 4% Y 8%*****/
byte hour_itt;      /* ACTIV increment A 4% Y 8%%*****/
byte dayOfWeek_itt; /* ACTIV increment A 4% Y 8%*****/
byte dayOfMonth_itt;/* ACTIV increment A 4% Y 8%*****/
byte day_itt;      /* ACTIV increment A 4% Y 8%*****/
byte month_itt;     /* ACTIV increment A 4% Y 8%*****/
byte year_itt;      /* ACTIV increment A 4% Y 8%*****/
/**************************************************************************
 **************   CONTROL MENU SERIAL
 **************   LED ACTIVITY 13
 **************   char rxChar= 0; COMMAND RECEIVED
 ***************************************************************************/
char rxChar = 0;  // RXcHAR holds the received command.//*CONTROL MENU SERIAL
/***************************************************************************/
/*** TERMS TO BE USED FOR RA APPLICATION:: 
 *  Last Temperature response==>> LTR_Ra, LTR_Rb, LTR_Rc
 *  last Controller PID Calculation==> LCPC
 *  cycle time limits=>Time_CTL
 */
unsigned long LTR_Ra, LTR_Rb, LTR_Rc   = 0;
unsigned long LCPC = 0;
#define Time_CTL 1000
/***:::::::: SENSOR TEMP DEFINITION FOR MPC9700 ::::::::::::::::***/
#define Ta_Ra A14   
#define Tb_Rb A13  
#define Tc1_Rc A7 
#define Tc2_Rc A6 
#define Tc3_Rc A5  
#define Tcl_Rc A11
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//::::::::  PID CONTROL DEFINITIONS   ::::::::::::::::::::::::::::::::::::::::::://
double Setpoint, Input, Output;        
/*******  𝐺(𝑠)=4,942,78𝑆+1,72 Eq- 40**************/
double Kp_Ra=0.4590, Ki_Ra=0.4805, Kd_Ra=-0.1539;
/***Specifies initial RA parameters **********/
PID Copagetec_PID_RA(&Input, &Output, &Setpoint, Kp_Ra, Ki_Ra, Kd_Ra, DIRECT);
/*******   𝐺(𝑠)=4,944/(4,826𝑆+0,583)**************/
double Kp_Rb=0.1548, Ki_Rb=0.0312, Kd_Rb=-0.269;
/***Specifies initial RA parameters **********/
PID Copagetec_PID_RB(&Input, &Output, &Setpoint, Kp_Rb, Ki_Rb, Kd_Rb, DIRECT);
/*******  𝐺(𝑠)= 36/(12,42𝑠+7,27) Eq- 66 **************/
double Kp_Rc=0.2662, Ki_Rc=0.2636, Kd_Rc=-0.094;
/***Specifies initial RC parameters **********/
PID Copagetec_PID_RC(&Input, &Output, &Setpoint, Kp_Rc, Ki_Rc, Kd_Rc, DIRECT);
/*** CALC RA RESISTANCE CONTROL CURVE********************************/
double Kp = 0.459, Ki = 0.485, Kd = -0.1539;        
/*** CALCULATION FOR THE Humidity CONTROL CURVE    ********************************/
double Kp1 = 0.459, Ki1 = 0.485, Kd1 = -0.1539;       
/*** CALCULATION FOR THE PELTIER CONTROL CURVE Temp_cold******************************/
double Kp2 = 0.459, Ki2 = 0.485, Kd2 = -0.1539;      
/*** CALCULATION FOR THE RESISTANCE CONTROL CURVE Temp Hot****************************/
double Kp3 = 0.459, Ki3 = 0.485, Kd3 = -0.1539;      
/*** ELECTRONIC PID CONTROLLER********************************/
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_Humidity(&Input, &Output, &Setpoint, Kp1, Ki1, Kd1, DIRECT);
PID myPID_Temp_frio(&Input, &Output, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
PID myPID_Temp_Hot(&Input, &Output, &Setpoint, Kp3, Ki3, Kd3, DIRECT);
PID myPID_gas(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*** VARIABLES ASSIGNED TO THE CONTROLLER********************************/
float prevTemperature = -9999.0;
float temperatura = 0;
unsigned long respuestaUltimaTemperatura = 0;
unsigned long lastPIDCalculation = 0;
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
/************************************************************************************/
void setup() 
    {/****SETUP INIT**/
           Parameters_Comunications_setup();
           delay(500);
           Parameters_Controller_setup();
           //Parameters_RTC_setup();
           Parameters_SSR_Current_setup();
            /**** INIT SENSOR DHT*******/
            dht_H1.begin();
            dht_H2.begin();
            Serial.flush();       // Clear receive buffer.
          Parameters_Print_Menus_setup();
           //:: KEYBOARD CONFIG
            keyboard_Config_setup();
            //:: DISPLAY CONFIG ::::
            display_Config_setup();
           /*SET EFC WITH OUTPUTS:::::::::::::::::::*/
            actuators_setup();
            Parameters_Leds_setup();
            /*SET DEVICE SIGNALS LOW O HIGH:::::::::::::::::::*/
            actuators_setup_start();
            digitalWrite(LED, LOW); // Sets pin 13 as OUTPUT.
            //outputs_sensors_test();
            // Menu_ppal();
            delay(500);
           /****  INIT control Parameters_Config copagetec***/
             Parameters_Config_Setup_Ra();   
             Parameters_Config_Setup_Rb(); 
             Parameters_Config_Setup_Rc(); 
           
}/****  END SETUP copagetec***/

void Parameters_Comunications_setup()
{
  /**** START IF FOR SERIAL IN THE COM6 CONECTION WIFI MODULE****/
   Serial.begin(9600);  /* Define baud rate for serial communication MODEM WIFI*/
/**** START IF FOR SERIAL 1 IN THE COM5 CONECTION SMARTPHONE LED YELLOW****/
   Serial1.begin(9600);  /* Define baud rate for serial 1 communication Bluetooth_SMARTP*/
/**** START IF FOR SERIAL 2 IN THE COM5 CONECTION TABLET LED YELLOW****/
   Serial2.begin(9600);  /* Define baud rate for serial 2 communication Bluetooth_TABLET*/
  /**** START IF FOR SERIAL 3 IN THE COM6 CONECTION GPRS_SIM800*********************/
   Serial3.begin(9600);  /* Define baud rate for serial 2 communication GPRS_SIM800*/
}


void Parameters_Controller_setup()
{
  Setpoint = 30.0;                 // initialize the variables we're linked to
  myPID.SetOutputLimits(0, tiempoCiclo);
  myPID.SetSampleTime(tiempoCiclo);
  myPID.SetMode(AUTOMATIC); 
}
void Parameters_RTC_setup()
{
            //RTC.begin(); //RTC START
            //RTC.adjust(DateTime(__DATE__, __TIME__)); //TIME DATA
            //:: REAL TIME CLOCK ::
            //  setSyncProvider(RTC.get);   // the function to get the time from the RTC
            //  if (timeStatus() != timeSet)
            //    Serial.println("Unable to sync with the RTC");
            //  else
            //    Serial.println("RTC has set the system time");
            //  //
            //  setSyncProvider(RTC.get);// 
            //

}
void Parameters_SSR_Current_setup()
{
           //  pinMode(SSR, OUTPUT);
            //  digitalWrite(SSR, LOW);
            //   pinMode(led, OUTPUT);
            //  digitalWrite(led, LOW);
            //  sensors.begin();                       
            //emon1.current(14, 111.1);             
            //  pinMode(SSR, OUTPUT);
            digitalWrite(SSR, LOW);
            //   pinMode(led, OUTPUT);
            digitalWrite(led, LOW);
            //delay(500);
}


void Parameters_Sensors_setup()
{
 /**** INIT SENSORS DHT*******/
            dht_H1.begin();
            dht_H2.begin();
 /**** INIT SENSORS TEMP DSB1820*******/
            sensors.begin();
  sensors.setResolution(TTUB_Thermometer, 10);
  sensors.setResolution(T_IAH_Thermometer, 10);
  sensors.setResolution(TF_Thermometer, 10);
}

void Parameters_Print_Menus_setup()
{
             //  printSystem();
            //  printHelp();          
            //  Menu_Others_Sensors();
            //RTC.adjust(DateTime(__DATE__, __TIME__)); 
            Menu_ppal();
           //Serial_Menu_Init_Copagetec();
}
/** This code is in the KEYBOARD configuration *********
** I removed it because the code is long and is used for ************
** Here you set the devices to low or high signals::*/
void keyboard_Config_setup()
{
     //:: KEYBOARD CONFIG
    Wire.begin(); // init transmission
    Wire.beginTransmission(KeypapAddress);
    Wire.write(0b11111110); //pin. ref. "0" PUSH one value
    Wire.endTransmission(); // End transmission
    delay(500);
}
/** This code is inside the DISPLAY configuration*************
** I removed it because the code is long and is used for all ***************
** Here you set the devices to low or high signals::::::::::::::::::::*/
void display_Config_setup()
{
    lcd.begin(16, 2);
    lcd.createChar(0, flecha);//Reg. New Char
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("COPAGETEC");
    lcd.setCursor(3, 1); //set the cursor to column 0, line 1
    lcd.print("SYSTEM TEMP");
}

/** This code is inside the setup configuration ***************************
** I removed it because the code is long and is used for all ***************
** Final config control elements as outputs :::::::::::::::::::::::::::::::::::*/
void actuators_setup()
{/** INIT actuators setup***/
  //:::::: ACTUATORS::::::::://
  /*******PELTIER*****************/
  pinMode(Pin_Peltier_PE1, OUTPUT);
  pinMode(Pin_Peltier_PE2, OUTPUT);
  pinMode(Pin_Peltier_PE3, OUTPUT);
  pinMode(Pin_Peltier_PE4, OUTPUT);
  /*******HEATERS*****************/
  pinMode(Pin_Heater, OUTPUT);
  pinMode(Pin_Heat_R1300, OUTPUT);
  //::::::     FAN PIN OUT ::::::::://
  pinMode(Pin_Fan_Heater, OUTPUT);
  pinMode(Pin_Fan_Recircu, OUTPUT);
  pinMode(Pin_Fan_Peltier_B1, OUTPUT);
  pinMode(Pin_Fan_Peltier_B2, OUTPUT);
  pinMode(Pin_Fan_Peltier_B3, OUTPUT);
 /****BANK_COOLER_EX_S1*******/
 pinMode(B1_FAN_EX_S1, OUTPUT);
 pinMode(B1_FAN_EX_S2, OUTPUT);
  pinMode(Pin_Fan_Propaga, OUTPUT);
  pinMode(Pin_Fan_Other, OUTPUT);
/****RELE SYSTEMS*******/
 pinMode(Rele7_Valve_Output_Air, OUTPUT);
 pinMode(Pump_Water_Flow, OUTPUT);
}/** END actuators setup***/

/** This code is inside the setup configuration start outputs*************
** I removed it because the code is long and is used for all ***************
** Here the devices are set to low or high signals::::::::::::::::::::*/
void actuators_setup_start()
{/** INIT actuators_setup_start***/
  //:::::: ACTUATORS STATUS STAR::::::::://
  /*******PELTIER*****************/
  digitalWrite(Pin_Peltier_PE1, LOW);
  digitalWrite(Pin_Peltier_PE2, LOW);
  digitalWrite(Pin_Peltier_PE3, LOW);
  digitalWrite(Pin_Peltier_PE4, LOW);
  /*******HEATERS*****************/
  digitalWrite(Pin_Heater, LOW);
  digitalWrite(Pin_Heat_R1300, LOW);
  //::::::     FAN PIN OUT ::::::::://
  digitalWrite(Pin_Fan_Heater, LOW);
  digitalWrite(Pin_Fan_Recircu, LOW);
  digitalWrite(Pin_Fan_Peltier_B1, LOW);
  digitalWrite(Pin_Fan_Peltier_B2, LOW);
  digitalWrite(Pin_Fan_Peltier_B3, LOW);
 /****BANK_COOLER_EX_S1*******/
 digitalWrite(B1_FAN_EX_S1, LOW);
 digitalWrite(B1_FAN_EX_S2, LOW);
  digitalWrite(Pin_Fan_Propaga, LOW);
  digitalWrite(Pin_Fan_Other, LOW);
/****RELE SYSTEMS*******/
 digitalWrite(Rele7_Valve_Output_Air, LOW);
 digitalWrite(Pump_Water_Flow, LOW);
}/** END actuators_setup_start***/

void Parameters_Leds_setup()
{
  pinMode(LED, OUTPUT);
  pinMode(LED_01, OUTPUT);
  pinMode(LED_02, OUTPUT);
  pinMode(LED_03, OUTPUT);
  pinMode(LED_04, OUTPUT);
  pinMode(LED_05, OUTPUT);
  pinMode(LED_06, OUTPUT);
  pinMode(LED_07, OUTPUT);
  pinMode(LED_08, OUTPUT);
  pinMode(LED_09, OUTPUT);
}





//::Start loop  parameters:://
void loop()
{ /****INIT SECTION LOOP ******/


/****INIT SECTION the controller LOOP ******/
//   loop_CON_PID_Ra_actuator();
//   loop_CON_PID_Rb_actuator();
//   loop_CON_PID_Rc_actuator();
 /*** activate here if you want to use the controller**/
//  Sistema_control(); 
//  control();
  /*** only for sensor testing**/
  //Rutina_Sens_Temp_DS_loop();

  x = 0;
  //mP1(z);
  getpulsacion();
  //handleSerial();
  //printHelp();

  Serial_Menu_Init_Copagetec(); //(1)
  Serial_EFC_Actuators_Menu();  //(2)
  Serial_temp_menu();//(3)
  Serial_Menu_Others_Sensors(); //(4)
  Serial_Calibration_Menu();//(5)

  
  if (pulsacion == F2) {
    if (z < 2) {
      z = z + 1;
    }
  }
  if (pulsacion == F1) {
    if (z > 0) {
      z = z - 1;
    }
  }
  // submenu calibracion
  if (pulsacion == Menu && z == 0) {
    c = 0;
    x = 0;
    while (c == 0) {
      getpulsacion();
      // MC(x);
      Puntero(3);//Numero de Submenus N-1 =3
      if (pulsacion == Esc) {
        c = 1;
        z = 0;
      }
      if (pulsacion == Menu && x == 0) {
        d = 0;
        x = 0;
        while (d == 0)   {
          getpulsacion();
          // MT(x);
          Puntero(10);
          // ExecuteFunctionT(pulsacion,x);
          if (pulsacion == Esc) {
            d = 1;
            x = 0;
          }
        }//Fin while d
      }//Fin if
      if (pulsacion == Menu && x == 1) {
        d = 0;
        x = 0;
        while (d == 0)   {
          getpulsacion();
          //MN(x);
          Puntero(2);
          //ExecuteFunctionN(pulsacion,x);

          if (pulsacion == Esc) {
            d = 1;
            x = 0;
          }
        }//Fin while d
      }//Fin if
      if (pulsacion == Menu && x == 2) {
        d = 0;
        x = 0;
        while (d == 0)   {
          getpulsacion();
          // MF(x);
          Puntero(2);
          //ExecuteFunctionF(pulsacion,x);

          if (pulsacion == Esc) {
            d = 1;
            x = 0;
          }
        }//Fin while d
      }//Fin if
      if (pulsacion == Menu && x == 3) {
        if (pulsacion == Esc) {
          d = 0;
          x = 0;
          while (d == 0)   {
            getpulsacion();


            if (pulsacion == Esc) {
              d = 1;
              x = 0;
            }
          }//Fin while d
        }
      }//Fin if
    }//Fin while c
  }
}/*****end loop***/


void Rutina_Sens_Temp_DS_loop(void)
{/*****Init Rutina_Sens_Temp_DS_loop****/
  Serial.println(F("Reading temps..."));
  sensors.requestTemperatures();
  Serial.print(F("pipe temperature: "));
  printTemperature(TTUB_Thermometer);
  Serial.print(F("Ambient temperature interaction "));
  printTemperature(T_IAH_Thermometer);
  Serial.print(F("temperature thermal gradients "));
  printTemperature(TF_Thermometer);
  Serial.println(F("*********************"));
  delay(2500);
}/*****end Rutina_Sens_Temp_DS_loop****/

/******* :: COPAGITEC ::********/
void outputs_sensors_test()
{
  // put your main code here, to run repeatedly:
  tone(pinzumbador, frecuencia_SJ);   // inicia el zumbido
  delay(2000);
  tone(pinzumbador, frecuencia_TD);   // inicia el zumbido
  delay(1000);
  digitalWrite(Pin_Heater, HIGH);              // turn the yellow LED on
  delay(400);
  digitalWrite(Pin_Heater, LOW);                  // turn the yellow LED off
  delay(800);
  digitalWrite(Pin_Fan_Heater, HIGH);              // turn the yellow LED on
  delay(400);
  digitalWrite(Pin_Fan_Heater, LOW);                  // turn the yellow LED off
  delay(800);

  digitalWrite(Pin_Peltier_PE1, HIGH);              // turn the yellow LED on
  delay(400);
  digitalWrite(Pin_Peltier_PE1, LOW);                  // turn the yellow LED off
  delay(800);

  digitalWrite(Pin_Fan_Recircu, HIGH);              // turn the yellow LED on
  delay(400);
  digitalWrite(Pin_Fan_Recircu, LOW);                  // turn the yellow LED off
  delay(800);

  tone(pinzumbador, frecuencia_EJ);   // inicia el zumbido
  delay(1000);
  noTone(pinzumbador);               // lo detiene a los dos segundos
  delay(2000);
}

void Flow_Coolant_Liquid()
{
  digitalWrite(Pump_Water_Flow, HIGH);   // poner el Pin en HIGH
  delay(10000);               // esperar 10 segundos
  digitalWrite(Pump_Water_Flow, LOW);    // poner el Pin en LOW
  delay(10000);               // esperar 10 segundos
}

void printTemperature(DeviceAddress deviceAddress)
{
 float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print(F("No find Temp"));
  } else {
    Serial.print(tempC);
  Serial.println(F(" ºC"));
  }
}



void Sistema_control() 
{
 
  //// for thermocouple 1
     Serial.print(F("C1 = "));
     Serial.print(thermocouple1.readCelsius());
  //// for thermocouple 2
  delay(400);
     Serial.print(F("C2 = "));
     Serial.println(thermocouple2.readCelsius());
  delay(400);
  //pid inicio
  if (millis() - respuestaUltimaTemperatura >= tiempoCiclo) {
      temperatura = sensors.getTempCByIndex(0);
      Input = (double)temperatura;
  
      myPID.Compute();
      lastPIDCalculation = millis();
       Serial.print(temperatura);
       Serial.print(F(" , "));
       Serial.println(Output/50);
      sensors.requestTemperatures();
      respuestaUltimaTemperatura = millis();}
  }
  
void control() 
{
  if ((millis() <= (lastPIDCalculation + Output)) || (Output == tiempoCiclo)) {
    // Power on:
    digitalWrite(SSR, HIGH);
    digitalWrite(led, HIGH);
  } else {
    // Power off:
    digitalWrite(SSR, LOW);
    digitalWrite(led, LOW);
  }
}

/******************CONTROL SECTION COPAGETEC************************************/

void Parameters_Config_Setup_Ra()
{/****  INIT SETUP Parameters_Config copagetec***/
  Setpoint = 70.0;                                                 
  Copagetec_PID_RA.SetOutputLimits(0, Time_CTL);                           
  Copagetec_PID_RA.SetSampleTime(Time_CTL);
  Copagetec_PID_RA.SetMode(AUTOMATIC); 
  //::::::     PIN OUT ::::::::://
  pinMode(Pin_Heat_R1300, OUTPUT);
  pinMode(LED_07, OUTPUT);
  /****BANK_COOLER_EX_S1*******/
  pinMode(Pin_Fan_Recircu, OUTPUT);
  pinMode(Pin_Fan_Propaga, OUTPUT);
  /****RELE SYSTEMS*******/
 pinMode(Rele7_Valve_Output_Air, OUTPUT);

 /****STAR INIT OUTPUTS IN LOW**/
   digitalWrite(Pin_Heat_R1300, LOW); 
   digitalWrite(Pin_Fan_Recircu, LOW); 
   digitalWrite(Pin_Fan_Propaga, LOW); 
   digitalWrite(Rele7_Valve_Output_Air, LOW); 
   digitalWrite(LED_07, LOW);
}/****  END SETUP Parameters_Config copagetec***/

/************ 𝐺(𝑠)=𝑍𝑁𝑆+𝑀 Eq- 39 **********************
 ******* using the resistance Ra: 48 𝞨 to 220 Vac**** 
 *******  𝐺(𝑠)=4,942,78𝑆+1,72 Eq- 40**************/
void loop_CON_PID_Ra_actuator()
{/*****INIT loop loop_CON_PID_Ra_actuator***/
  if (millis() - LTR_Ra >= Time_CTL) 
    {
    temperatura = Ta_Ra;
    Input = (double)temperatura;
    Copagetec_PID_RA.Compute();
    LCPC = millis();
    Serial.print(temperatura);
    Serial.print(" , ");
    Serial.println(Output/50);    
     LTR_Ra = millis();
    }
     control_Ra();
}/*****end loop loop_CON_PID_Ra_actuator***/

/************ 𝐺(𝑠)=𝑍𝑁𝑆+𝑀 Eq- 39 **********************
 ******* using the resistance Ra: 48 𝞨 to 220 Vac**** 
 *******  𝐺(𝑠)=4,942,78𝑆+1,72 Eq- 40**************/
void control_Ra() 
{/****  start routine Control Ra Element heat***/

    if ((millis() <= (LCPC + Output)) || (Output == Time_CTL)) 
     {/***star heating elements*/
   /*** start the heating element**/
    digitalWrite(Pin_Heat_R1300, HIGH);
    /**it is necessary that the air flow be constant*/
    digitalWrite(Pin_Fan_Propaga, HIGH);
    digitalWrite(LED_07, HIGH);
     }/***stop heating elements*/
  else 
  {/**start with smoothing of hot air extraction curve**/
    /***everything should be set to low for control***/
    digitalWrite(Pin_Heat_R1300, LOW);
    digitalWrite(LED_07, LOW);
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, HIGH);
   digitalWrite(Pin_Fan_Propaga, HIGH);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, HIGH);
   delay(2500);   
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, LOW);
   digitalWrite(Pin_Fan_Propaga, LOW);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, LOW);
   delay(2500);
  }/**Shutdown stop with smoothing of hot air extraction curve**/
}/****  end routine Control Ra Element heat***/

/************ 𝐺(𝑠)=C/(𝐴𝑠+B) :::  Eq- 47 *****************
 *   using the resistance Rb: 220 𝞨 to 12 Vdc ********** 
 *   𝐺(𝑠)=4,944/(4,826𝑆+0,583) :::: Eq- 48 **************/
void Parameters_Config_Setup_Rb()
{/****  INIT SETUP Parameters_Config copagetec***/
  Setpoint = 50.0;                                                 
  Copagetec_PID_RB.SetOutputLimits(0, Time_CTL);                           
  Copagetec_PID_RB.SetSampleTime(Time_CTL);
  Copagetec_PID_RB.SetMode(AUTOMATIC); 
  //::::::     PIN OUT ::::::::://
  pinMode(Pin_Heater, OUTPUT);
  pinMode(Pin_Fan_Heater, OUTPUT);
  pinMode(LED_08, OUTPUT);
  /****BANK_COOLER_EX_S1*******/
  pinMode(Pin_Fan_Recircu, OUTPUT);
  pinMode(Pin_Fan_Propaga, OUTPUT);
  /****RELE SYSTEMS*******/
 pinMode(Rele7_Valve_Output_Air, OUTPUT);

 /****STAR INIT OUTPUTS IN LOW**/
   digitalWrite(Pin_Heater, LOW); 
   pinMode(Pin_Fan_Heater, LOW);
   digitalWrite(Pin_Fan_Recircu, LOW); 
   digitalWrite(Pin_Fan_Propaga, LOW); 
   digitalWrite(Rele7_Valve_Output_Air, LOW); 
   digitalWrite(LED_08, LOW);
}/****  END SETUP Parameters_Config copagetec***/

/************ 𝐺(𝑠)=C/(𝐴𝑠+B) :::  Eq- 47 *****************
 *   using the resistance Rb: 220 𝞨 to 12 Vdc ********** 
 *   𝐺(𝑠)=4,944/(4,826𝑆+0,583) :::: Eq- 48 **************/
void loop_CON_PID_Rb_actuator()
{/*****INIT loop loop_CON_PID_Rb_actuator***/
  if (millis() - LTR_Rb >= Time_CTL) 
    {
    temperatura = Tb_Rb;
    Input = (double)temperatura;
    Copagetec_PID_RB.Compute();
    LCPC = millis();
    Serial.print(temperatura);
    Serial.print(F(" , "));
    Serial.println(Output/50);    
     LTR_Rb = millis();
    }
     control_Rb();
}/*****end loop loop_CON_PID_Rb_actuator***/

/************ 𝐺(𝑠)=C/(𝐴𝑠+B) :::  Eq- 47 *****************
 *   using the resistance Rb: 220 𝞨 to 12 Vdc ********** 
 *   𝐺(𝑠)=4,944/(4,826𝑆+0,583) :::: Eq- 48 **************/
void control_Rb() 
{/****  start routine Control Rb Element heat***/
     /***everything should be set to low for control***/
    if ((millis() <= (LCPC + Output)) || (Output == Time_CTL)) 
     {/***star heating elements*/
   /*** start the heating element**/
    digitalWrite(Pin_Heater, HIGH);
    digitalWrite(Pin_Fan_Heater, HIGH);
    /**it is necessary that the air flow be constant*/
    digitalWrite(Pin_Fan_Propaga, HIGH);
    digitalWrite(LED_08, HIGH);
     }/***stop heating elements*/
  else 
  {/**start with smoothing of hot air extraction curve**/
    // Power off:
    digitalWrite(Pin_Heater, LOW);
    digitalWrite(LED_08, LOW);
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, HIGH);
   digitalWrite(Pin_Fan_Propaga, HIGH);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, HIGH);
   delay(2500);   
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, LOW);
   digitalWrite(Pin_Fan_Propaga, LOW);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, LOW);
   delay(2500);
  }/**Shutdown stop with smoothing of hot air extraction curve**/
}/****  end routine Control Rb Element heat***/

/************𝐺(𝑠)=F/(𝐷𝑠+E)  :::::  Eq- 65************* 
 *****using the resistance Rc: 1,98 𝞨 to 12 Vdc******* 
 ****** 𝐺(𝑠)= 36/(12,42𝑠+7,27) :::: Eq- 66*************/
void Parameters_Config_Setup_Rc()
{/****  INIT SETUP Parameters_Config copagetec***/
  Setpoint = 50.0;                                                 
  Copagetec_PID_RC.SetOutputLimits(0, Time_CTL);                           
  Copagetec_PID_RC.SetSampleTime(Time_CTL);
  Copagetec_PID_RC.SetMode(AUTOMATIC); 
  //::::::     PIN OUT ::::::::://
  pinMode(Pin_Peltier_PE1, OUTPUT);
  pinMode(Pin_Peltier_PE2, OUTPUT);
  pinMode(Pin_Peltier_PE3, OUTPUT);
  pinMode(Pin_Peltier_PE4, OUTPUT);
  
  pinMode(Pin_Fan_Peltier_B1, OUTPUT);
  pinMode(Pin_Fan_Peltier_B2, OUTPUT);
  pinMode(Pin_Fan_Peltier_B3, OUTPUT);
  pinMode(LED_09, OUTPUT);
  /****BANK_COOLER_EX_S1*******/
  pinMode(Pin_Fan_Recircu, OUTPUT);
  pinMode(Pin_Fan_Propaga, OUTPUT);
  /****RELE SYSTEMS*******/
 pinMode(Rele7_Valve_Output_Air, OUTPUT);

 /****STAR INIT OUTPUTS IN LOW**/
    digitalWrite(Pin_Peltier_PE1, LOW); 
    digitalWrite(Pin_Peltier_PE2, LOW); 
    digitalWrite(Pin_Peltier_PE3, LOW); 
    digitalWrite(Pin_Peltier_PE4, LOW); 
   
    pinMode(Pin_Fan_Peltier_B1, LOW);
    pinMode(Pin_Fan_Peltier_B2, LOW);
    pinMode(Pin_Fan_Peltier_B3, LOW);
   
   digitalWrite(Pin_Fan_Recircu, LOW); 
   digitalWrite(Pin_Fan_Propaga, LOW); 
   digitalWrite(Rele7_Valve_Output_Air, LOW); 
   digitalWrite(LED_09, LOW);
}/****  END SETUP Parameters_Config copagetec***/


/************𝐺(𝑠)=F/(𝐷𝑠+E)  :::::  Eq- 65************* 
 *****using the resistance Rc: 1,98 𝞨 to 12 Vdc******* 
 ****** 𝐺(𝑠)= 36/(12,42𝑠+7,27) :::: Eq- 66*************/
void loop_CON_PID_Rc_actuator()
{/*****INIT loop loop_CON_PID_Rc_actuator***/
  if (millis() - LTR_Rc >= Time_CTL) 
    {
    temperatura = Tcl_Rc;
    Input = (double)temperatura;
    Copagetec_PID_RC.Compute();
    LCPC = millis();
    Serial.print(temperatura);
    Serial.print(F(" , "));
    Serial.println(Output/50);    
     LTR_Rc = millis();
    }
     control_Rc();
}/*****end loop loop_CON_PID_Rc_actuator***/

/************𝐺(𝑠)=F/(𝐷𝑠+E)  :::::  Eq- 65************* 
 *****using the resistance Rc: 1,98 𝞨 to 12 Vdc******* 
 ****** 𝐺(𝑠)= 36/(12,42𝑠+7,27) :::: Eq- 66*************/
void control_Rc() 
{/****  start routine Control Rc Element Could***/

    if ((millis() <= (LCPC + Output)) || (Output == Time_CTL)) 
     {/***star Could elements*/
   /*** start the Could element**/
    digitalWrite(Pin_Peltier_PE1, HIGH); 
    digitalWrite(Pin_Peltier_PE2, HIGH); 
    digitalWrite(Pin_Peltier_PE3, HIGH); 
    digitalWrite(Pin_Peltier_PE4, HIGH); 
    digitalWrite(Pin_Fan_Heater, HIGH);
    
    pinMode(Pin_Fan_Peltier_B1, HIGH);
    pinMode(Pin_Fan_Peltier_B2, HIGH);
    pinMode(Pin_Fan_Peltier_B3, HIGH);
   /**it is necessary that the air flow be constant*/
    digitalWrite(Pin_Fan_Propaga, HIGH);
    digitalWrite(LED_09, HIGH);
     }/***stop heating elements*/
  else 
  {/**start with smoothing of hot air extraction curve**/
    /***everything should be set to low for control***/
     /****STAR INIT OUTPUTS IN LOW**/
    digitalWrite(Pin_Peltier_PE1, LOW); 
    digitalWrite(Pin_Peltier_PE2, LOW); 
    digitalWrite(Pin_Peltier_PE3, LOW); 
    digitalWrite(Pin_Peltier_PE4, LOW); 
    pinMode(Pin_Fan_Peltier_B1, LOW);
    pinMode(Pin_Fan_Peltier_B2, LOW);
    pinMode(Pin_Fan_Peltier_B3, LOW);
    digitalWrite(LED_09, LOW);
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, LOW);
   digitalWrite(Pin_Fan_Propaga, HIGH);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, LOW);
   delay(2500);   
    /***soften the temperature drop**/
      //::::::     FAN PIN OUT ::::::::://
   digitalWrite(Pin_Fan_Recircu, LOW);
   digitalWrite(Pin_Fan_Propaga, LOW);
    /****RELE SYSTEMS*******/
   digitalWrite(Rele7_Valve_Output_Air, LOW);
   delay(2500);
  }/**Shutdown stop with smoothing of hot air extraction curve**/
}/****  end routine Control Rc Element heat***/
