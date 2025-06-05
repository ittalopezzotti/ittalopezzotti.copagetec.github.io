/************************************************************************************* 
**Author::: ITTALO PEZZOTTI ESCOBAR **************************************************
**Date::::: 26/04/2023- 26/05/2025****************************************************
**Project:: CALCULATION AND OBTAINING OF PARAMETERS TO GENERATE THERMAL CYCLES USING** 
**:::::::::PID CONTROL IN A NON-PRESSURIZED CLOSED SYSTEM FOR MATERIALS TEST**********
**Prototypes:::: CACOTE AND COPAGETEC*************************************************
**NAME PROGRAM::1_Copagetec_Temperature_Chamber_PID_V015.INO**************************
**************************************************************************************/

/***************************************************************************************************************
 ******************************SEXTION MENUS PROGRAMM***********************************************************
****************************************************************************************************************/
void Menu_ppal() 
   {
     delay(100);//delay for visual
      Menu_Init_Copagetec();
      delay(100);//delay for visual  
     // Serial_Menu_Init_Copagetec();  
    } 

//void Menu_ppal() 
//   {
//     Menu_Init_Copagetec();
//     delay(100);//delay for visual
//     Menu_EFC_Actuators();
//     delay(100);//delay for visual
//     printHelp();
//     delay(100);//delay for visual
//     Menu_Others_Sensors();
//     delay(100);//delay for visual
//     //Calibration_sensors();
//     Serial_Calibration_Menu();
//    } 
 
/************************************************************** 
********************** Menu_printSystem 1_Copagetec_Temperature_Chamber_PID_V72_26052025====>>>> 
***************************************************************/
void Menu_EFC_Actuators(void)
  {
   Serial.println(F(" ****************************************************** "));
   Serial.println(F("           FINAL CONTROL ELEMENT SYSTEM           "));
   Serial.println(F("A -> BANK COOLER EXTERNAL S1           ||    B -> HR HEATING RESISTANCE")); 
   Serial.println(F("C -> AHF AIR HOMOGENIZING FAN          ||    D -> CSG COLD SYSTEM GENERATOR")); 
   Serial.println(F("E -> RAA RECIRCULATION OF AMBIENT AIR  ||    F -> RTD RECIRCULATION OF TEMPERATURE DELTA")); 
   Serial.println(F("G -> TFFR TEST FAN FLOW RECIRCULATION  ||    H -> CFG COLD FLOW GENERATOR")); 
   Serial.println(F("I -> FPT FAN PELTIER TEST              ||    J -> FRT FAN RECIRCULATION TEST")); 
   Serial.println(F("K -> FHT FAN HEATER TEST               ||    L -> PUMP FLOW COOLANT LIQUID "));
   Serial.println(F("M->  HEAT 130 WATT                     ||    N->  HEAT FAST 130 WATT "));
   Serial.println(F("& -> VALVE TEST "));
   Serial.println(F("q -> quit                              ||    '' -> space bar for repeat Menu")); 
   }
   
/************************************************************** 
********************** Menu_printHelp ====>>>> 
***************************************************************/
void printHelp(void)
{
   
}

/************************************************************** 
********************** Menu_Temperature_Sensors ====>>>>
***************************************************************/
void Menu_Temperature_Sensors(void){
   Serial.println(F("*****************************************************************************************************"));
   Serial.println(F("0 -> TCL TEMPERATURE COOLANT LIQUID   ||    1 -> TA TEMPERATURE SENSOR TA")); 
   Serial.println(F("2 -> TB TEMPERATURE SENSOR TB         ||    3 -> TIC TEMPERATURE SENSOR TIC")); 
   Serial.println(F("4 -> TE TEMPERATURE SENSOR TE         ||    5 -> TI TEMPERATURE SENSOR TI")); 
   Serial.println(F("6 -> TC1 TEMPERATURE SENSOR T_PEB1    ||    7 -> TC2 TEMPERATURE SENSOR T_PEB2"));
   Serial.println(F("8 -> TC3 TEMPERATURE SENSOR T_PEB3    ||    9 -> ATS ALL TEMPERATURE SENSORS"));
   Serial.println(F("q -> quit                             ||    '' -> space bar for repeat Menu")); 
   }
/************************************************************** 
********************** Menu_Others_Sensors ====>>>> 
***************************************************************/
void Menu_Others_Sensors(void){
   Serial.println(F("*****************************************************************************************************"));
   Serial.println(F("| -> IHS INTERNAL HUMIDITY SENSOR     ||    ! -> EHS EXTERNAL HUMIDITY SENSOR"));
   Serial.println(F("/ -> AHS ALL  HUMIDITY SENSORS        ||    + -> ASF ALL SENSORS FULL "));
   Serial.println(F(" q -> quit                            ||    '' -> space bar for repeat Menu")); 
   }

   /************************************************************** 
********************** Menu_printSystem ====>>>> 
***************************************************************/
void Calibration_sensors(void){
   Serial.println(F("*******************************************************************************************"));
   Serial.println(F("a -> HM0 Humidity MIN 0 %              ||    b -> HM100 Humidity MAX 100 %  ")); 
   Serial.println(F("c -> AHF AIR HOMOGENIZING FAN          ||    d -> CSG COLD SYSTEM GENERATOR")); 
   Serial.println(F("e -> RAA RECIRCULATION OF AMBIENT AIR  ||    f -> RTD RECIRCULATION OF TEMPERATURE DELTA")); 
   Serial.println(F("g -> TFFR TEST FAN FLOW RECIRCULATION  ||    h -> CFG COLD FLOW GENERATOR")); 
   Serial.println(F("i -> FPT FAN PELTIER TEST              ||    j -> FRT FAN RECIRCULATION TEST")); 
   Serial.println(F("k -> FHT FAN HEATER TEST               ||    l -> quit"));
   Serial.println(F(" q -> quit                             ||    '' -> space bar for repeat Menu")); 
   }

void Menu_Init_Copagetec(void)
{
   Serial.println(F(" ***************************************************************************************************** "));
   Serial.println(F("K -> RUN Copagetec Chamber V82         ||  '' -> space bar for repeat Menu"));  
   Serial.println(F("Y -> Controller PID RA actuator        ||    Z -> The response curve of the Ra actuator")); 
   Serial.println(F("W -> Controller PID RB actuator        ||    X -> The response curve of the Rb actuator")); 
   Serial.println(F("U -> Controller PID RC actuator        ||    V -> The response curve of the Rc actuator")); 
   Serial.println(F("S -> MENU FINAL CONTROL ELEMENT SYSTEM ||    T -> MENU TEMPERATURE SENSORS")); 
   Serial.println(F("O -> MENU OTHER SENSORS                ||    P -> MENU SYSTEM CALIBRATION")); 
   Serial.println(F("q -> quit"));   
}

/********************************************************** 
*********   CLEAR MENU  OTHER TERMINAL ********************
***********************************************************/
 void clearAndHome() 
{ 
  Serial.write(27); 
  Serial.print("[2J"); // clear screen 
  Serial.write(27); // ESC 
  Serial.print("[H"); // cursor to home 
} 
/********************************************************** 
*********   REPIT MENU   **********************************
***********************************************************/
void repitmenu() 
{ 
  Serial.write(27); 
  Serial.print("[2J"); // clear screen 
  Serial.write(27); // ESC 
  Serial.print("[H"); // cursor to home 
}  


/*********************************************************************************
********************  MENU SERIAL TEMP  ******************************************
**********************************************************************************/
  
/************************************************************** 
********************** Serial_Temp_menu ====>>>> 
***************************************************************/ 
void  Serial_Menu_Init_Copagetec(){
  if (Serial.available() >0){          // Check receive buffer.
    rxChar = Serial.read();            // Save character received. 
    Serial.flush();                    // Clear receive buffersdfdfsd.
//claro();
switch (rxChar) 
  {

/************************Menu_Others_Sensors****************/
/************************************************************** 
******** CASE K====>>>> RUN Copagetec Chamber V82
***************************************************************/
case 'K':  
    Serial.println(F("-> RUN Copagetec Chamber V82 "));
    do
       {
        input=Serial.read();
       // Run_Mode_Copagetec_Chamber(); 
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TCL IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE Y====>>>>  Controller PID RA actuator 
***************************************************************/
case 'Y':  
    Serial.println(F(" -> Controller PID RA actuator "));
     do
       {
         input=Serial.read();
         loop_CON_PID_Ra_actuator();
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TA IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
******** CASE Z====>>>> The response curve of the Ra actuator
***************************************************************/
case 'Z':  
    Serial.println(F(" -> The response curve of the Ra actuator "));
     do
       {
         input=Serial.read();
         Temperatura_TB();//temp ambient
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TB IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
      Menu_ppal();
break;
/************************************************************** 
******** CASE W====>>>> Controller PID RB actuator
***************************************************************/
case 'W':  
    Serial.println(F("-> Controller PID RB actuator  "));
     do
       {
           input=Serial.read();
        loop_CON_PID_Rb_actuator();
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TIC IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
    Menu_ppal();
break;
/************************************************************** 
******** CASE X====>>>> The response curve of the Rb actuator
***************************************************************/
case 'X':  
    Serial.println(F(" -> The response curve of the Rb actuator "));
    do
       {
        input=Serial.read();
        Temperatura_TE(); 
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TE IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE U====>>>> Controller PID RC actuator
***************************************************************/
case 'U':  
    Serial.println(F(" -> Controller PID RC actuator "));
    do
       {
        input=Serial.read();
        loop_CON_PID_Rc_actuator();
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TI IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
******** CASE V ====>>>> The response curve of the Rc actuator
***************************************************************/
case 'V':  
    Serial.println(F(" -> The response curve of the Rc actuator "));
     do
       {
          input=Serial.read();
         Temperatura_TC1(); 
       } while (input !=113);//(quit)
         Serial.println(F("Temperatura_TC1 IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE S ====>>>> MENU FINAL CONTROL ELEMENT SYSTEM 
***************************************************************/
case 'S':  
    Serial.println(F(" -> MENU FINAL CONTROL ELEMENT SYSTEM "));
     do
       {
          input=Serial.read();
          Menu_EFC_Actuators(); 
         Serial_EFC_Actuators_Menu();
       } while (input !=113);//(quit)
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Serial_Menu_Init_Copagetec();
break;
/************************************************************** 
******** CASE T8 ====>>>> MENU TEMPERATURE SENSORS
***************************************************************/
case 'T':  
    Serial.println(F(" -> MENU TEMPERATURE SENSORS "));
     do
       {
          input=Serial.read();
         Menu_Temperature_Sensors();
         Serial_temp_menu(); 
       } while (input !=113);//(quit)
         Serial.println(F("Temperatura_TC1 IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE O====>>>> MENU OTHER SENSORS  
***************************************************************/
case 'O':  
    Serial.println(F(" -> MENU OTHER SENSORS   "));
          input=Serial.read();
          Menu_Others_Sensors();
          Serial_Menu_Others_Sensors();
        Serial.println(F("TEST MODE MENU OTHER SENSORS"));
break;

/************************************************************** 
******** CASE P ====>>>> MENU SYSTEM CALIBRATION
***************************************************************/
case 'P':  
    Serial.println(F(" -> MENU SYSTEM CALIBRATION "));
          Serial_Calibration_Menu();
         Calibration_sensors();

break;
     }//END switch CASE (rxChar) 
   }//END  if Serial.available  Check receive buffer.
 }//END void  Serial_Control_Menu()
/*
Lo sketch usa 33586 byte (13%) dello spazio disponibile per i programmi. Il massimo è 253952 byte.
Le variabili globali usano 1756 byte (21%) di memoria dinamica, lasciando altri 6436 byte liberi per le variabili locali. Il massimo è 8192 byte.
*/
////*******END PROGRAM AND PAGE MENU SERIAL TEMP





  
/** Lo sketch usa 33586 byte (13%) dello spazio disponibile per i programmi. Il massimo è 253952 byte.
Le variabili globali usano 1756 byte (21%) di memoria dinamica, lasciando altri 6436 byte liberi per le variabili locali. Il massimo è 8192 byte.
**/
 
