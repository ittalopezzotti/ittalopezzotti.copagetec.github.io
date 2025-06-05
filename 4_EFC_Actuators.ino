/************************************************************************************* 
**Author::: ITTALO PEZZOTTI ESCOBAR **************************************************
**Date::::: 26/04/2023- 26/05/2025****************************************************
**Project:: CALCULATION AND OBTAINING OF PARAMETERS TO GENERATE THERMAL CYCLES USING** 
**:::::::::PID CONTROL IN A NON-PRESSURIZED CLOSED SYSTEM FOR MATERIALS TEST**********
**Prototypes:::: CACOTE AND COPAGETEC*************************************************
**NAME PROGRAM::1_Copagetec_Temperature_Chamber_PID_V015.INO**************************
**************************************************************************************/
  
/************************************************************** 
********************** Serial_EFC_Actuators_Menu ====>>>> 
***************************************************************/ 
void  Serial_EFC_Actuators_Menu(){
  if (Serial.available() >0){          // Check receive buffer.
    rxChar = Serial.read();            // Save character received. 
    Serial.flush();                    // Clear receive buffersdfdfsd.
//claro();
switch (rxChar) 
  {
/************************************************************** 
******** CASE B====>>>> HR HEATING RESISTANCE
***************************************************************/
case 'B': 
    Serial.println(F(" B -> HR HEATING RESISTANCE ")); 
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Heater, HIGH);
              Serial.print(F("Led and device in high mode"));
              digitalClockDisplay();
              delay(200);
              }
       else if (input==50)
           {
            digitalWrite(Pin_Heater, LOW);
            Serial.println(F("Led and device in low mode"));
            }
       } while (input !=113);//(quit)
   digitalWrite(Pin_Heater, LOW);
   Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
******** CASE C====>>>> AHF AIR HOMOGENIZING FAN
***************************************************************/
case 'C':  
    Serial.println(F(" AHF AIR HOMOGENIZING FAN "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
         All_Other_Sensors();
       if (input==49) 
              {
                
             digitalWrite(Pin_Fan_Heater, HIGH);
             digitalWrite(Pin_Heater, HIGH);
             delay(500);//delay for visual
             Serial.println(F("FLOW AIR HOT HOMOGENIZING ON "));
              All_Other_Sensors();
              }
       else if (input==50)
           {
             digitalWrite(Pin_Heater, LOW);
             delay(1000);//delay for visual
             digitalWrite(Pin_Fan_Heater, LOW);
             delay(500);//delay for visual
             Serial.println(F("FLOW AIR HOT HOMOGENIZING OFF "));
              All_Other_Sensors();
            }
       } while (input !=113);//(quit)
             digitalWrite(Pin_Heater, LOW);
             delay(1000);//delay for visual
             digitalWrite(Pin_Fan_Heater, LOW);
             delay(500);//delay for visual
             Serial.println(F("FLOW AIR SYSTEM OFF "));
              All_Other_Sensors();
   Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
******** CASE D====>>>> CSG COLD SYSTEM GENERATOR
***************************************************************/
case 'D':  
    Serial.println(F(" CSG COLD SYSTEM GENERATOR "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Peltier_B1, HIGH);  
              delay(1000);//delay for visual
              digitalWrite(Pin_Peltier_PE1, HIGH);
              delay(2000);//delay for visual
              Serial.println(F("COLD SYSTEM GENERATOR IS ON"));
              }
       else if (input==50)
           {
              digitalWrite(Pin_Peltier_PE1, LOW);  
              delay(7000);//delay for visual
              digitalWrite(Pin_Fan_Peltier_B1, LOW); 
              delay(2000);//delay for visual
              Serial.println(F("COLD SYSTEM GENERATOR IS OFF"));
            }
       } while (input !=113);///SALE CON EL q EN ASCCI (quit)
              digitalWrite(Pin_Peltier_PE1, LOW);  
              delay(7000);//delay for visual
              digitalWrite(Pin_Fan_Peltier_B1, LOW); 
              delay(2000);//delay for visual
              Serial.println(F("COLD SYSTEM GENERATOR IS OFF"));
            Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
******** CASE E====>>>> RAA RECIRCULATION OF AMBIENT AIR
***************************************************************/
case 'E':  
    Serial.println(F(" RAA RECIRCULATION OF AMBIENT AIR "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Recircu, HIGH);
              Serial.println(F("Led and device in high mode"));
              }
       else if (input==50)
           {
            digitalWrite(Pin_Fan_Recircu, LOW);
            Serial.println(F("Led and device in low mode"));
            }
       } while (input !=113);//(quit)
   digitalWrite(Pin_Fan_Recircu, LOW);
   Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
     // delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
******** CASE F====>>>> RTD RECIRCULATION OF TEMPERATURE DELTA
***************************************************************/
case 'F':  
    Serial.println(F(" RTD RECIRCULATION OF TEMPERATURE DELTA "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              /*** COLD SYSTEM MODE IS IN ON ***/
              digitalWrite(Pin_Fan_Peltier_B1, HIGH);  
              delay(200);//delay for visual
              digitalWrite(Pin_Peltier_PE1, HIGH);
              delay(8000);//delay for visual
              digitalWrite(Pin_Fan_Recircu, HIGH);
              Serial.println(F("COLD SYSTEM ON MODE"));
                /*** HOT SYSTEM MODE IS IN ON ***/
               digitalWrite(Pin_Heater, HIGH);
               delay(300);//delay for visual
               digitalWrite(Pin_Fan_Heater, HIGH);
               delay(500);//delay for visual
               Serial.println(F("HOT SYSTEM ON MODE"));
               Serial.println(F("DELTA TEMPERATURA MODE IS ON"));
              }
       else if (input==50)
           {
              /*** COLD SYSTEM MODE IS IN ON ***/
              digitalWrite(Pin_Fan_Peltier_B1, LOW);  
              delay(200);//delay for visual
              digitalWrite(Pin_Peltier_PE1, LOW);
              delay(8000);//delay for visual
              digitalWrite(Pin_Fan_Recircu, LOW);
              Serial.println(F("COLD SYSTEM ON MODE"));
                /*** HOT SYSTEM MODE IS IN OFF ***/
               digitalWrite(Pin_Heater, LOW);
               delay(300);//delay for visual
               digitalWrite(Pin_Fan_Heater, LOW);
               delay(500);//delay for visual
               Serial.println(F("HOT SYSTEM OFF MODE"));
               Serial.println(F("DELTA TEMPERATURA MODE IS OFF"));
            }
       } while (input !=113);//(quit)
              /*** COLD SYSTEM MODE IS IN ON ***/
              digitalWrite(Pin_Fan_Peltier_B1, LOW);  
              delay(200);//delay for visual
              digitalWrite(Pin_Peltier_PE1, LOW);
              delay(8000);//delay for visual
              digitalWrite(Pin_Fan_Recircu, LOW);
              Serial.println(F("COLD SYSTEM ON MODE"));
                /*** HOT SYSTEM MODE IS IN OFF ***/
               digitalWrite(Pin_Heater, LOW);
               delay(300);//delay for visual
               digitalWrite(Pin_Fan_Heater, LOW);
               delay(500);//delay for visual
               Serial.println(F("HOT SYSTEM OFF MODE"));
               Serial.println(F("DELTA TEMPERATURA MODE IS OFF"));
            
    Serial.println(F("ALL DEVICE IS IN LOW MODE"));
    Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
    //  delay(500);//delay for visual
 Menu_ppal();
break;


/************************************************************** 
*********** CASE G====>>>> TFFR TEST FAN FLOW RECIRCULATION
***************************************************************/
case 'G':  
    Serial.println(F(" TFFR TEST FAN FLOW RECIRCULATION "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              Serial.println(F("ALL FANS TEST"));
              digitalWrite(Pin_Fan_Recircu, HIGH);
              digitalWrite(Pin_Fan_Heater, HIGH);
              digitalWrite(Pin_Fan_Peltier_B1, HIGH);  
              delay(300);//delay for visual
              Serial.println(F("ALL FANS ON"));
              }
       else if (input==50)
           {
              digitalWrite(Pin_Fan_Recircu, LOW);
              digitalWrite(Pin_Fan_Heater, LOW);
              digitalWrite(Pin_Fan_Peltier_B1, LOW);  
              Serial.println(F("ALL FANS OFF"));
            }
       } while (input !=113);// (quit)
              digitalWrite(Pin_Fan_Recircu, LOW);
              digitalWrite(Pin_Fan_Heater, LOW);
              digitalWrite(Pin_Fan_Peltier_B1, LOW);  
              Serial.println(F("ALL FANS OFF"));
    Serial.println(F("Exit program"));
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
*********** CASE H====>>>> CFG COLD FLOW GENERATOR 
***************************************************************/
case 'H':  
    Serial.println(F(" COLD FLOW GENERATOR"));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
        All_Other_Sensors();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Peltier_B1, HIGH);  
              delay(200);//delay for visual
              digitalWrite(Pin_Peltier_PE1, HIGH);
              delay(8000);//delay for visual
              digitalWrite(Pin_Fan_Recircu, HIGH);
              Serial.println(F("FLOW COLD SYSTEM ON MODE"));
                All_Other_Sensors();
              
              }
       else if (input==50)
           {
            Serial.println(F("wait turn off peltier"));
            digitalWrite(Pin_Peltier_PE1, LOW);
            delay(8000);//delay for visual
            digitalWrite(Pin_Fan_Recircu, LOW);
            delay(8000);//delay for visual
            digitalWrite(Pin_Fan_Peltier_B1, LOW);  
            delay(100);//delay for visual
              All_Other_Sensors();
            Serial.println(F("FLOW COLD OFF MODE"));
            }
       } while (input !=113);//(quit)
            Serial.println(F("wait turn off peltier"));
            digitalWrite(Pin_Peltier_PE1, LOW);
            delay(8000);//delay for visual
            digitalWrite(Pin_Fan_Recircu, LOW);
            delay(8000);//delay for visual
            digitalWrite(Pin_Fan_Peltier_B1, LOW);  
            delay(100);//delay for visual
              All_Other_Sensors();
            Serial.println(F("FLOW COLD OFF MODE"));
            Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
     // delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
*********** CASE I====>>>> FAN PELTIER TEST
***************************************************************/
case 'I':  
    Serial.println(F(" FPT FAN PELTIER TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Peltier_B1, HIGH);  
              delay(300);//delay for visual
              Serial.println(F("FAN PELTIER ON "));
              }
       else if (input==50)
           {
              digitalWrite(Pin_Fan_Peltier_B1, LOW);  
              Serial.println(F("FAN PELTIER OFF"));
            }
       } while (input !=113);//(quit)
              digitalWrite(Pin_Fan_Peltier_B1, LOW);    
              Serial.println(F("FAN PELTIER OFF"));
     Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
     // delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
*********** CASE J====>>>> FRT FAN RECIRCULATION TEST
***************************************************************/
case 'J':  
    Serial.println(F(" FAN RECIRCULATION TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Recircu, HIGH);
                Serial.println(F("FAN AMBIENT FLOW ON"));
              }
       else if (input==50)
           {
              digitalWrite(Pin_Fan_Recircu, LOW);
              Serial.println(F("FAN AMBIENT FLOW OFF"));
            }
       } while (input !=113);//(quit)
              digitalWrite(Pin_Fan_Recircu, LOW);
              Serial.println(F("FAN AMBIENT FLOW OFF"));
      Serial.println(F("Exit program"));
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
*********** CASE  K====>>>> FHT FAN HEATER TEST  
***************************************************************/
case 'K':  
    Serial.println(F(" FAN HEATER TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
              digitalWrite(Pin_Fan_Heater, HIGH);
              Serial.println(F("FAN HEATER MODE ON"));
              }
       else if (input==50)
           {
               digitalWrite(Pin_Fan_Heater, LOW);
               Serial.println(F("FAN HEATER MODE OFF"));
            }
       } while (input !=113);//(quit)
                digitalWrite(Pin_Fan_Heater, LOW);
                Serial.println(F("FAN HEATER MODE OFF"));
     Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;

/************************************************************** 
*********** CASE  K====>>>> PUMP FLOW COOLANT LIQUID 
***************************************************************/
case 'L':  
    Serial.println(F(" FLOW COOLANT LIQUID   "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
         input=Serial.read();
          Flow_Coolant_Liquid();
       } while (input !=113);//(quit)
          Serial.println(F(" SECADOR IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break; 

/************************************************************** 
*********** CASE M====>>>> COLD FLOW GENERATOR II
***************************************************************/
case 'M':  
    Serial.println(F(" HEAT 1300 WATT TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
        // Heat_R1300_Off();
            Heat_R1300();
        All_Other_Sensors();
        Temp_Heat_1300=thermocouple1.readCelsius();
       if (input==49) 
              {
                Heat_R1300();
                All_Other_Sensors();
             }
       else if (input==50)
           {
            Serial.println(F("wait turn off R 1300"));
            digitalWrite(Pin_Heat_R1300, LOW);
              All_Other_Sensors();
            Serial.println(F("FLOW COLD OFF MODE"));
            }
       } while (input !=113);//(quit)
            Serial.println(F("wait turn off R 1300"));
            digitalWrite(Pin_Heat_R1300, LOW);
              All_Other_Sensors();
            Serial.println(F("FLOW COLD OFF MODE"));
            Serial.println(F("Exit program"));
     //clearAndHome();
     //Code tested with Putty terminal
      Serial.write(12);//ASCII for a Form feed
      //delay(500);//delay for visual
 Menu_ppal();
break;
/************************************************************** 
*********** CASE Y====>>>> HEAT 130 WATT  FAST TEST
***************************************************************/
case '&':  
    Serial.println(F(" HEAT 130 WATT FAST TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
         Heat_R1300_fast();
        All_Other_Sensors2();
      //  Temp_Heat_1300=thermocouple1.readCelsius();
       if (input==49) 
              {
               // Heat_R1300_fast();
                All_Other_Sensors2();
             }
       else if (input==50)
           {
            Serial.println(F("wait turn off R 1300"));
            digitalWrite(Pin_Heat_R1300, LOW);
            Serial.println(F("FLOW COLD OFF MODE"));
            }
       } while (input !=113);//(quit)
            Serial.println(F("wait turn off R 1300"));
            digitalWrite(Pin_Heat_R1300, LOW);
            All_Other_Sensors2();
            Serial.println(F("FLOW COLD OFF MODE"));
            Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
*********** CASE N====>>>> Rele_Valve_Output_Air
***************************************************************/
case 'N':  
    Serial.println(F(" VALVE OUTPUT AIR TEST "));
    Serial.println(F(" press: || 1. turn 'ON' || 2: turn 'OFF' "));
     do
       {
        input=Serial.read();
       if (input==49) 
              {
                Serial.println(F("VALVE OUTPUT AIR ON"));
                 delay(200);//delay for visual
              }
       else if (input==50)
           {
              Serial.println(F("VALVE OUTPUT AIR OFF"));
               delay(200);//delay for visual
            }
       } while (input !=113);// (quit)
            Serial.println(F("VALVE OUTPUT AIR OFF"));
      Serial.println(F("Exit program"));
     //Code tested with Putty terminal
   //   Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

//********************************************************        
       case '?':                          // If received a ?:
          Menu_ppal();
        break;
 
     /********************************************************** 
     *********    print NEW MENU  ******************************
     ***********************************************************/
       case ' ':                          // If received a ?:
            Menu_ppal();
        break;

     }//END switch CASE (rxChar) 
   }//END  if Serial.available  Check receive buffer.
 }//END void  Serial_Control_Menu()
/*
Lo sketch usa 33586 byte (13%) dello spazio disponibile per i programmi. Il massimo è 253952 byte.
Le variabili globali usano 1756 byte (21%) di memoria dinamica, lasciando altri 6436 byte liberi per le variabili locali. Il massimo è 8192 byte.
*/
////*******END PROGRAM AND PAGE EFC_Actuators();
