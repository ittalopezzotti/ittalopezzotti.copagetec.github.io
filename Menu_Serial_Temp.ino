/************************************************************************************* 
**Author::: ITTALO PEZZOTTI ESCOBAR **************************************************
**Date::::: 26/04/2023- 26/05/2025****************************************************
**Project:: CALCULATION AND OBTAINING OF PARAMETERS TO GENERATE THERMAL CYCLES USING** 
**:::::::::PID CONTROL IN A NON-PRESSURIZED CLOSED SYSTEM FOR MATERIALS TEST**********
**Prototypes:::: CACOTE AND COPAGETEC*************************************************
**NAME PROGRAM::1_Copagetec_Temperature_Chamber_PID_V015.INO**************************
**************************************************************************************/

/************************************************************** 
********************** Serial_Temp_menu ====>>>> 
***************************************************************/ 
void  Serial_temp_menu(){
  if (Serial.available() >0){          // Check receive buffer.
    rxChar = Serial.read();            // Save character received. 
    Serial.flush();                    // Clear receive buffersdfdfsd.
//claro();
switch (rxChar) 
  {

/************************Menu_Others_Sensors****************/
/************************************************************** 
******** CASE 0====>>>> TCL TEMPERATURE COOLANT LIQUID
***************************************************************/
case '0':  
    Serial.println(F(" TCL TEMPERATURE COOLANT LIQUID "));
    do
       {
        input=Serial.read();
       // Temperatura_TV(); 
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TCL IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE 1====>>>> TA TEMPERATURE TA
***************************************************************/
case '1':  
    Serial.println(F(" TA TEMPERATURE TA "));
     do
       {
         input=Serial.read();
         Temperatura_TA();
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TA IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
******** CASE 2====>>>> TB TEMPERATURE TB
***************************************************************/
case '2':  
    Serial.println(F(" TB TEMPERATURE TB "));
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
******** CASE 3====>>>> TIC TEMPERATURE SENSOR TIC
***************************************************************/
case '3':  
    Serial.println(F(" TIC TEMPERATURE SENSOR TIC "));
     do
       {
           input=Serial.read();
         Temperatura_TIC();
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TIC IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
    Menu_ppal();
break;
/************************************************************** 
******** CASE 4====>>>> TE TEMPERATURE SENSOR TE
***************************************************************/
case '4':  
    Serial.println(F(" TE TEMPERATURE SENSOR TE "));
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
******** CASE 5====>>>> TI TEMPERATURE SENSOR TI
***************************************************************/
case '5':  
    Serial.println(F(" TI TEMPERATURE SENSOR TI "));
    do
       {
        input=Serial.read();
        Temperatura_TI(); 
       } while (input !=113);//(quit)
          Serial.println(F("TEMP TI IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
******** CASE 6====>>>> TC1 TEMPERATURE SENSOR T_PEB1
***************************************************************/
case '6':  
    Serial.println(F(" TC1 TEMPERATURE SENSOR T_PEB1 "));
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
******** CASE 7====>>>> TC2 TEMPERATURE SENSOR T_PEB2
***************************************************************/
case '7':  
    Serial.println(F(" TC2 TEMPERATURE SENSOR T_PEB2 "));
     do
       {
          input=Serial.read();
         Temperatura_TC2(); 
       } while (input !=113);//(quit)
         Serial.println(F("Temperatura_TC2 IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;
/************************************************************** 
******** CASE 8====>>>> TC3 TEMPERATURE SENSOR T_PEB3
***************************************************************/
case '8':  
    Serial.println(F(" TC3 TEMPERATURE SENSOR T_PEB3 "));
     do
       {
          input=Serial.read();
         Temperatura_TC3(); 
       } while (input !=113);//(quit)
         Serial.println(F("Temperatura_TC1 IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
 Menu_ppal();
break;

/************************************************************** 
******** CASE 9====>>>> ATS ALL TEMPERATURE SENSORS 
***************************************************************/
case '9':  
    Serial.println(F(" ATS ALL TEMPERATURE SENSORS "));
     do
       {
          input=Serial.read();
          All_Temperature(); 
       } while (input !=113);//(quit)
         Serial.println(F("ATS ALL TEMPERATURE SENSORS IN OFF MODE"));
      Serial.println(F("Exit program"));
      Serial.write(12);//ASCII for a Form feed
  Menu_ppal();
break;

   /********************************************************** 
     *********    print NEW MENU  ******************************
     ***********************************************************/
       case 'q':                          // If received a ?:
            loop_Reset_System();
           // Menu_ppal();
        break;

    }//END switch CASE (rxChar) 
   }//END  if Serial.available  Check receive buffer.
 }//END void  Serial_Control_Menu()
/*
Lo sketch usa 33586 byte (13%) dello spazio disponibile per i programmi. Il massimo è 253952 byte.
Le variabili globali usano 1756 byte (21%) di memoria dinamica, lasciando altri 6436 byte liberi per le variabili locali. Il massimo è 8192 byte.
*/
////*******END PROGRAM AND PAGE MENU SERIAL TEMP
