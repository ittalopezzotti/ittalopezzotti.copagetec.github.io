/************************************************************************************* 
**Author::: ITTALO PEZZOTTI ESCOBAR **************************************************
**Date::::: 26/04/2023- 26/05/2025****************************************************
**Project:: CALCULATION AND OBTAINING OF PARAMETERS TO GENERATE THERMAL CYCLES USING** 
**:::::::::PID CONTROL IN A NON-PRESSURIZED CLOSED SYSTEM FOR MATERIALS TEST**********
**Prototypes:::: CACOTE AND COPAGETEC*************************************************
**NAME PROGRAM::1_Copagetec_Temperature_Chamber_PID_V015.INO**************************
**************************************************************************************/

/***************************************************************************************************************
 ******************************FUNCTIONS AND ROUTINES **********************************************************
****************************************************************************************************************/
/** THIS CODE DETECTS WHICH KEY HAS BEEN PRESSED**
** AND THEN STORES IT IN THE VARIABLE KEYPRESS****/
void getpulsacion() {
  short channel = 1;
  Wire.requestFrom(KeypapAddress, 1 << channel);
  if (Wire.available())
  {
    pulsacion = Wire.read();
  }
  Wire.endTransmission();
  delay(200);
}
/** THIS CODE ALLOWS YOU TO MOVE THE ARROW ********
******* WITHIN THE MENU THROUGH THE DISPLAY*******/
int Puntero(int NumMenus)
{
  switch (pulsacion) {
    case F2:
      if (x < NumMenus) {
        x = x + 1;
      }
      break;
    case F1:
      if (x > 0) {
        x = x - 1;
      }
      break;
  }
  return x;
}
/***THIS CODE ALLOWS UPDATING THE DATE*****************
***SENDING DATA SERIALLY TO THE CLOCK******************
***ACTIVATED 04/28/2022 THIS INCREASES TO 4% AND 8%****
***IN THE RTC SYSTEM AND MENU ************************/
  void digitalClockDisplay()
   {     Serial.print(hour());
         printDigits(minute());
         printDigits(second());
         Serial.print(F("; "));
         Serial.print(day());
         Serial.print(F("/"));
         Serial.print(month());
         Serial.print(F("/"));
         Serial.print(year());
         Serial.print(F("; "));
         Serial.println();
   }
/** AUXILIARY FUNCTION TO ALWAYS PRINT***********
** 1 DIGIT IN DISPLAY AND SERIAL****************/   
void printDigits(int digits)
   {     Serial.print(F(":"));
         if(digits < 10)
            Serial.print('0');
         Serial.print(digits);
   }
   

//void Set_time() {
//  // Declaramos una estructura "tm" llamada datetime para almacenar
//  // La fecha y la hora actual obtenida desde el chip RTC.
//  struct tm datetime;
//
//  // Obtener la fecha y hora desde el chip RTC
//  if(RTC.read(datetime)) {
//
//    // Comenzamos a imprimir la informacion
//    Serial.print("OK, Time = ");
//    print2digits(datetime.tm_hour);
//    Serial.write(':');
//    print2digits(datetime.tm_min);
//    Serial.write(':');
//    print2digits(datetime.tm_sec);
//    Serial.print(", Date (D/M/Y) = ");
//    Serial.print(datetime.tm_mday);
//    Serial.write('/');
//    Serial.print(datetime.tm_mon);
//    Serial.write('/');
//    Serial.print(time_tm2y2k(datetime.tm_year));
//    Serial.println();
//    delay(1000);
//  }
//  else {
//    if (RTC.chipPresent()) {
//      // El reloj esta detenido, es necesario ponerlo a tiempo
//      Serial.println("The DS1307 is stopped.");
//      Serial.println("Run timeset example.");
//      Serial.println();
//    }
//    else {
//      // No se puede comunicar con el RTC en el bus I2C, revisar las conexiones.
//      Serial.println("DS1307 Not Detected.");
//      Serial.println("Check hardware connections and reset board.");
//      Serial.println();
//    }
//    // Esperamos indefinidamente
//    for(;;);
//  }
//}/**


/** AUXILIARY FUNCTION TO ALWAYS PRINT************
** 2 DIGITS IN DISPLAY AND SERIAL****************/
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void loop_Reset_System() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == COMMAND_RESET) {
      Serial.println("Starting...");
      reset_program(); 
    }
  }
}

/** Reset function ***/
void reset_program() {
  delay(10);
  reset_program(); 
}

/************************************************************** 
*****************   Calc temperature TA por analogic 14 ====>>>> 
***************************************************************/
  /* // read the value, TensionAnalog = analogRead(A14);    
   ******** TA:::>>>SENSOR HEATER ****************************/ 
  void Temperatura_TA() 
   {
    Ta = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TA = analogRead(A14);   
    Ta =  Ta + ( ( (TensionAnalog_TA /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TA = Ta/5; //compute the average value
   Serial.print(F("T_Ta: "));
   Serial.print(TA);
   Serial.print(F(" C; "));
    digitalClockDisplay();
   delay(200);
    }

/************************************************************** 
*****************   Calc temperatura TB por analogic 13 ====>>>> 
***************************************************************/
  /* // read the value, TensionAnalog = analogRead(A13);    
   ******** TB:::>>>SENSOR HEATER ****************************/ 
 
  void Temperatura_TB() 
   {
    Tb = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TB = analogRead(A13);   
    Tb =  Tb + ( ( (TensionAnalog_TB /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TB = Tb/5; //compute the average value
   Serial.print(F("T_Tb: "));
   Serial.print(TB);
   Serial.print(F(" C; "));
    digitalClockDisplay();
   delay(200);
    }

    // */************************************************************** 
//*****************   Calc temperatura TIC por analogic A9 ====>>>> 
//***************************************************************/
//  /* // read the value, TensionAnalog = analogRead(A9);    
//   ******** TIC:::>>>SENSOR FLOW RECIRCULATION******************/
  void Temperatura_TIC() 
    {
    Tic = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TIC = analogRead(A9);   
    Tic =  Tic + ( ( (TensionAnalog_TIC /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TIC = Tic/5; //compute the average value
   Serial.print(F("T_Tic: "));
   Serial.print(TIC);
   Serial.print(F(" C; "));
   digitalClockDisplay();
   delay(200);
    }

// */************************************************************** 
//*****************   Calc temperatura TE por analogic A10 ====>>>> 
//***************************************************************/
//  /* // read the value, TensionAnalog = analogRead(A10);    
//   ******** TC:::>>>SENSOR FLOW RECIRCULATION******************/
  void Temperatura_TE() 
    {
    Te = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TE = analogRead(A10);   
    Te =  Te + ( ( (TensionAnalog_TE /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TE = Te/5; //compute the average value
   Serial.print(F("T_Te: "));
   Serial.print(TE);
   Serial.print(F(" C; "));
   digitalClockDisplay();
   delay(200);
    }
    
// */************************************************************** 
//*****************   Calc temperatura TI por analogic A12 ====>>>> 
//***************************************************************/
//  /* // read the value, TensionAnalog = analogRead(A12);    
//   ******** TI:::>>>SENSOR FLOW RECIRCULATION******************/
  void Temperatura_TI() 
    {
    Ti = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TE = analogRead(A12);   
    Ti =  Ti + ( ( (TensionAnalog_TI /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TI = Ti/5; //compute the average value
   Serial.print(F("T_Ti: "));
   Serial.print(TI);
   Serial.print(F(" C; "));
   digitalClockDisplay();
   delay(200);
    }

   
/************************************************************** 
*****************   Calc temperatura TC1 por analogic A7 ====>>>> 
***************************************************************/
  /* // read the value, TensionAnalog = analogRead(A7);    
   ******** TC1:::>>>TEMPERATURA PELTIER BANK 1*******************/ 

  void Temperatura_TC1() 
   {
    Tc1 = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TC1 = analogRead(A7);   
    Tc1 =  Tc1 + ( ( (TensionAnalog_TC1 /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TC1 = Tc1/5; //compute the average value
    //TEMPERATURA PELTIER BANK 1
   Serial.print(F("T_Tc1: "));
   Serial.print(TC1);
   Serial.print(F(" C; "));
    digitalClockDisplay();
   delay(200);
    }
    
/************************************************************** 
*****************   Calc temperatura TC2 por analogic A6 ====>>>> 
***************************************************************/
  /* // read the value, TensionAnalog = analogRead(A6);    
   ******** TC2:::>>>TEMPERATURA PELTIER BANK 2*******************/
  
  void Temperatura_TC2() 
   {
    Tc2 = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TC2 = analogRead(A6);   
    Tc2 =  Tc2 + ( ( (TensionAnalog_TC2 /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TC2 = Tc2/5; //compute the average value
    //TEMPERATURA PELTIER BANK 1
   Serial.print(F("T_Tc2: "));
   Serial.print(TC2);
   Serial.print(F(" C; "));
    digitalClockDisplay();
   delay(200);
    }
    
/************************************************************** 
*****************   Calc temperatura TC3 por analogic A5 ====>>>> 
***************************************************************/
  /* // read the value, TensionAnalog = analogRead(A5);    
   ******** TC3:::>>>TEMPERATURA PELTIER BANK 3*******************/
    void Temperatura_TC3() 
   {
    Tc3 = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TC2 = analogRead(A5);   
    Tc3 =  Tc3 + ( ( (TensionAnalog_TC3 /204.8)  -0.5) / 0.01);
    delay(15);
  }
    TC3 = Tc3/5; //compute the average value
    //TEMPERATURA PELTIER BANK 3
   Serial.print(F("T_Tc3: "));
   Serial.print(TC3);
   Serial.print(F(" C; "));
    digitalClockDisplay();
   delay(200);
    }

 
///************************************************************** 
//*****************   Calc temperatura TE por analogic 13 ====>>>> 
//***************************************************************/
  void All_Temperature() 
   {
    // float TV = getTempva(); //SENS TEMP VA
     Temp_Heat_1300=thermocouple1.readCelsius();//TERMOCUPLA
     Temp_Sec_Air=thermocouple2.readCelsius();//TERMOCUPLA 2
     
     Ta = 0;  //INPUTS MCP9700
     Tb = 0;  //INPUTS MCP9700
     Te = 0;  //INPUTS MCP9700
     Ti = 0;  //INPUTS MCP9700
     Tf = 0;  //INPUTS DS18B20
     Tic = 0;  //INPUTS MCP9700
     Tcl = 0;  //INPUTS MCP9700
     Tc1 = 0;  //INPUTS MCP9700
     Tc2 = 0;  //INPUTS MCP9700
     Tc3 = 0;  //INPUTS MCP9700
     T_IAH = 0;//INPUTS DS18B20
     Ttub=0;   //INPUTS DS18B20
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TA = analogRead(A14);   
    Ta =  Ta + ( ( (TensionAnalog_TA /204.8)  -0.5) / 0.01);
    delay(15);
    }
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TB = analogRead(A13);   
    Tb =  Tb + ( ( (TensionAnalog_TB /204.8)  -0.5) / 0.01);
    delay(15);
    }
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TI = analogRead(A12);   
    Ti =  Ti + ( ( (TensionAnalog_TI /204.8)  -0.5) / 0.01);
    delay(15);
    }
   
   for (int i=1;i<=5;i++)
    {
    TensionAnalog_TCL = analogRead(A11);   
    Tcl =  Tcl + ( ( (TensionAnalog_TCL /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TE = analogRead(A10);   
    Te =  Te + ( ( (TensionAnalog_TE /204.8)  -0.5) / 0.01);
    delay(15);
   }
   for (int i=1;i<=5;i++)
    {
    TensionAnalog_TIC = analogRead(A9);   
    Tic =  Tic + ( ( (TensionAnalog_TIC /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++){
    TensionAnalog_TC1 = analogRead(A7);   
    Tc1 =  Tc1 + ( ( (TensionAnalog_TC1 /204.8)  -0.5) / 0.01);
    delay(15);
    }
    
    for (int i=1;i<=5;i++){
    TensionAnalog_TC2 = analogRead(A6);   
    Tc2 =  Tc2 + ( ( (TensionAnalog_TC2 /204.8)  -0.5) / 0.01);
    delay(15);
    }
    
    for (int i=1;i<=5;i++){
    TensionAnalog_TC3 = analogRead(A5);   
    Tc3 =  Tc3 + ( ( (TensionAnalog_TC3 /204.8)  -0.5) / 0.01);
    delay(15);
    }
    for (int i=1;i<=5;i++){
    TensionAnalog_Ttub = analogRead(A14);   
    Ttub =  Ttub + ( ( (TensionAnalog_Ttub /204.8)  -0.5) / 0.01);
    delay(15);
    }
   for (int i=1;i<=5;i++)
   {
    TensionAnalog_TIAH = analogRead(A15);   
    T_IAH =  T_IAH + ( ( (TensionAnalog_TIAH /204.8)  -0.5) / 0.01);
    delay(15);
   }
      TA = Ta/5; //compute the average value
      TB = Tb/5; //compute the average value
      
      TI = Ti/5; //compute the average value
      TCL = Tcl/5; //compute the average value
      TE = Te/5; //compute the average value
        
      TIC = Tic/5; //compute the average value
      TC1 = Tc1/5; //compute the average value
      TC2 = Tc2/5; //compute the average value
      TC3 = Tc3/5; //compute the average value
      
      TTUB = Ttub/5; //compute the average value
      T_IAH = T_IAH/5; //compute the average value

      
     
   Serial.print(F("T_TA: "));
   Serial.print(TA);
   Serial.print(F(" C; "));
   Serial.print(F("T_TB: "));
   Serial.print(TB);
   Serial.print(F(" C; "));


   Serial.print(F("T_TI: "));
   Serial.print(TI);
   Serial.print(F(" C; "));
   Serial.print(F("T_TCL: "));
   Serial.print(TCL);
   Serial.print(F(" C; "));
   Serial.print(F("T_TE: "));
   Serial.print(TE);
   Serial.print(F(" C; "));

   Serial.print(F("T_TIC: "));
   Serial.print(TIC);
   Serial.print(F(" C; "));
   Serial.print(F("T_TC1: "));
   Serial.print(TC1);
   Serial.print(F(" C; "));
   Serial.print(F("T_TC2: "));
   Serial.print(TC2);
   Serial.print(F(" C; "));
   Serial.print(F("T_TC3: "));
   Serial.print(TC3);
   Serial.print(F(" C; "));
    
   Serial.print(F("T_H1300: "));
   Serial.print(Temp_Heat_1300);
   Serial.print(F(" C; "));

   Serial.print(F("T_OAH: "));
   Serial.print(thermocouple2.readCelsius());
   Serial.print(F(" C; "));

   Serial.print(F("T_IAH: "));
   Serial.print(T_IAH);
   Serial.print(F(" C; "));

   Serial.print(F("T_Tub: "));
   Serial.print(TTUB);
   Serial.print(F(" C; "));
   
   digitalClockDisplay();
   delay(1000);
   
    }


void Humidity_H1() 
   {
  // Wait 5 Seconds before measure
  delay(5000);
 
  // We read the relative humidity
  float h =  dht_H1.readHumidity();
  // We read the temperature in degrees Celsius (default)
  float t = dht_H1.readTemperature();
  // Llet's give the temperature in degrees Fahrenheit
  float f = dht_H1.readTemperature(true);

 
  // CWe check if there has been any error in the reading
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error sensor DHT11");
    return;
  }
 
  // Calculate the heat index in Fahrenheit
  float hif = dht_H1.computeHeatIndex(f, h);
  // Calculate the heat index in degrees Celsius
  float hic = dht_H1.computeHeatIndex(t, h, false);
 
  Serial.print(F("H1: "));
  Serial.print(h);
  Serial.print(F("; "));
  
  Serial.print(F("T1: "));
  Serial.print(t);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
  Serial.print(f);
  Serial.print(F("; "));
  
  Serial.print(F("Q1: "));
  Serial.print(hic);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
  Serial.print(hif);
  Serial.print(F(" F"));
  Serial.print(F("; "));
  
   digitalClockDisplay();
   delay(200);
 }
 
void Humidity_H2() 
   {

  delay(5000);
  float h =  dht_H2.readHumidity();
  float t = dht_H2.readTemperature();
  float f = dht_H2.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");
    return;
  }
 

  float hif = dht_H2.computeHeatIndex(f, h);
  float hic = dht_H2.computeHeatIndex(t, h, false);
 
  Serial.print(F("H2: "));
  Serial.print(h);
  Serial.print(F("; "));
  
  Serial.print(F("T2: "));
  Serial.print(t);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
  Serial.print(f);
  Serial.print(F("; "));
  
  Serial.print(F("Q2: "));
  Serial.print(hic);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  Serial.print(hif);
  Serial.print(F(" F"));
  Serial.print(F("; "));
   digitalClockDisplay();
   delay(200);
 }

  void All_Humidity() 
   {
   delay(5000);
  float h1 = dht_H1.readHumidity();
  float h2 = dht_H2.readHumidity();
  float t1 = dht_H1.readTemperature();
  float t2 = dht_H2.readTemperature();
  float f1 = dht_H1.readTemperature(true);
  float f2 = dht_H2.readTemperature(true);
  
  float hif1 = dht_H1.computeHeatIndex(f1, h1);
  float hif2 = dht_H2.computeHeatIndex(f2, h2);
  float hic1 = dht_H1.computeHeatIndex(t1, h1, false);
  float hic2 = dht_H2.computeHeatIndex(t2, h2, false);
  
  if (isnan(h1) || isnan(t1) || isnan(f1)) {
    Serial.println("Error obteniendo los datos del sensor DHT11 H1");
    return;
  }

   if (isnan(h2) || isnan(t2) || isnan(f2)) {
    Serial.println("Error obteniendo los datos del sensor DHT11 H2");
    return;
  }
 
  Serial.print(F("H1: "));
  Serial.print(h1);
  Serial.print(F("; "));
  
  Serial.print(F("H2: "));
  Serial.print(h2);
  Serial.print(F("; "));
  
  Serial.print(F("T1: "));
  Serial.print(t1);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
   Serial.print(F("T2: "));
  Serial.print(t2);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
   Serial.print(F("Q1: "));
  Serial.print(hic1);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
  Serial.print(F("Q2: "));
  Serial.print(hic2);
  Serial.print(F(" C "));
  Serial.print(F("; "));
  
  digitalClockDisplay();
   delay(200);
 }


/************************************************************** 
*****************   Calc temperatura TE por analogic 13 ====>>>> 
***************************************************************/
  void All_Other_Sensors() 
   {

// quitada    double Irms = emon1.calcIrms(1480);  // Calculate Irms only current sensor
   // float TV = getTempva(); //SENS TEMP VA
     
     Temp_Heat_1300=thermocouple1.readCelsius();//TERMOCUPLA
     Temp_Sec_Air=thermocouple2.readCelsius();//TERMOCUPLA 2
     
     Ta = 0;
     Tc = 0;
     Te = 0;
     Tf = 0;
     Ttub=0;  //INPUTS MCP9700
     T_IAH = 0;
    

  delay(500);

  float h1 = dht_H1.readHumidity();
  float h2 = dht_H2.readHumidity();
  float t1 = dht_H1.readTemperature();
  float t2 = dht_H2.readTemperature();

   float f1 = dht_H1.readTemperature(true);
   float f2 = dht_H2.readTemperature(true);
   float hif1 = dht_H1.computeHeatIndex(f1, h1);
   float hif2 = dht_H2.computeHeatIndex(f2, h2);
   
  float hic1 = dht_H1.computeHeatIndex(t1, h1, false);
  float hic2 = dht_H2.computeHeatIndex(t2, h2, false);

      
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TA = analogRead(A9);   
    Ta =  Ta + ( ( (TensionAnalog_TA /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++){
    TensionAnalog_TC = analogRead(A11);   
    Tc =  Tc + ( ( (TensionAnalog_TC /204.8)  -0.5) / 0.01);
    delay(15);
    }
  
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TE = analogRead(A12);   
    Te =  Te + ( ( (TensionAnalog_TE /204.8)  -0.5) / 0.01);
    delay(15);
   }
      
     for (int i=1;i<=5;i++)
    {
    TensionAnalog_TF = analogRead(A13);   
    Tf =  Tf + ( ( (TensionAnalog_TF /204.8)  -0.5) / 0.01);
    delay(15);
    }

      for (int i=1;i<=5;i++){
    TensionAnalog_Ttub = analogRead(A14);   
    Ttub =  Ttub + ( ( (TensionAnalog_Ttub /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TIAH = analogRead(A15);   
    T_IAH =  T_IAH + ( ( (TensionAnalog_TIAH /204.8)  -0.5) / 0.01);
    delay(15);
    }
      
      TA = Ta/5; //compute the average value
      TC = Tc/5; //compute the average value
      TE = Te/5; //compute the average value
      TF = Tf/5; //compute the average value
      TTUB = Ttub/5; //compute the average value
      T_IAH = T_IAH/5; //compute the average value
      
   Serial.print(F("T_TA: "));
   Serial.print(TA);
   Serial.print(F(" C; "));
   delay(50);
    
   Serial.print(F("T_TC: "));
   Serial.print(TC);
   Serial.print(F(" C; "));
   delay(50);
   
   Serial.print(F("T_TE: "));
   Serial.print(TE);
   Serial.print(F(" C; "));
   delay(50);

   Serial.print(F("T_TF: "));
   Serial.print(TF);
   Serial.print(F(" C; "));
   delay(50);
   
//   Serial.print("T_VA: ");
//   Serial.print(TV);
//   Serial.print(" C; ");
//   delay(100);
   
   Serial.print(F("T_H1300: "));
   Serial.print(Temp_Heat_1300);
   Serial.print(F(" C; "));
   delay(100);

   Serial.print(F("T_OAH: "));
   Serial.print(Temp_Sec_Air);
   Serial.print(F(" C; "));
   delay(100);

   Serial.print(F("T_IAH: "));
   Serial.print(T_IAH);
   Serial.print(F(" C; "));
   delay(100);
   
   Serial.print(F("T_Tub: "));
   Serial.print(TTUB);
   Serial.print(F(" C; "));
   delay(100);
  
  Serial.print(F("H1:"));
  Serial.print(h1);
  Serial.print(F("; "));
  delay(50);
  
  Serial.print(F("H2:"));
  Serial.print(h2);
  Serial.print(F("; "));
  delay(50);

  Serial.print(F("DHT_T1: "));
  Serial.print(t1);
  Serial.print(F(" C "));
  delay(50);
  Serial.print(F("DHT_T2: "));
  Serial.print(t2);
  Serial.print(F(" C "));
  delay(50);

  Serial.print(F("Q1: "));
  Serial.print(hic1);
  Serial.print(F(" C "));
  delay(50);

  Serial.print(F("Q2: "));
  Serial.print(hic2);
  Serial.print(F(" C "));

  digitalClockDisplay();
   delay(200);
  }


// */************************************************************** 
//*****************   Calc temperature T_IAH por analogic A15 ====>>>> 
//***************************************************************/
//  /* // read the value, TensionAnalog = analogRead(A6);    
//   ******** T_IAH::>>>SENSOR INPUT AIR FLOW HAIR DRYER******************/
  void Temperatura_T_IAH() 
    {
    T_IAH = 0;
    for (int i=1;i<=5;i++){
    TensionAnalog_TIAH = analogRead(A6);   
    T_IAH =  T_IAH + ( ( (TensionAnalog_TIAH /204.8)  -0.5) / 0.01);
    delay(15);
  }
    T_IAH = T_IAH/5; //compute the average value
   Serial.print(F("T_IAH: "));
   Serial.print(T_IAH);
   Serial.print(F(" C; "));
   digitalClockDisplay();
   delay(200);
    }

/*************************************************************** 
*****************   Calc temperatura TF por analogic A7 ====>>>> 
//***************************************************************/
 /******** TC:::>>>SENSOR TEMP TUB ******************/
    void Temperatura_Ttub() 
   {
        Ttub = 0;
        for (int i=1;i<=5;i++){
        TensionAnalog_Ttub = analogRead(A7);   
        Ttub =  Ttub + ( ( (TensionAnalog_Ttub /204.8)  -0.5) / 0.01);
        delay(15);
      }
        TTUB = Ttub/5; //compute the average value
       Serial.print(F("Ttub: "));
       Serial.print(Ttub);
       Serial.print(F(" C; "));
        digitalClockDisplay();
       delay(200);
   }

/************************************************************** 
*****************   Calc temperatura TF por analogic A9 ====>>>> 
***************************************************************/
/*****read the value, TensionAnalog = analogRead(A19);    
      /******** TC:::>>>SENSOR TF******************/
    void Temperatura_TF() 
   {
        Td = 0;
        for (int i=1;i<=5;i++){
        TensionAnalog_TF = analogRead(A9);   
        Tf =  Tf + ( ( (TensionAnalog_TF /204.8)  -0.5) / 0.01);
        delay(15);
      }
        TF = Tf/5; //compute the average value
       Serial.print(F("T_Tf: "));
       Serial.print(TF);
       Serial.print(F(" C; "));
        digitalClockDisplay();
       delay(200);
    }






/*************************************************************** 
********   Calc temp DIGITAL MAX 6675 RES HIGH POWER ;====>>>> 
***************************************************************/
 /******** Temp_Heat_1300:::>>>TERMOCUPLA RESISTENCE 1300W**/
/************* RES::48 OHM   VOLT:220 VAC***/
    void Temperatura_Heat_1300() 
   {
        //termocupla res 1300 watt
       Temp_Heat_1300=thermocouple1.readCelsius();
       Serial.print(F("T_H1300: "));
       Serial.print(Temp_Heat_1300);
       Serial.print(F(" C; "));
       digitalClockDisplay();
       delay(1000);
    }

/************************************************************** 
********   Calc temp DIGITAL MAX 6675 AIR HAIR====>>>> 
***************************************************************/
   /******** Temp_SEC_AIR:::>>>TERMOCUPLA SECCAP 2200 W
   ******** RES::   VOLT:220 VAC***/
    void Temperatura_Sec_Air() 
   {
    //termocuple res 1300 watt
   //Temp_Sec_Air=thermocouple2.readCelsius();
   Serial.print(F("T_OAH: "));
   Serial.print(thermocouple2.readCelsius());
   Serial.print(F(" C; "));
   digitalClockDisplay();
   delay(1000);
    }

/************************************************************** 
*****************   Calc temperatura TE por analogic 13 ====>>>> 
***************************************************************/
  void All_Other_Sensors2() 
   {
   // float TV = getTempva(); //SENS TEMP-VA
    Temp_Heat_1300=thermocouple1.readCelsius();//TERMOCUPLA
     Temp_Sec_Air=thermocouple2.readCelsius();//TERMOCUPLA 2
     Ta, Tc, Te,Tf, Ttub, T_IAH = 0;//INPUTS MCP9700
   
 // Esperamos 5 segundos entre medidas
 // delay(5000); cambiado 200720918
   delay(500);
  float h1 = dht_H1.readHumidity();
  float h2 = dht_H2.readHumidity();
  float t1 = dht_H1.readTemperature();
  float t2 = dht_H2.readTemperature();
  float f1 = dht_H1.readTemperature(true);
  float f2 = dht_H2.readTemperature(true);
  float hif1 = dht_H1.computeHeatIndex(f1, h1);
  float hif2 = dht_H2.computeHeatIndex(f2, h2);
  float hic1 = dht_H1.computeHeatIndex(t1, h1, false);
  float hic2 = dht_H2.computeHeatIndex(t2, h2, false);

      
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TA = analogRead(A14);   
    Ta =  Ta + ( ( (TensionAnalog_TA /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++){
    TensionAnalog_TC = analogRead(A12);   
    Tc =  Tc + ( ( (TensionAnalog_TC /204.8)  -0.5) / 0.01);
    delay(15);
    }
  
    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TE = analogRead(A10);   
    Te =  Te + ( ( (TensionAnalog_TE /204.8)  -0.5) / 0.01);
    delay(15);
   }
      
     for (int i=1;i<=5;i++)
    {
    TensionAnalog_TF = analogRead(A9);   
    Tf =  Tf + ( ( (TensionAnalog_TF /204.8)  -0.5) / 0.01);
    delay(15);
    }

      for (int i=1;i<=5;i++){
    TensionAnalog_Ttub = analogRead(A7);   
    Ttub =  Ttub + ( ( (TensionAnalog_Ttub /204.8)  -0.5) / 0.01);
    delay(15);
    }

    for (int i=1;i<=5;i++)
    {
    TensionAnalog_TIAH = analogRead(A6);   
    T_IAH =  T_IAH + ( ( (TensionAnalog_TIAH /204.8)  -0.5) / 0.01);
    delay(15);
    }
      
      TA = Ta/5; //compute the average value
      TC = Tc/5; //compute the average value
      TE = Te/5; //compute the average value
      TF = Tf/5; //compute the average value
      TTUB = Ttub/5; //compute the average value
      T_IAH = T_IAH/5; //compute the average value
      
   Serial.print(F("T_TA: "));
   Serial.print(TA);
   Serial.print(F(" C; "));
   delay(50);

   //Ambient temperature inside the container
    
  // Serial.print("T_TC: ");
   Serial.print(F("T_AmCon: "));
   Serial.print(TC);
   Serial.print(F(" C; "));
   delay(50);
   
//   Serial.print("T_TE: ");
//   Serial.print(TE);
//   Serial.print(" C; ");
//   delay(50);
//
//   Serial.print("T_TF: ");
//   Serial.print(TF);
//   Serial.print(" C; ");
//   delay(50);
   
//   Serial.print("T_VA: ");
//   Serial.print(TV);
//   Serial.print(" C; ");
   delay(100);
   
   Serial.print(F("T_H1300: "));
   Serial.print(Temp_Heat_1300);
   Serial.print(F(" C; "));
   delay(100);

   Serial.print(F("T_OAH: "));
   Serial.print(Temp_Sec_Air);
   Serial.print(F(" C; "));
   delay(100);

   Serial.print(F("T_IAH: "));
   Serial.print(T_IAH);
   Serial.print(F(" C; "));
   delay(100);
   
   Serial.print(F("T_Tub: "));
   Serial.print(TTUB);
   Serial.print(F(" C; "));
   delay(100);

//   Serial.print(F("T_ICAH: ");
//   Serial.print(TV);
//   Serial.print(F(" C; ");
//   delay(100);
  
//  Serial.print(F("Ap:Po_R130 "); //currente
//  Serial.print(Irms*230.0);  // Apparent power
//  Serial.print(F("; ");
//  
//  Serial.print(F("Curr_R130 "); //currente
//  Serial.print(Irms);      // Irms
//  Serial.print(F("; ");
   
  Serial.print(F("H1_Int:"));
  Serial.print(h1);
  Serial.print(F("; "));
  delay(50);
  
  Serial.print(F("DHT_T1_Int: "));
  Serial.print(t1);
  Serial.print(F(" C "));
  delay(50);
  Serial.print(F("Q1_Int: "));
  Serial.print(hic1);
  Serial.print(F(" C "));
  delay(50);
  
  Serial.print(F("H2_Ext_Int:"));
  Serial.print(h2);
  Serial.print(F("; "));
  delay(50);
  
  Serial.print(F("DHT_T2_Ext: "));
  Serial.print(t2);
  Serial.print(F(" C "));
  delay(50);

   Serial.print(F("Q2_Ext: "));
  Serial.print(hic2);
  Serial.print(F(" C "));

  digitalClockDisplay();
   delay(200);
  }


  
void read_H1_TA()
 {
   Ta = 0;
                float h =  dht_H1.readHumidity();
                float t = dht_H1.readTemperature();
                float f = dht_H1.readTemperature(true);
                float hif = dht_H1.computeHeatIndex(f, h);
                float hic = dht_H1.computeHeatIndex(t, h, false);
              
              for (int i=1;i<=5;i++)
                      {
                      TensionAnalog_TA = analogRead(A14);   
                      Ta =  Ta + ( ( (TensionAnalog_TA /204.8)  -0.5) / 0.01);
                      delay(100);
                      }
                      TA = Ta/5; //compute the average value
                  if (isnan(h) || isnan(t) || isnan(f)) 
                {
                    Serial.println(F("ERROR SENSOR DHT11 IS BROKEN"));
                  return;
                }
             Serial.print(F("T_Ta: "));
             Serial.print(TA);
             Serial.print(F(" C; "));
             delay(100);
            
            Serial.print(F("H1: "));
            Serial.print(h);
            Serial.print(F("; "));
            delay(100);
            
            Serial.print(F("TH1: "));
            Serial.print(t);
            delay(100);
            Serial.print(F(" C "));
            
            Serial.print(F("Q1: "));
            Serial.print(hic);
            Serial.print(F(" C "));
            digitalClockDisplay();
            delay(100);
  }

/************************************************************** 
******** CASE a====>>>> a -> HM0 Humidity MIN 0 %  
* copied case CASE C====>>>> AHF AIR HOMOGENIZING FAN
***************************************************************/
void H100_H0_Humidity()
 {
              //termocupla res 1300 watt
              Temp_Heat_1300=thermocouple1.readCelsius();
              Temp_Sec_Air=thermocouple2.readCelsius();//TERMOCUPLA 2
     
             digitalWrite(Pin_Fan_Heater, HIGH);
             digitalWrite(Pin_Heater, HIGH);
              All_Other_Sensors();
                     
             if (Temp_Heat_1300< 200)
             {
               Heat_R1300();
             //    Serial.print("TR<200 "); 
                // All_Other_Sensors();
             } 
             else
                {
                  Heat_R1300_Off();
                     // Serial.print("TR>200"); 
                     // All_Other_Sensors();
                } 
               // Secador_Test();
               delay(50);
                //All_Other_Sensors();
       
  }

//float getTempva(){
//  //returns the temperature from one DS18B20 in DEG Celsius
//  byte data[12];
//  byte addr[8];
// 
//  if ( !ds.search(addr)) {
//      //no more sensors on chain, reset search
//      ds.reset_search();
//      return -1000;
//  }
// 
//  if ( OneWire::crc8( addr, 7) != addr[7]) {
//      Serial.println("CRC is not valid!");
//      return -1000;
//  }
// 
//  if ( addr[0] != 0x10 && addr[0] != 0x28) {
//      Serial.print("Device is not recognized");
//      return -1000;
//  }
// 
//  ds.reset();
//  ds.select(addr);
//  ds.write(0x44,1); // start conversion, with parasite power on at the end
// 
//  byte present = ds.reset();
//  ds.select(addr);    
//  ds.write(0xBE); // Read Scratchpad
// 
//  for (int i = 0; i < 9; i++) { // we need 9 bytes
//    data[i] = ds.read();
//  }
// 
//  ds.reset_search();
// 
//  byte MSB = data[1];
//  byte LSB = data[0];
// 
//  float tempRead = ((MSB << 8) | LSB); //using two's compliment
//  float TemperatureSum = tempRead / 16;
//   return TemperatureSum;
//}


/************************************************************** 
******** CASE b====>>>> b -> HM100 Humidity MAX 100 %  
* copied case CASE C====>>>> AHF AIR HOMOGENIZING FAN
***************************************************************/
void Heat_R1300()
 {
               // procedure to slowly raise the temperature
                          digitalWrite(Pin_Heat_R1300, HIGH);
                          delay(200);
                         digitalWrite(Pin_Heat_R1300, LOW);
                          delay(100);
                          digitalWrite(Pin_Heat_R1300, HIGH);
                          delay(100);
                         digitalWrite(Pin_Heat_R1300, LOW);
                          delay(100);
//                          digitalWrite(Pin_Heat_R1300, HIGH);
//                          delay(100);
                         digitalWrite(Pin_Heat_R1300, LOW);
                          delay(100);
}


/************************************************************** 
******** CASE b====>>>> b -> HM100 HUMEDITY MAX 100 %  
* copied case CASE C====>>>> AHF AIR HOMOGENIZING FAN
***************************************************************/
void Heat_R1300_fast()
 {
    //procedure to quickly raise the temperature
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(600);
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(6000);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(100);
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(400);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(100);
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(6000);
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(6000);
}


/************************************************************** 
******** CASE b====>>>> b -> HM100 Humidity MAX 100 %  
* copied case CASE C====>>>> AHF AIR HOMOGENIZING FAN
***************************************************************/
void Heat_R1300_Off()
 {
   //procedure to slowly raise the temperature
    digitalWrite(Pin_Heat_R1300, HIGH);
    delay(166);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(500);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(166);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(500);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(166);
    digitalWrite(Pin_Heat_R1300, LOW);
    delay(500);
}

/**********************************************
 ***    PROGRAM  CURRENT SENSOR **************
 *********************************************/
void Current_Sensor()
{
//  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
//  Serial.print(Irms*230.0);         // Apparent power
//  Serial.print(" ");
//  Serial.println(Irms);          // Irms
}   
/** Lo sketch usa 33586 byte (13%) dello spazio disponibile per i programmi. Il massimo è 253952 byte.
Le variabili globali usano 1756 byte (21%) di memoria dinamica, lasciando altri 6436 byte liberi per le variabili locali. Il massimo è 8192 byte.
**/
 
