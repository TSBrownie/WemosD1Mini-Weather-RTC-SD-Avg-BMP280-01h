//Arduino Wemos D1R1 mini w/ RTC, SD Shield (ESP8266 Boards 2.5.2)
//RTC, SD File: Absolute Barometric Pressure (SL & Comp), Temp, no humidity
//Wemos SDA(D2)-->BMP280 pin=SDA, Wemos SCL(D1)-->BMP280=SCL
//Wemos 3v3-->BMP280 Vdd, Wemos GND-->BMP280 GND
//Wemos Vdd or GND, BMP280=SDO (Address control) NC
//Wemos Vdd or GND, BMP280=CSB (I2C select, GND=SPI) NC
//BMP280 Data is LSBMSB ijklmnopabcdefgh order (not abcdefghijklmnop)
//WeMos Micro SD Shield uses HSPI(12-15) not (5-8), 3V3, G:
//GPIO12(D6)=MISO (main in, secondary out); GPIO13(D7)=MOSI (main out, secondary in) 
//GPIO14(D5)=CLK (Clock); GPIO15(D8)=CS (Chip select)
//SD library-->8.3 filenames (ABCDEFGH.txt = abcdefgh.txt)
//RTC DS1307. I2C--> SCL(clk)=D1, SDA(data)=D2 (Shared with BMP280)
//20210806 - TSBrownie.  Non-commercial use.

#include<Wire.h>                   //Wire library I2C
#include <SD.h>                    //SD card library
#include <SPI.h>                   //Serial Peripheral Interface bus lib for COMM, SD com
#define BMP280Addr 0x76            //BMP280 I2C 0x76(108), or 0x77
#define DS1307 0x68                //I2C Addr of RTC1307 (Default=0x68 Hex)
#define S_prt Serial.print         //Short name for Serial.print
#define S_pln Serial.println       //Short name for Serial.println
File dataFile;                     //Link to file name

byte second, minute, hour, DoW, Date, month, year;   //Btye variables for BCD time
String DoWList[]={"Null",",Sun,",",Mon,",",Tue,",",Wed,",",Thr,",",Fri,",",Sat,"}; //DOW from 1-7
String SDData;                     //Build data to write to SD "disk"
String timeString;                 //Build date time data
String FName = "20220122.txt";     //USER SUPPLIED: SD card file name (ANSI encoding is best)

unsigned int time2reading = 10000; //USER SUPPLIED: Milliseconds between readings
double calib = 0;                  //USER SUPPLIED: Clock calibration
double TempCal = -5;               //USER SUPPLIED: Temp C calibration factor
double Dh = 6.0;                   //USER SUPPLIED: Device height above sea level m
double P, p0;                      //P Uncomp Press at device alt, p0 Sea Level
double cTempLast = 0;              //Keep last temp for rising, falling, steady
double pLast = 0;                  //Keep last pressure for rising, falling, steady
unsigned int b1[24];               //16 bit array 0-24 (2 bytes)
unsigned int data[8];              //16 bit Data array 0-8
const unsigned int avgPTsz = 20;   //Size of avtPT array
double avgPT[2][avgPTsz+1];        //Store past press & temp for averages
unsigned int avgPTindx = 0;        //Index to avgPT
int i = 0;                         //for(index....)
bool avgFlag = false;              //Suppress avg press until enough samples

int Compare(float f1, float f2, int ex1){                   //Compare(F1, F2, # decimal places)
  if (round(f1 * pow(10,ex1)) > (round(f2 * pow(10,ex1)))){ //Compare ex1 decimals           
     return(1);}
     else if (round(f1*pow(10,ex1)) < (round(f2*pow(10,ex1)))){  //Compare ex1 decimals
       return(-1);}
       else return(0);
}

void avgPTCalc(){                  //Calc all avgs
  avgPT[0][0] = 0;                 //Init Press accumulator
  avgPT[1][0] = 0;                 //Init Temp accumulator
  for(i=1; i<=avgPTsz; i++){       //Loop thru avg readings
    avgPT[0][0] += avgPT[0][i];    //Add to accumulator for P
    avgPT[1][0] += avgPT[1][i];    //Add to accumulator for T
  }
  avgPT[0][0] = avgPT[0][0]/avgPTsz;  //Put average P at loc 0
  avgPT[1][0] = avgPT[1][0]/avgPTsz;  //Put average T at loc 0
}

//RTC FUNCTIONS =====================================
byte BCD2DEC(byte val){            //Ex: 51 = 01010001 BCD. 01010001/16-->0101=5 then x10-->50.  
  return(((val/16)*10)+(val%16));} //         01010001%16-->0001. 50+0001 = 51 DEC

void GetRTCTime(){                               //Routine read real time clock, format data
  byte second;byte minute;byte hour;byte DoW;byte Date;byte month;byte year;
  Wire.beginTransmission(DS1307);                //Open I2C to RTC DS1307
  Wire.write(0x00);                              //Write reg pointer to 0x00 Hex
  Wire.endTransmission();                        //End xmit to I2C.  Send requested data.
  Wire.requestFrom(DS1307, 7);                   //Get 7 bytes from RTC buffer
  second = BCD2DEC(Wire.read() & 0x7f);          //Seconds.  Remove hi order bit
  minute = BCD2DEC(Wire.read());                 //Minutes
  hour = BCD2DEC(Wire.read() & 0x3f);            //Hour.  Remove 2 hi order bits
  DoW = BCD2DEC(Wire.read());                    //Day of week
  Date = BCD2DEC(Wire.read());                   //Date
  month = BCD2DEC(Wire.read());                  //Month
  year = BCD2DEC(Wire.read());                   //Year
  timeString = 2000+year;                        //Build Date-Time data to write to SD
  if (month<10){timeString = timeString + '0';}  //Pad leading 0 if needed
  timeString = timeString + month;               //Month (1-12)  
  if(Date<10){timeString = timeString + '0';}    //Pad leading 0 if needed
  timeString = timeString + Date;                //Date (1-30)
  timeString = timeString + DoWList[DoW];        //1Sun-7Sat (0=null)
  if (hour<10){timeString = timeString + '0';}   //Pad leading 0 if needed
  timeString = timeString + hour + ':';          //HH (0-24)
  if (minute<10){timeString = timeString + '0';} //Pad leading 0 if needed
  timeString = timeString + minute + ':';        //MM (0-60)
  if (second<10){timeString = timeString + '0';} //Pad leading 0 if needed
  timeString = timeString + second;              //SS (0-60)
}

//SD CARD FUNCTIONS =================================
void openSD() {                          //Routine to open SD card
  S_pln(); S_pln("Open SD card");        //User message
  if (!SD.begin(15)) {                   //If not open, print message.
    S_pln("Open SD card failed");
    return;}
  S_pln("SD Card open");
}

char openFile(char RW) {                 //Open SD file.  Only 1 at a time.
  dataFile.close();                      //Ensure file status, before re-opening
  dataFile = SD.open(FName, RW);}        //Open Read at BOF. Open write/append at EOF.

String print2File(String tmp1) {         //Print data to SD file
  openFile(FILE_WRITE);                  //Open user SD file for write
  if (dataFile) {                        //If file there & opened --> write
    dataFile.println(tmp1);              //Print string to file
    dataFile.close();                    //Close file, flush buffer (reliable but slower)
  } else {S_pln("Error opening file for write");}   //File didn't open
}

void getRecordFile() {                   //Read from SD file
  if (dataFile) {                        //If file is there
    Serial.write(dataFile.read());       //Read data, then write to COM
  } else {S_pln("Error opening file for read");}  //File didn't open
}

void getdata() {                         //Get file data
  openFile(FILE_READ);                   //Open SD card file
  int SDfileSz = dataFile.size();        //Get file size
  S_prt("SDfileSz: ");  S_pln(SDfileSz); //Print data file size
  dataFile.close();                      //Close SD file
  delay(100);
}
//End SD Card FUNCTIONS =================================

void setup(){                            //SETUP()
  Wire.begin();                          //Init I2C com
  Serial.begin(9600);                    //Init Serial com
  delay(3000);                           //Allow serial to come online
  openSD();                              //Call open SD card routine
  GetRTCTime();                          //Get time from real time clock
  SDData = String("RecType,DeviceAlt,DateTime,DevicehPa,SeaLevel-hPa,TempC"); //File Header
  print2File(SDData);                    //Write string to SD file
  SDData = "C,"+timeString+calib;         //Prepare calibration string
  print2File(SDData);                    //Write string to SD file
}

void loop(){                             //LOOP()
  for (int i = 0; i < 24; i++)  {        //Get data 1 reg (2 bytes, 16 bits) at a time
    Wire.beginTransmission(BMP280Addr);  //Begin I2C to BMx280
    Wire.write((136 + i));               //Data register 136-159 (0x88-0x9F)
    Wire.endTransmission();              //End I2C Transmission
    Wire.requestFrom(BMP280Addr, 1);     //Get BMx280 1 byte data in LSBMSB order
    if (Wire.available() == 1) {         //If device available
      b1[i] = Wire.read();               //Read reg, store in b1[0-23]
    }
  }
  
  //Temperature coefficients (byte1: LSB<>MSB, bitwise & 11111111; byte 2 same+shift)
  unsigned int dig_T1=(b1[0] & 0xFF)+((b1[1] & 0xFF)*256); //0x88/0x89 
  int dig_T2 = b1[2] + (b1[3] * 256);    //0x8A/0x8B
  int dig_T3 = b1[4] + (b1[5] * 256);    //0x8C/0x8D

  //Pressure coefficients (byte1: LSB<>MSB, bitwise & 11111111; byte 2 same+shift)
  unsigned int dig_P1=(b1[6] & 0xFF)+((b1[7] & 0xFF)*256);//0x8E/0x8F
  int dig_P2 = b1[8] + (b1[9] * 256);    //0x90/0x91
  int dig_P3 = b1[10] + (b1[11] * 256);  //0x92/0x93
  int dig_P4 = b1[12] + (b1[13] * 256);  //0x94/0x95
  int dig_P5 = b1[14] + (b1[15] * 256);  //0x96/0x97
  int dig_P6 = b1[16] + (b1[17] * 256);  //0x98/0x99
  int dig_P7 = b1[18] + (b1[19] * 256);  //0x9A/0x9B
  int dig_P8 = b1[20] + (b1[21] * 256);  //0x9C/0x9D
  int dig_P9 = b1[22] + (b1[23] * 256);  //0x9E/0x9F

  Wire.beginTransmission(BMP280Addr);    //Start I2C Transmission BMx280
  Wire.write(0xF4);                      //Select control measurement register
  Wire.write(0x27);                      //Set normal mode, temp & press over-sampling rate = 1
  Wire.endTransmission();                //End I2C Transmission
  Wire.beginTransmission(BMP280Addr);    //Start I2C Transmission BMx280
  Wire.write(0xF5);                      //Select Config register
  Wire.write(0xA0);                      //Set stand_by time = 1000ms
  Wire.endTransmission();                //End I2C Transmission to get data
  for (int i = 0; i < 8; i++){           //Collect temp & press data
    Wire.beginTransmission(BMP280Addr);  //Start I2C Transmission BMx280
    Wire.write((247 + i));               //Select data register
    Wire.endTransmission();              //End I2C Transmission
    Wire.requestFrom(BMP280Addr, 1);     //Request 1 byte of data from BMx280
    if (Wire.available() == 1){          //If data
      data[i] = Wire.read();             //Read & store 1 byte of data
    }
  }

  //Convert pressure & temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

  //Temperature offset calculations (per Bosch)
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);              //
  double cTemp = ((var1 + var2) / 5120.0) + TempCal;//Calc C, add calibration factor
  double fTemp = cTemp * 1.8 + 32;                  //Convert C to F

  //Pressure offset calculations (per Bosch)
  var1 = ((double)t_fine / 2.0) - 64000.0;          //
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;  //
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;      //
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0); //
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1); //
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;        //
  var1 = ((double) dig_P9) * p * p / 2147483648.0;  //
  var2 = p * ((double) dig_P8) / 32768.0;           //
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100; //hPa

  //Output To Serial Monitor
  p0 = ((pressure/100) * pow(1 - (0.0065 * Dh / (cTemp + 0.0065 * Dh + 273.15)), -5.257));
  avgPTCalc();                                      //Calculate averages
  S_pln();S_prt("============== ");S_pln(timeString); 
  S_prt("Device Altitude (abs): ");S_prt(Dh);S_pln(" meters above sea level");
  S_prt("Temperature: ");S_prt(cTemp);S_prt("°C; ");
  S_prt(fTemp);S_prt("°F  ");
  if (Compare(cTemp,avgPT[1][0],1) == 1){           //Compare(F1,F2,# decimal places compared)
    S_pln("Rising");}                               //If temp is rising from prior
    else if(Compare(cTemp,avgPT[1][0],1) == -1){      
      S_pln("Falling");}                            //If temp is falling from prior 
      else {S_pln("Steady");}                       //Else steady
  
  S_prt("Barometric Pressure-Absolute: ");
  if (Compare(pressure,avgPT[0][0],1) == 1){        //Compare(F1, F2, # decimal places compared)
    if(avgFlag){S_prt("Rising - Fair Weather");}}   //If press is rising from prior
    else if(Compare(pressure,avgPT[0][0],1) == -1){
      if(avgFlag){S_prt("Falling - Stormy");}}      //If press is falling from prior
      else {if(avgFlag){S_prt("Steady - No Change");}}  //Else steady
      
  if(avgPTindx >= avgPTsz){                         //Rolling index
    avgPTindx = 1;                                  //If too big-->reset to 1
    avgFlag = true;}                                //Display average press
    else {avgPTindx++;}                             //Increment index
  avgPT[0][avgPTindx] = pressure;                   //Save press to avg
  avgPT[1][avgPTindx] = cTemp;                      //Save temp to avg
  
  S_pln();S_prt("  ");S_prt(pressure, 3);S_prt(" hPa(mbar); "); //Print Pressure in Pa mbar
  S_prt(pressure * 100, 3);S_pln(" Pa");            //Print Pressure in hPa
  if(avgFlag){  
    S_prt("  ");S_prt(avgPT[0][0], 3);S_pln(" hPa, Short Term Past Average ");}//Avg Pressure
  S_prt("  ");S_prt(pressure * 0.750061683, 3);
  S_pln(" mmHg");                                   //Print Pressure in mmHg
  S_prt("  ");S_prt(pressure * 0.000986923, 6);     //Print Pressure in atm
  S_pln(" Atm; ");

  S_pln("Barometric Pressure-Comp'ed To Sealevel");
  S_prt("  ");S_prt(p0 * 100, 3);S_prt(" hPa(mbar); "); //Comp'ed Pressure in Pa,mbar
  S_prt(p0 * 10000, 3);S_pln(" Pa");                //Comp'ed Pressure in hPa
  S_prt("  ");
  S_prt(p0 * 075.0061683, 3);S_pln(" mmHg");        //Comp'ed Pressure in mmHg
  S_prt("  ");S_prt(p0 * 000.0986923, 6);           //Comp'ed Pressure in Atm
  S_pln(" Atm; ");

  GetRTCTime();                                    //Get time from RTC
  SDData = "D,"+String(Dh)+','+timeString+','+
    String(pressure)+','+String(p0*100)+','+String(cTemp);  //SD data
  print2File(SDData);                              //Write data to SD file
  delay(time2reading);                             //Delay until next reading
}
