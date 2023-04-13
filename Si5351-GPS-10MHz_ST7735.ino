// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Datei Si5351-GPS-10MHz_m2.ino   vom 13.04.2023
// Fork von  hier: https://github.com/flafforgue/Si5351-GPS-10MHz
// GPS Uhr mit Si5351 10 MHz Ausgang
// Die Uhr laeuft auch ohne GPS weiter
// Weil kein ST7789 Display vorhanden, hier eine Version mit ST7735 Display & Treiber
// TFT-Display ok, Schrift angepasst
// Dallas DS18B20 Temperatursensor ok
// Keine Reaktion auf Encoder und Button implementiert
// LED nicht angeschlossen und nicht benutzt
// 
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//  0    : GPS    - Tx ( alternate )
//  1    :      
//  2    : GPS    - 1 PPS GPS Modul     
//  3    : Rotary A   Rotary Clk
//  4    : Rotary B   Rotary Dat
//  5    : Si5351 - Clk0 2.5 MHz          - FIn
//  6    : Tft    - cs
//  7    : Tft    - DC  (A0)
//  8    : GPS    - Tx  ( Base )    GPS Modul SoftwareSerial
//  9    : GPS    - Rx  ( Not connected ) GPS Modul SoftwareSerial
// 10    :
// 11    : Tft    -  SDA
// 12    :
// 13    : Tft    -  SCL
// 14 A0 :
// 15 A1 :
// 16 A2 :        - Led  ( nicht benutzt )
// 17 A3 : Dalas  - DS18B20
// 18 A4 : Si5351 - SDA
// 19 A5 : Si5351 - SCL
// 20 A6 : Rotary - Btn
// 21 A7 :

// ============================================================================
//                          TFT LCD 128 x 160 Screen 
// ============================================================================

#include <SPI.h>                      // Arduino SPI library
#include <Adafruit_GFX.h>             // Core graphics library
// #include <Adafruit_ST7789.h>       // Hardware-specific library for ST7789  - original
#include <Adafruit_ST7735.h>          // Hardware-specific library for ST7735 

static const int  cs =  6;            // TFT cs
static const int  dc =  7;            // TFT DC
static const int rst = -1;            // TFT RST
bool GPSok = 0;                       // 1, wenn GPS empfangen wird

// Initialize Adafruit ST7789 TFT library   -cs -dc -rst 

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);       // mein Display



// ============================================================================
//                                 ---   InitTFT   ---
// ============================================================================

void InitTft() {   
 //-  Serial.println(F("initialisiere TFT 'BLACKTAB'"));  
  tft.initR(INITR_BLACKTAB);                        // initialize a ST7735S chip, black tab  
  tft.invertDisplay(true);                          // (true) Invertiere die Farben des Displays bei Bedarf (yellow tab Display aus China)
  tft.setRotation(1);                               // (1) Ausrichtung des Displays 
  tft.fillScreen(ST77XX_BLACK);                     // Loesche TFT
  tft.setTextSize(1);                               // (1) Textgroesse
  tft.setTextWrap(false);                           // (false) Textumbruch    
  tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);     // Textfarbe , Hintergrund (wenn kein Hintergrund, dann transparent)
}

// ============================================================================
//                                 ---   GPS   ---
// ============================================================================

#include <TimeLib.h>                                // Arduino TimeLib Library by "*Time* by *Michael Margolis*"  einbinden
#include <TinyGPS++.h>                              // TinyGPSPlus Library einbinden
#include <SoftwareSerial.h>                         // SoftwareSerial Library einbinden

static const int RXPin = 8;                         // (8) RxPin fuer GPS Modul SoftwareSerial
static const int TXPin = 9;                         // (9) TxPin fuer GPS Modul SoftwareSerial - nicht benutzt
static const uint32_t GPSBaud = 9600;               // (9600) Standard Geschwindigkeit

TinyGPSPlus gps;                                    // Objekt gps erstellen
SoftwareSerial ss(RXPin, TXPin);                    // Objekt ss für SoftwareSerial erstellen

// ============================================================================
//                              ---   Smart Delay   ---
// ============================================================================

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}
// ----------------------------------------------------------------------------

byte oday = 0;                                      // Merker oldday; Anzeige, wenn der Tag sich geaendert hat
byte osec = 61;                                     // Merker oldsec; Anzeige, wenn die Sekunde sich geaendert hat

// ============================================================================
//                          ---   GPS Time Display   ---
// ============================================================================

void GPSTimeDisplay(){                                     
  if ( gps.date.isUpdated() ) {                     // Datum/Zeit uebernehmen
    int  Year   = gps.date.year();                  // Jahr uebernehmen
    byte Month  = gps.date.month();                 // Monat uebernehmen
    byte Day    = gps.date.day();                   // Tag uebernehmen
    byte Hour   = gps.time.hour();                  // Stunde uebernehmen
    byte Minute = gps.time.minute();                // Minute uebernehmen
    byte Second = gps.time.second();                // Sekunde uebernehmen

// ----------- Funktion EU-Sommerzeit eingefuegt -------------------------------
    byte tzHours=1;                                           // Zeitzone 0=UTC, 1=MEZ
    boolean st = sommertime_EU(Year,Month,Day,Hour,tzHours);  // Funktion aufrufen
    if (st) 
      { tzHours = 2 ; }                                       // UTC -2 (Berlin Sommerzeit)
   else
      { tzHours = 1 ; }                                       // UTC -1 (Berlin Winterzeit)
// -----------------------------------------------------------------------------
    setTime(Hour, Minute, Second, Day, Month, Year);          // Zeit der Softwareuhr setzen
    adjustTime( tzHours * SECS_PER_HOUR );                    // (tzHours) abhaening addieren (tzHours * 3600 = h)
 }
  
  char sz[32];                                                // Character Array
  if ( oday != day() ) {                                      // bei Aenderung Datum anzeigen
    tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);             // Schrift gelb, Hintergrund schwarz
    tft.setTextSize(2);                                       // (2) Textgroesse
    tft.setCursor(20,10);                                     // (20,10) setze Cursor
    sprintf(sz, "%02d.%02d.%02d", day(), month(), year() );   // Ausgabe formatiern
    tft.print(sz);                                            // formatierte Ausgabe auf TFT
    oday = day();
  }

  if ( osec != second() ) {                                   // bei Aenderung Uhrzeit anzeigen
    tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);             // Schrift gelb, Hintergrund schwarz
    tft.setTextSize(2);                                       // (2) Textgroesse   
    tft.setCursor(8,40);                                      // (8,40)  setze Cursor
    sprintf(sz,"%02d:%02d:%02d Uhr",hour(),minute(),second());   // Ausgabe formatiern
    tft.print(sz);                                            // formatierte Ausgabe auf TFT
    osec = second();                                          // Sekunde merken 
  }  
} 

// ============================================================================
//                      ---   Init GPS Software Serial   ---
// ============================================================================
 
void InitGps() {
  ss.begin(GPSBaud);  
}

// ============================================================================
//                 ---   Dalas D18B20 Temperature Sensor   ---
// ============================================================================

#include <OneWire.h>                              // OnWire Library einbinden
#include <DallasTemperature.h>                    // DallasTemperature Library einbinden
OneWire oneWire(A3);                              // (A3) Onewire Pin fuer Sensor festlegen
DallasTemperature sensors(&oneWire);              // DallasTemperature mit OneWire Pin starten
DeviceAddress Thermometer;                        // Objekt erstellen

void InitThermometer() {                          // Thermometer starten
 //- Serial.println(F("inititalisere Temp-Sensor"));
  sensors.begin();
  if (!sensors.getAddress(Thermometer, 0))
  {tft.setCursor(10,40);                          // Fehlermeldeung
    tft.print("Sensor Fehler!") ;                 // kein Sensor gefunden
  Serial.println(F("kein DS18B20 Sensor erkannt"));
  smartDelay(2000);
  tft.fillScreen(ST77XX_BLACK);                   // Loesche TFT  
  }  
  sensors.setResolution(Thermometer, 10);         // (9) Aufloesung festlegen 
                            
}

// ============================================================================
//                         ---   Rotary  Encoder  ---
// ============================================================================

#define encoder0PinA      3                     // (3)  Encoder CLK an Pin 3
#define encoder0PinB      4                     // (4)  Encoder DA an Pin 4
#define BtnEncoder       20                     // (20 A6) Encoder SW an Pin 20

//#define BTN_None          0                   // (0)  Pin 0  ??
// ------ neuer Versuch  ---------------------
byte BTN_None     = 0;                          // nicht gedrueckt

#define BTN_Encoder       1                     // (1)  Pin 1  ??
#define BTN_EncoderL      2                     // (2)  Pin 2  ??
#define BTN_LONGDELAY   800

int posencoder          = 0;                    // Zaehler fuer Endoder Position
int encodermov          = 0;                    // Merker Bewegung
unsigned long ntime     = 0;                    // Merker ntime
unsigned long otime     = 0;                    // Merker otime

byte keydown            = BTN_None;             // nicht gedrueckt
byte key                = BTN_None;             // nicht gedrueckt
unsigned long BTNTime;                          // Zeitdauer fuer Tastendruck

// ----------------------------------------------------------------------------

void doEncoder() {                              // Drehung feststellen und zaehlen
  ntime=millis();
  if ( ntime-otime > 60 ) {
    if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
      encodermov =+1;
      posencoder++;
    } else {
      encodermov =-1;
      posencoder--;
    }
    otime=ntime;
    // Serial.print(" EncPos "); Serial.println(posencoder);      //#### zum Debuggen
  }  
}

// ------------------- Button gedrueckt? ---------------------------------------------------------

void readBtnState() {                                 // Encoder Taster gedrueckt? -- laeuft nicht
  unsigned long NTime; 
  NTime=millis();
 
  if ( keydown == BTN_None ) { // no key waiting 
    if ( digitalRead (BtnEncoder)==LOW ) { BTNTime=NTime;  keydown=BTN_Encoder;  }
//    if ( digitalRead (BtnCh1    )==LOW ) { BTNTime=NTime;  keydown=BTN_Ch1;      }
//    if ( digitalRead (BtnCh2    )==LOW ) { BTNTime=NTime;  keydown=BTN_Ch2;      }    
  } else {                     // key allready down
    if ( NTime - BTNTime > 10 ) { // avoid rebounds
        switch (keydown) {
           case BTN_Encoder:
                 if ( digitalRead (BtnEncoder)==HIGH ) { // keypress on release ;)   
                   if ( NTime - BTNTime >= BTN_LONGDELAY )  key = BTN_EncoderL;                
                   else                                     key = BTN_Encoder;
                   keydown=BTN_None;
                 }
                 break;  
             Serial.print(F("Taster ")); Serial.println(key);     //#### zum Debuggen      
        }       
    }
  }
}

// ----------------------------------------------------------------------------

bool keypressed() {
  return ( key != BTN_None );
}

byte readkey() {
  byte tmp = key;
  key=BTN_None;
  return( tmp);
}

// ----------------------------------------------------------------------------

void InitRotary() {
  // encoder  
  // ----------
  pinMode(encoder0PinA, INPUT_PULLUP);                    // EingangsPin fuer Encoder-CLK , aktiviere Pullup
  pinMode(encoder0PinB, INPUT_PULLUP);                    // EingangsPin fuer Encoder-DAT, aktiviere Pullup
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder, CHANGE);    // aktiviere Interrupt Change
  
  // Boutons 
  // ----------
  pinMode(BtnEncoder  , INPUT_PULLUP);                    // EingangsPin fuer Taster des Encoders, aktiviere Pullup
}

// ============================================================================
//                             ---   Frequency   ---
// ============================================================================

#include <Wire.h>                             // Library Wire.h einbinden
#include "Si5351.h"                           // Si5351 hier im Ordner einbinden

#define PPS_pin 2                             // 1PPS vom GPS Modul
#define FIn_Pin 5                             // 2,5MHz vom Si5351 (CLK 0)

boolean FrequencyUpdatePending = false;       // (false) Frequenz setzen erforderlich
double  CorrectionFactor       =  1.0;        // (1.0)   ##### wozu?
long    Freq_2_5_Mhz           =  80000000 ;  //  2500000 * 32;  // 2.5 MHz * 32 ( Final divider ) = 80 MHz
long    Freq_10_Mhz            =  80000000 ;  // 10000000 *  8;  // 80 Mhz with Final divider 8   = 10 Mhz
long    Freq_1_Mhz             =  64000000 ;  //  1000000 * 64;  // 1.0 Mhz * 64 

long    Old_2_5_Mhz            =  0;
long    Old_10_Mhz             =  0;
long    Old_1_Mhz              =  0;

#define  count_error              3           // (3)counter error due to Test function and additional instruction
                                              // value to be removed from counter value  #### Feinjustierung der 10MHz Ausgangsfrequenz 

#define counterdelay              4           // (4) Verzoegerung in Sekunden bis zum Start des Zaehlers
int     timegate               = 64;          // (64) Dauer der Torzeit ( + counterdelay )
 
// ----------------------------------------------------------------------------
//                          ---  Timer 1 overflow  ---
// ----------------------------------------------------------------------------

long ovfcpt = 0;

ISR( TIMER1_OVF_vect ) {
  ovfcpt++;                                   // Increment multiplier
  TIFR1 = ( 1 << TOV1 );                      // Clear overlow flag 
}

// ----------------------------------------------------------------------------
//                             ---  PPS pulse    ---
// ----------------------------------------------------------------------------

unsigned long PPSCount = 0;
unsigned long Fcounter = 0;

void doPPS() {

 
  PPSCount++;
 // Serial.println(PPSCount);
  if ( PPSCount == counterdelay ) {
    TCCR1B   = 7;
  } else if ( PPSCount == timegate ) {
__asm__("nop");             //12 mal // 35* = +0.5  
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");
__asm__("nop");

      
    TCCR1B   = 0;                           // Turn off counter  
    Fcounter = ovfcpt * 0x10000 + TCNT1;
    TCNT1    = 0;
    ovfcpt   = 0;
    PPSCount = 0;
    FrequencyUpdatePending=true;            // Frequenz anzeigen ok
  }
}

// ----------------------------------------------------------------------------

void InitFrequency() {
  noInterrupts();                           // Interrupts ausschalten
 
  pinMode(PPS_pin, INPUT);
  pinMode(FIn_Pin, INPUT);

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                               // Disable Timer5 during setup
  TCCR1A = 0;                               // Reset
  TCNT1  = 0;                               // Reset counter to zero
  TIFR1  = 1;                               // Reset overflow
  TIMSK1 = 1;                               // Turn on overflow flag
  
  attachInterrupt(digitalPinToInterrupt(PPS_pin), doPPS, RISING ); 
  interrupts();                             // Interrupts wieder einschalten
}

// ----------------------------------------------------------------------------

// double OldCorrectionFactor = 9.0;        // (9.0)    nicht benutzt !

void UpdateFrequency() {

    Serial.println(F("----------"));   
    Serial.print (F("FCounter: ")); Serial.println(Fcounter); 
    Serial.print (F("Freq_2_5_MHz: ")); Serial.println(Freq_2_5_Mhz);
            
    noInterrupts();
    CorrectionFactor = ( 2500000.0 * ( timegate - counterdelay ) ) / ( Fcounter - count_error );  // Target Frequency x (GateTime-CouterDelay) / (Actual Counter value-CountError)
    //                                     64   -     4                  150000000 - 0
    PPSCount         = 0;
    interrupts(); 
    
    Freq_2_5_Mhz         = long( Freq_2_5_Mhz * CorrectionFactor + 0.5 );
    Freq_10_Mhz          = long( Freq_10_Mhz  * CorrectionFactor + 0.5 );
    Freq_1_Mhz           = long( Freq_1_Mhz   * CorrectionFactor + 0.5 );    

    
    Serial.print((1.0 - CorrectionFactor )*1000000.0); Serial.println(F(" ppm"));    
    Serial.print (F("Freq_2_5_MHz: ")); Serial.println(Freq_2_5_Mhz);
    Serial.print (F("Freq_10_Mhz:  ")); Serial.println(Freq_10_Mhz); 

    float tempC = sensors.getTempC(Thermometer);
    Serial.print  (tempC);                              // Temperatur seriell ausgeben
    Serial.println(F(" °C"));                           // Grad Celsius
  }

 // ----------- Temperaturanzeige auf TFT ------------------------------------
 void TempAnz(){
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(Thermometer);          // Temperatur lesen
  tft.setTextSize(1);                                   // (1) Textgroesse 
  tft.setTextColor(ST77XX_GREEN,ST77XX_BLACK);          // (ST77XX_GREEN,ST77XX_BLACK)
  tft.setCursor(2,116);                                 // (2,116)
  tft.print(tempC);                                     // Temperatur auf TFT ausgeben
  tft.drawCircle(34, 116, 1,ST77XX_GREEN);              // (34,116,1) schreibe Grad Symbol ( ° )
  tft.setCursor(38,116);                                // (38,116)
  tft.print("C");                                       //
 }

// ============================================================================
//                               ---   Setup   ---
// ============================================================================

void setup(void) {
 Serial.begin(115200);
 while (!Serial) ;                                      // Wenn die serielle SS nicht bereit ist, warten
 Serial.print(F("\n\t"));                               // neue Zeile == \n und  Tab == \t
 Serial.print(F("DL8LAB Si5351 10MHz GPS Uhr "));     
 Serial.print(F("\nSketch:   ")); Serial.println(__FILE__);        //   Sketchnamen ausgeben
 Serial.print(F("geflasht am: ")); Serial.print(__DATE__); Serial.print(F(" um ")); Serial.print(__TIME__); Serial.println(F(" Uhr  \n"));  // Datum und Sketchzeit ausgeben

 delay(50);                   // kurze Verzoegerung

 Serial.println(F("Si5351-GPS-10MHz Uhr gestartet"));
  
  InitTft();
//---------------------  Startbildschirm  -------------------------------------
 char text_1[13] = "";                                              // Variable definieren
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);                    // Text- und Hintergrundfarbe
  tft.setTextSize(2);                                               // Textgroesse 2
  strcpy(text_1, "GPS Uhr mit");                                    // Startmeldung Teil 2
  tft.setCursor((( 160 - (strlen(text_1) * 12)) / 2 ), 30);         // zentriert ausgeben Zeile 30
  tft.print(text_1);                                                // ausgeben
  strcpy(text_1, "10 MHz out");                                     // Startmeldung Teil 2
  tft.setCursor((( 160 - (strlen("10 MHz out") * 12)) / 2 ), 70);   // zentriert ausgeben Zeile 70
  tft.print(text_1);                                                // ausgeben
  smartDelay(2000);                                                 // kurz anzeigen 
  tft.fillScreen(ST77XX_BLACK);                                     // TFT loeschen

  
//--------------------  Dein Name  --------------------------------------------
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);                     // Text- und Hintergrundfarbe
  tft.setTextSize(2);                                               // Textgroesse 2
  strcpy(text_1, "Dein");                                           // gebe Deinen Voramen ein
  tft.setCursor((( 160 - (strlen(text_1) * 12)) / 2 ), 30);         // zentriert ausgeben Zeile 30
  tft.print (text_1);                                               // ausgeben
  strcpy(text_1, "Name");                                           // gebe Deinen Namen ein
  tft.setCursor((( 160 - (strlen(text_1) * 12)) / 2 ), 70);         // zentriert ausgeben Zeile 70
  tft.print(text_1);                                                // ausgeben
  smartDelay(2000);                                                 // Anzeige-Verzoegerung
  tft.fillScreen(ST77XX_BLACK);                                     // TFT loeschen  
  
  InitThermometer();                                                // init DS1820
  InitRotary();                                                     // init Encoder
  InitGps();                                                        // init GPS
  InitFrequency();                                                  // init Si5351
 


  Wire.begin();                                                     // I2C Bus starten
  Si5351_Init();                                                    // Si5351 initiieren
  Si5351_SetFreq(SYNTH_MS_0, Freq_2_5_Mhz ,RDIV_32 );
  Si5351_SetFreq(SYNTH_MS_1, Freq_10_Mhz  ,RDIV_8 );
  Si5351_SetFreq(SYNTH_MS_2, Freq_1_Mhz   ,RDIV_64 );    
  Si5351_write(CLK_ENABLE_CONTROL,0b00000100);                      // Clk2 disable  1MHz wird nicht ausgegeben


/*
  while ( gps.charsProcessed() < 10 ) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);  
      tft.setCursor(23, 70);
      tft.setTextSize(2);                 // (3)  original
      tft.print("Kein GPS !");
      smartDelay(500);
    }  
         
  tft.fillScreen(ST77XX_BLACK);
  TempAnz();
  tft.setTextSize(2); 
  tft.setCursor(52,10);
  tft.print("Warte ");
  tft.setCursor(34,30);
  tft.print("auf Zeit");  
  while ( not(gps.time.isValid()) ) {
  smartDelay(500);
  }
  tft.setCursor(52,70);
  tft.print("Warte");
  tft.setCursor(40,90);
  tft.print("auf GPS");
  while ( not(gps.location.isValid()) ) {
  smartDelay(500);
  }
*/

    
  noInterrupts();                                             // Interrupt abschalten
  TCCR1B   = 0;
  Fcounter = 0;
  TCNT1    = 0;
  ovfcpt   = 0;
  PPSCount = 0;
  interrupts();                                               // Interrupt einschalten
  smartDelay(50);
  tft.fillScreen(ST77XX_BLACK);  
  // smartDelay(500);

 //  Serial.println(F("GPS in Ordnung"));  
}

// ============================================================================
//                              ---   Main Loop   ---
// ============================================================================

char xy[4];                                                 // Variable definieren
int ActSel=0;                                               // Merker fuer Sel Knopf gedrueckt

long   DisplayFrequency ;                                   // anzuzeigende Frequenz    nicht mehr noetig !?                     

void loop() {
  readBtnState();
  if (encodermov!= 0 ) {
    ActSel+=encodermov;
    encodermov=0;
  }

  GPSTimeDisplay();                                           // Zeit anzeigen
  TempAnz();                                                  // Temperatur auf TFT anzeigen
  
 // ------------ Zaehlintervall auf TFT ----------------------------------
 
  tft.setTextSize(1);                                         // (1)      
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(142 ,116);                                    // (142,116)
  sprintf(xy, "%2d ", ( timegate - PPSCount));                // sprintf(cz, "%02d ", PPSCount );
  tft.print(xy);  
  // tft.print(PPSCount);





  

// ---------- schlechter oder kein GPS Empfang -------------------------------------
int anz  = gps.satellites.value();                            // Anzahl der Satelliten
 if (anz <2)
 { 
  tft.setTextSize(2); 
  tft.setTextColor(ST77XX_RED,ST77XX_BLACK);  
  tft.setCursor(2,80);                                        // (2,80
  tft.print("warte auf GPS");  
  GPSok=0;
  }
else 
if (GPSok==0) {
  tft.setTextSize(2); 
  tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);  
  tft.setCursor(2,80);                                        // (24,80
  // tft.setCursor((( 160 - (strlen("GPS ok!") * 12)) / 2 ), 80);          // zentriert ausgeben Zeile 30 -- sinnlos
  
  tft.print("   GPS ok!   ");                                 // 13 Stellen * 12 Pixel
  GPSok=1;                                                    // GPS Empfang ok
}
 

 if (FrequencyUpdatePending) {
    UpdateFrequency();
    FrequencyUpdatePending=false;

// ---- Setze geaenderte Frequenz ----------------------------------------
    if ( Old_2_5_Mhz != Freq_2_5_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_0, Freq_2_5_Mhz ,RDIV_32 );
      Old_2_5_Mhz=Freq_2_5_Mhz;
    }
    
    if ( Old_10_Mhz != Freq_10_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_1, Freq_10_Mhz ,RDIV_8);
      Old_10_Mhz = Freq_10_Mhz;
    }
    
    if ( Old_1_Mhz != Freq_1_Mhz ) {
      Si5351_SetFreq(SYNTH_MS_2, Freq_1_Mhz ,RDIV_64);     
      Old_1_Mhz = Freq_1_Mhz;  
    }
    
    DisplayFrequency =  4 * ( Fcounter - count_error )  / ( timegate - counterdelay ) ;   // 4*(150000000-2) / (64 - 4)
    
 // -------------  Frequenz auf TFT anzeigen  --------------------------
    tft.setTextSize(2);                                       // Textgroesse 2
    tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);              // Text- und Hintergrundfarbe festlegen
    tft.setCursor(0,80);                                      // (0,80) setze den Cursor
    //tft.print(DisplayFrequency );                           // berechnete Frequenz auf TFT ausgeben
    tft.print(" 10000000 ");                                  // konstanten Wert auf TFT anzeigen
    tft.setCursor(113,80);                                    // (113,80)  
    tft.print("MHz "); 
   
    tft.setTextSize(1);                                       // (2) ppm anzeigen
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(44,116);                                    // (165,126);
    tft.print("            ");                                // vorherige Ausgabe loeschen   
  bool unsich;                                                // Ausgabefrequenz ist unsicher

float Abw=(1.0 - CorrectionFactor )*1000000.0;                // Fehler in Float uebernehmen
 
   if ((Abw>1000) || (Abw<=-1000)){                           // Auf Abweichung ueberpruefen
    unsich =(true);                                           // merken fuer Text "unsicher" Anzeige
   }
   else
  {
    unsich =(false);                                          // Ausgabefrequenz wieder "sicher"
  }
     
  if ((Abw >= 0) && (Abw <10)) tft.setCursor(68,116);                                     // 1 Stelle vor dem Komma
  if (((Abw >= 10) && (Abw <100)) || ((Abw <0) && (Abw>-10)))   tft.setCursor(62,116);    // 2 Stellen vor dem Komma
  if ((Abw <= -10) && (Abw >-100) || (Abw >100 && Abw<1000))    tft.setCursor(56,116);    // 3 Stellen vor dem Komma
  if ((Abw <= -100) && (Abw >-1000))  tft.setCursor(50,116);                              // 4 Stellen vor dem Komma
   
   //tft.print((1.0 - CorrectionFactor )*1000000.0);
   if (unsich==false)                                         // Genauigkeit brauchbar
   {
    tft.print(Abw);                                           // Abweichung in ppm anzeigen
    tft.setCursor(94,116);                                    // (94,116) setze den Cursor
    tft.print("ppm");                                         // parts per million
   }
   else
   {
    tft.setTextColor(ST77XX_WHITE, ST77XX_RED);               // Text- und Hintergrundfarbe
    tft.setCursor(60,116);                                    // (94,116) setze den Cursor
    tft.print("unsicher");                                    // Meldung auf TFT ausgeben
   }
    
  }
  
  smartDelay(500);                                            // (500)  ms Pause pro Durchlauf
}
 
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
