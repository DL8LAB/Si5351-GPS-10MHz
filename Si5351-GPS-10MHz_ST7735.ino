// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Datei Si5351-GPS-10MHz_ST7735.ino   vom 27.03.2023
// Von hier: https://github.com/flafforgue/Si5351-GPS-10MHz
// GPS Uhr mit Si5351 10 MHz Ausgang
// Die Uhr laeuft auch ohne GPS weiter
// Weil kein ST7789 Display vorhanden, zunaechst mit ST7735 Display & Treiber versucht
// TFT-Display ok, Schrift angepasst
// Dallas DS18B20 ok
// Keine Reaktion auf Encoder und Button 
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
// 16 A2 :        - Led
// 17 A3 : Dalas  - DS18B20
// 18 A4 : Si5351 - SDA
// 19 A5 : Si5351 - SCL
// 20 A6 : Rotary - Btn
// 21 A7 :

// ============================================================================
//                          OLED 240 x 240 Screen ?? LCD !
// ============================================================================

#include <SPI.h>             // Arduino SPI library
#include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789  - original
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735 - Versuch

static const int  cs =  6;                     // TFT cs
static const int  dc =  7;                     // TFT DC
static const int rst = -1;                     // TFT RST

// Initialize Adafruit ST7789 TFT library   -cs -dc -rst 
// Adafruit_ST7789 tft = Adafruit_ST7789(-1, 6, 7);        // original stimmt das ?

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);       // mein Display - ok



// ----------------------------------------------------------------------------

void InitTft() {
   
  Serial.println(F("initialisiere TFT 'BLACKTAB'"));  
 // tft.init(240, 240, SPI_MODE2);     // original
  tft.initR(INITR_BLACKTAB);           // initialize a ST7735S chip, black tab  //####
  tft.invertDisplay(true);             // Invertiere die Farben des Displays    //#### bei meinen YELLOTAB Display = true
  tft.setRotation(1);                  // (3) Ausrichtung des Displays 
  tft.fillScreen(ST77XX_BLACK);        // Loesche TFT
  tft.setTextSize(1);                  // (2) Textgroesse
  tft.setTextWrap(false);              // (false) Textumbruch    
  tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);     // Textfarbe , Hintergrund (wenn kein Hintergrund, dann transparent)
  Serial.println(F("TFT in Ordnung"));
}

// ============================================================================
//                                 ---   GPS   ---
// ============================================================================

#include <TimeLib.h>                            // Arduino TimeLib Library by "*Time* by *Michael Margolis*"  einbinden
#include <TinyGPS++.h>                          // TinyGPSPlus Library einbinden
#include <SoftwareSerial.h>                     // SoftwareSerial Library einbinden

static const int RXPin = 8;                     // (8) RxPin fuer GPS Modul SoftwareSerial
static const int TXPin = 9;                     // (9) TxPin fuer GPS Modul SoftwareSerial - nicht benutzt
static const uint32_t GPSBaud = 9600;           // (9600) Standard Geschwindigkeit

TinyGPSPlus gps;                                // Objekt gps erstellen
SoftwareSerial ss(RXPin, TXPin);                // Objekt ss erstellen

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

byte oday = 0;                                  // Merker oldday; Anzeige, wenn der Tag sich geaendert hat
byte osec = 61;                                 // Merker oldsec; Anzeige, wenn die Sekunde sich geaendert hat

// ============================================================================
//                          ---   GPS Time Display   ---
// ============================================================================

void GPSTimeDisplay(){                                     
  if ( gps.date.isUpdated() ) {                             //  Datum uebernehmen
    int  Year   = gps.date.year();
    byte Month  = gps.date.month();
    byte Day    = gps.date.day();
    byte Hour   = gps.time.hour();                          //  Zeit uebernehmen
    byte Minute = gps.time.minute();
    byte Second = gps.time.second();

// ----------- Funktion EU-Sommerzeit eingefuegt ------------------------------
    byte tzHours=1;                                         // Zeitzone 0=UTC, 1=MEZ
    boolean st = sommertime_EU(Year,Month,Day,Hour,tzHours);  // Funktion aufrufen
    if (st) 
      { tzHours = 2 ; }                                     // UTC -2 (Sommerzeit)
   else
      { tzHours = 1 ; }                                     // UTC -1 (Winterzeit)
// -----------------------------------------------------------------------------
    setTime(Hour, Minute, Second, Day, Month, Year);        // Zeit der Softwareuhr setzen
    // adjustTime( 2 * SECS_PER_HOUR );                     // (2) fix, Zeitzone addieren (x * 3600 = h)
    adjustTime( tzHours * SECS_PER_HOUR );                  // (tzHours) abhaening addieren (tzHours * 3600 = h)
 }
  
  char sz[32];
  if ( oday != day() ) {                                    // bei Aenderung Datum anzeigen
    tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);           // Schrift gelb, Hintergrund schwarz
    tft.setTextSize(2);                                     // (3) Textgroesse
    tft.setCursor(20,10);                                   // (5,20) setze Cursor
    sprintf(sz, "%02d.%02d.%02d", day(), month(), year() );    // Ausgabe formatiern
    tft.print(sz);                                          // formatierte Ausgabe auf TFT
    oday = day();
  }

  if ( osec != second() ) {                                 // bei Aenderung Uhrzeit anzeigen
    tft.setTextColor(ST77XX_YELLOW,ST77XX_BLACK);           // Schrift gelb, Hintergrund schwarz
    tft.setTextSize(2);                                     // (3) Textgroesse   
    tft.setCursor(8,40);                                    // (5,50)  setze Cursor
    sprintf(sz, "%02d:%02d:%02d Uhr", hour(), minute(), second() );  // Ausgabe formatiern
    tft.print(sz);                                          // formatierte Ausgabe auf TFT
    osec = second();
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

#include <OneWire.h>                          // OnWire Library einbinden
#include <DallasTemperature.h>                // DallasTemperature Library einbinden
OneWire oneWire(A3);                          // (A3) Onewire SensorPin festlegen
DallasTemperature sensors(&oneWire);          // DallasTemperature mit OneWire Pin starten
DeviceAddress Thermometer;                    // Objekt erstellen

void InitThermometer() {                      // Thermometer starten
 Serial.println(F("inititalisere Temp-Sensor"));
  sensors.begin();
  if (!sensors.getAddress(Thermometer, 0)) tft.print(" Th. Error !") ;          // kein Sensor gefunden
  sensors.setResolution(Thermometer, 9);      // (9) Aufloesung festlegen 
  Serial.println(F("Temperatursensor erkannt"));                                
}

// ============================================================================
//                         ---   Rotary  Encoder  ---
// ============================================================================

#define encoder0PinA      3           // (3)  Encoder CLK an Pin 3
#define encoder0PinB      4           // (4)  Encoder DA an Pin 4
#define BtnEncoder       20           // (20 A6) Encoder SW an Pin 20

//#define BTN_None          0           // (0)  Pin 0  ??
// ------ neuer Versuch  ---------------------
byte BTN_None     = 0;                // nicht gedrueckt

#define BTN_Encoder       1           // (1)  Pin 1  ??
#define BTN_EncoderL      2           // (2)  Pin 2  ??
#define BTN_LONGDELAY   800

int posencoder          = 0;          // Zaehler fuer Endoder Position
int encodermov          = 0;          // Merker Bewegung
unsigned long ntime     = 0;          // Merker ntime
unsigned long otime     = 0;          // Merker otime

byte keydown            = BTN_None;   // nicht gedrueckt
byte key                = BTN_None;   // nicht gedrueckt
unsigned long BTNTime;                // Zeitdauer fuer Tastendruck

// ----------------------------------------------------------------------------

void doEncoder() {                      // Drehung feststellen und zaehlen
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
    // Serial.print(" EncPos "); Serial.println(posencoder);  //#### zum Debuggen
  }  
}

// ----------------------------------------------------------------------------

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
              Serial.print("Taster "); Serial.println(key);     //#### zum Debuggen      
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

#include <Wire.h>
#include "Si5351.h"

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

#define  count_error              3             // (2) counter error due to Test function and additional instruction
                                                 // value to be removed from counter value  #### Feinjustierung der 10MHz Ausgangsfrequenz 

#define counterdelay              4              // (4) Verzoegerung in Sekunden bis zum Start des Zaehlers
int     timegate               = 64;             // (64) Dauer der Torzeit ( + counterdelay )
 
// ----------------------------------------------------------------------------
//                          ---  Timer 1 overflow  ---
// ----------------------------------------------------------------------------

long ovfcpt = 0;

ISR( TIMER1_OVF_vect ) {
  ovfcpt++;                        // Increment multiplier
  TIFR1 = ( 1 << TOV1 );           // Clear overlow flag 
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
    TCCR1B   = 0;                          // Turn off counter  
    Fcounter = ovfcpt * 0x10000 + TCNT1;
    TCNT1    = 0;
    ovfcpt   = 0;
    PPSCount = 0;
    FrequencyUpdatePending=true;            // Frequenz anzeigen ok
  }
}

// ----------------------------------------------------------------------------

void InitFrequency() {
  noInterrupts();                       // Interrupts ausschalten
 
  pinMode(PPS_pin, INPUT);
  pinMode(FIn_Pin, INPUT);

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;                                    //Disable Timer5 during setup
  TCCR1A = 0;                                    //Reset
  TCNT1  = 0;                                    //Reset counter to zero
  TIFR1  = 1;                                    //Reset overflow
  TIMSK1 = 1;                                    //Turn on overflow flag
  
  attachInterrupt(digitalPinToInterrupt(PPS_pin), doPPS, RISING ); 
  interrupts();                                 // Interrupts wieder einschalten
}

// ----------------------------------------------------------------------------

double OldCorrectionFactor = 9.0;        // (9.0)    nicht benutzt !?

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

/*
    Serial.print("CorrectionFactor ");Serial.println(CorrectionFactor);         // diese Werte aendern sich nicht!
    Serial.print("timegate ");Serial.println(timegate);                         // fest: 64
    Serial.print("counterdelay ");Serial.println(counterdelay);                 // fest: 4
    Serial.print("count_error ");Serial.println(count_error);                   // laut Vorgabe
    Serial.print("CF: ");   Serial.println(CorrectionFactor );                  // fest: 1.0
*/
    
    Serial.print((1.0 - CorrectionFactor )*1000000.0); Serial.println(F(" ppm"));    
    Serial.print (F("Freq_2_5_MHz: ")); Serial.println(Freq_2_5_Mhz);
    Serial.print (F("Freq_10_Mhz:  ")); Serial.println(Freq_10_Mhz); 

    float tempC = sensors.getTempC(Thermometer);
    Serial.print  (tempC);
    Serial.println(F(" °C")); 

 
    
}

// ============================================================================
//                               ---   Setup   ---
// ============================================================================

void setup(void) {
 Serial.begin(115200);
 while (!Serial) ;            // Wenn die serielle SS nicht bereit ist, warten
 Serial.print(F("\n\t"));                                     // neue Zeile == \n und  Tab == \t
 Serial.print(F("DL8LAB Si5351 GPS 10MHz Uhr "));     
 Serial.print(F("\nSketch:   "));   Serial.println(__FILE__);        //   Sketchnamen ausgeben
 Serial.print(F("geflasht am: "));   Serial.print(__DATE__);  Serial.print(F(" um "));   Serial.print(__TIME__); Serial.println(F(" Uhr  \n"));    // Datum und Sketchzeit ausgeben

 delay(50);                   // kurze Verzoegerung

 
 Serial.println(F("Si5351-GPS-10MHz Uhr gestartet"));
  
  InitTft();
//---------------------------------------
  tft.setTextSize(2);                             // (1)
  tft.setCursor(16,30);
  tft.print("GPS Uhr mit");
  tft.setCursor(20,70);
  tft.print("10 MHz Out");
  smartDelay(2000);                               // kurz anzeigen 

tft.fillScreen(ST77XX_BLACK);    // original
// smartDelay(2000);                                 // kurz anzeigen 
  
//---------------------------------


  
  
  InitThermometer();                                  // init DS1820
  InitRotary();                                       // init Encoder
  InitGps();                                          // init GPS
  InitFrequency();                                    // init Si5351

  Wire.begin();
  Si5351_Init();
  Si5351_SetFreq(SYNTH_MS_0, Freq_2_5_Mhz ,RDIV_32 );
  Si5351_SetFreq(SYNTH_MS_1, Freq_10_Mhz  ,RDIV_8 );
  Si5351_SetFreq(SYNTH_MS_2, Freq_1_Mhz   ,RDIV_64 );    
  Si5351_write(CLK_ENABLE_CONTROL,0b00000100);        // Clk2 disable  1MHz wird nicht ausgegeben


  tft.setTextSize(2);         // (2) original
  tft.setCursor(40,10);
  tft.print(" Warte");
  tft.setCursor(40,30);
  tft.print("auf GPS");
  smartDelay(1000);
  

  while ( gps.charsProcessed() < 10 ) {
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(0, 110);
      tft.setTextSize(2);                 // (3)  original
      tft.print("No GPS Found !");
      smartDelay(1000);
  }
  tft.fillScreen(ST77XX_BLACK);

  tft.setCursor(40,10);
  tft.print(" Warte ");
  tft.setCursor(32,30);
  tft.print("auf Zeit");  
  while ( not(gps.time.isValid()) ) {
      smartDelay(500);
  }
  tft.setCursor(40,70);
  tft.print(" Warte ");
  tft.setCursor(40,90);
  tft.print("auf GPS");
  while ( not(gps.location.isValid()) ) {
      smartDelay(500);
  }
    
  noInterrupts();
  TCCR1B   = 0;
  Fcounter = 0;
  TCNT1    = 0;
  ovfcpt   = 0;
  PPSCount = 0;
  interrupts();
  tft.fillScreen(ST77XX_BLACK);  

  Serial.println(F("GPS in Ordnung"));    

  
}



// ============================================================================
//                              ---   Main Loop   ---
// ============================================================================

char xy[4];                                             // 

int ActSel=0;

long   DisplayFrequency ;
double DisplayError ;

void loop() {
  readBtnState();
  if (encodermov!= 0 ) {
    ActSel+=encodermov;
    encodermov=0;
  }


  GPSTimeDisplay();                                     // Zeit anzeigen

  
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(Thermometer);
  tft.setTextSize(1);                                   // (2)
  tft.setTextColor(ST77XX_GREEN,ST77XX_BLACK);          // (ST77XX_BLUE)
  tft.setCursor(2,116);                                 // (150,200)
  tft.print(tempC);
  tft.setCursor(34,116);                                // (200,200)
  tft.print("C");
  tft.setTextSize(1);                                   // (1)      
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(142 ,116);                              // (5,200)
  sprintf(xy, "%2d ", ( timegate - PPSCount));                      // sprintf(cz, "%02d ", PPSCount );
  tft.print(xy);  
   // tft.print(PPSCount);

int anz  = gps.satellites.value();
if (anz <2)
{ 
 tft.setTextSize(2); 
 tft.setTextColor(ST77XX_RED,ST77XX_BLACK);  
 tft.setCursor(2,80);                       // (0,80
 tft.print("warte auf GPS"); 
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
    DisplayError     = (4 * ( Fcounter - count_error )) % ( timegate - counterdelay ) ;
    DisplayError     = DisplayError / ( timegate - counterdelay ) ;
  
    tft.setTextSize(2);                         //  (3) Frequenz anzeigen
    tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK);
    tft.setCursor(0,80);                       // (16,80) (5,120)
    //tft.print(DisplayFrequency );         //  berechnete Frequenz 
    tft.print(" 10000000 "); 
    tft.setCursor(113,80);                   //  (114,80)  
    tft.print("MHz "); 
   
    tft.setTextSize(1);                                 // (2) ppm anzeigen
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(56,116);                              // (165,126);
    //tft.print( DisplayError );                          // Fehler in ppm anzeigen
    tft.print((1.0 - CorrectionFactor )*1000000.0);
    
    // tft.setCursor(88,116);      
    tft.print("ppm   ");
  }
  
  smartDelay(500);                    // (500)
}
 
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
