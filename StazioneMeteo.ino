#include "DHT.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define PIN_DHT 5
#define DHT_TYPE DHT11
#define DELAY_FRAME_DISPLAY  //ritardo tra una frame e l'altra dell'animazione del display
#define IMAGE_WIDTH 47  //lunghezza in pixels dello schermo
#define IMAGE_HEIGHT 60 //altezza in pixels dello schermo
#define NAN - 999.99
#define DELA 5000 //DELAY
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

const static char piantina2_piccola_bits[] PROGMEM = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xE0, 0xFF, 0x00, 0x80, 0x7F, 0x00, 0xE0, 0xFF, 0x00, 0x80, 0x7F, 0x00,
  0xE0, 0xFF, 0x00, 0x80, 0x7F, 0x00, 0xE0, 0xFF, 0x00, 0x80, 0x7F, 0x00,
  0x1E, 0x00, 0x0F, 0x78, 0x80, 0x03, 0x1E, 0x00, 0x0F, 0x78, 0x80, 0x03,
  0x1E, 0x00, 0x0F, 0x78, 0x80, 0x03, 0xFE, 0xFF, 0xF0, 0x87, 0x07, 0x3C,
  0xFE, 0xFF, 0xF0, 0x87, 0x07, 0x3C, 0xFE, 0xFF, 0xF0, 0x87, 0x07, 0x3C,
  0xFE, 0xFF, 0xF0, 0x87, 0x07, 0x3C, 0x1E, 0x00, 0x70, 0x00, 0xF8, 0x3F,
  0x1E, 0x00, 0x70, 0x00, 0xF8, 0x3F, 0x1E, 0x00, 0x70, 0x00, 0xF8, 0x3F,
  0x1E, 0x00, 0x70, 0x00, 0xF8, 0x3F, 0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00,
  0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00, 0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00,
  0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00, 0xE0, 0x00, 0xFF, 0x07, 0x78, 0x00,
  0xE0, 0x00, 0xFF, 0x07, 0x78, 0x00, 0xE0, 0x00, 0xFF, 0x07, 0x78, 0x00,
  0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00, 0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00,
  0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00, 0x00, 0xFF, 0x70, 0xF8, 0x07, 0x00,
  0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00,
  0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00,
  0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00,
  0xE0, 0x00, 0x00, 0x00, 0x78, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x78, 0x00,
  0xE0, 0x00, 0x00, 0x00, 0x78, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00,
  0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00,
  0xE0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0x0F, 0x00, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0x0F, 0x00, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0x0F, 0x00, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0x0F, 0x00, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0x0F, 0x00, 0x80, 0x07, 0x00, 0x00, 0x0F, 0x00, 0x80, 0x07, 0x00,
  0x00, 0xF0, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x7F, 0x00, 0x00,
  0x00, 0xF0, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x7F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

////////////////////////////////// DICHIARAZIONI VARIABILI SENSORI  //////////////////////////
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DHT dht(PIN_DHT, DHT_TYPE);
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
/////////////////////////////////////////////////////////////////////////////////////////////

float soglia_temperatura = 0;
float soglia_pressione = 0;
float soglia_umidita = 0;

int calcolaVariazione(float nuovo, float &soglia)
{

  /*
  Serial.print("valore soglia:");
  Serial.println(soglia);
  Serial.print("Valore letto:");
  Serial.println(nuovo);
  */

  if (nuovo == NAN) return 0;

  if (round(nuovo) > (soglia + (soglia *(10 / 100))))
  {
    soglia = nuovo + nuovo *(10 / 100);
    //Serial.println("sono di pi??");
    return 1;
  }
  else if (round(nuovo) < (soglia - soglia *((10 / 100))))
  {
    soglia = nuovo - nuovo *(10 / 100);
    //Serial.println("sono di meno");
    return -1;
  }
  else
  {
    return 0;
  }
}

//funzione di impostazione dei font dell'OLED
void u8g2_prepare()
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

/*
Funzione che converte il risultato numerico del controllo della variazione in una stringa ASCII visualizzabile sullo schermo.
*/
String esitoVariazione(int numeroVariazione)
{
  Serial.print(numeroVariazione);
  switch (numeroVariazione)
  {
    case -1:
      return "-";
      break;
    case 1:
      return "+";
      break;
    default:
      return "=";
      break;
  }
}

// SCRITTURA DELLA SCHERMATA CHE MOSTRA I VALORI RILEVATI A VIDEO
void disegna_rilevamento(float valhum, float valtemp, float valpres, int humVariazione, int tempVariazione, int presVariazione)
{
  u8g2.setCursor(30, 5);
  u8g2.print(F("Dati Rilevati"));
  u8g2.setCursor(45, 17);
  u8g2.print(F("Hum"));
  u8g2.setCursor(75, 17);
  u8g2.print(F("hPa"));
  u8g2.setCursor(105, 17);
  u8g2.print(F("C\xb0"));
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  u8g2.setCursor(15, 43); //primo numero ?? la colonna, il secondo la riga
  u8g2.print("var");
  u8g2.setCursor(15, 31);
  u8g2.print("val");

  //val e var Hum
  if (valhum != NAN)
  {
    u8g2.drawStr(40, 31, String(valhum).c_str());
    u8g2.drawUTF8(40, 43, esitoVariazione(humVariazione).c_str());
  }
  else
  {
    u8g2.setCursor(40, 31);
    u8g2.print(F("n/a"));
  }
  //val e var temp
  if (valtemp != NAN)
  {
    u8g2.drawStr(95, 31, String(valtemp).c_str());
    u8g2.drawUTF8(95, 43, esitoVariazione(tempVariazione).c_str());
  }
  else
  {
    u8g2.setCursor(95, 31);
    u8g2.print(F("n/a"));
  }

  //val e var pres
  if (valpres != NAN)
  {
    int valpres_int = valpres;
    u8g2.drawStr(70, 31, String(valpres_int).c_str());
    u8g2.drawUTF8(70, 43, esitoVariazione(presVariazione).c_str());
  }
  else
  {
    u8g2.setCursor(70, 31);
    u8g2.print(F("n/a"));
  }
}

// funzione di disegno dell'immagine di screensaver
void u8g2_bitmap()
{
  u8g2.clearBuffer();
  u8g2.drawXBMP(43, 0, IMAGE_WIDTH, IMAGE_HEIGHT, piantina2_piccola_bits);
  u8g2.sendBuffer();
  delay(DELA);
  u8g2.clearBuffer();
}

void setup()
{
  // =============== DHT11 ================
  //inizio lettura dal DHT11 e scrittura su porta seriale
  Serial.begin(9600);
  dht.begin();

  // ================ OLED ===============
  u8g2.begin();
  u8g2_prepare();

  // ================ BMP280 ===============
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, //modo di operazione
    Adafruit_BMP280::SAMPLING_X2, //oversampling della temperatura (non usato)
    Adafruit_BMP280::SAMPLING_X16,  //oversampling della pressione
    Adafruit_BMP280::FILTER_X16,  //filtering
    Adafruit_BMP280::STANDBY_MS_500); //tempo di standby
}

void loop()
{
  u8g2.clearBuffer();

  float humFloat = dht.readHumidity();
  float tempFloat = dht.readTemperature();
  float presFloat = bmp.readPressure()/100; //in hPa

  if (isnan(humFloat))
  {
    humFloat = NAN;
  }
  if (isnan(tempFloat))
  {
    tempFloat = NAN;
  }
  if (isnan(presFloat))
  {
    presFloat = NAN;
  }

  int humVariazione = calcolaVariazione(humFloat, soglia_umidita);
  int tempVariazione = calcolaVariazione(tempFloat, soglia_temperatura);
  int presVariazione = calcolaVariazione(presFloat, soglia_pressione);

  /*
  Serial.println(humFloat);
  Serial.println(tempFloat);
  Serial.println(presFloat);
  */
  
  u8g2.clearBuffer();
  disegna_rilevamento(humFloat, tempFloat, presFloat, humVariazione, tempVariazione, presVariazione);
  u8g2.sendBuffer();
  delay(DELA);

  u8g2_bitmap();
}
