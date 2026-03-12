/**************************************************************************
 * rakuda_face.ino
 * Waveshare ESP32-S3 Touch-LCD 1.69″ – 3 barre + bocca idle
 *
 * Dopo SILENCE_TIMEOUT_MS di silenzio appare una bocca (3 quadrati)
 * centrata rispetto all'area dell'animazione, stesso colore.
 **************************************************************************/

#include <Arduino_GFX_Library.h>
#include <canvas/Arduino_Canvas.h>
#include "USB.h"
#include "USBCDC.h"

USBCDC USBSerial;

/* ------- pin fissi Waveshare ------- */
constexpr int LCD_DC = 4,  LCD_CS = 5,  LCD_RST = 8,  LCD_BL = 15;
constexpr int LCD_SCK = 6, LCD_MOSI = 7;

/* ------- bus SPI ------- */
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI,
    GFX_NOT_DEFINED, 60000000);

/* ------- driver ST7789 240×280 ------- */
Arduino_GFX *tft = new Arduino_ST7789(
    bus, LCD_RST, 0, true,
    240, 280, 0, 20, 0, 20);

/* ------- parametri barre ------- */
constexpr uint8_t  SEGMENTS = 3;
constexpr uint16_t SEG_W    = 15;
constexpr int16_t  AXIS_X   = 150;
constexpr uint16_t BAR_H    = 30;
constexpr uint8_t  BAR_SP   = 20;
constexpr int16_t  CENTER_Y = 140;
constexpr int16_t  Y_OFFSET = CENTER_Y - (BAR_H / 2) - (BAR_H + BAR_SP);

/* ------- canvas PSRAM (240 × 112) ------- */
constexpr uint16_t CW = 240;
constexpr uint16_t CH = 3 * BAR_H + 2 * BAR_SP;   // 112 px
Arduino_Canvas *canvas = new Arduino_Canvas(CW, CH, tft, 0, Y_OFFSET);

/* ------- colori & timing ------- */
constexpr uint16_t COL_ON   = 0x6D9D;
constexpr uint16_t COL_OFF  = 0x0000;
constexpr uint16_t FRAME_MS = 30;

/* ------- timeout silenzio ──────────────────────────────────────────
   Modifica questo valore per cambiare i secondi di attesa           */
constexpr uint32_t SILENCE_TIMEOUT_MS = 0;

/* ------- parametri bocca ------- */
constexpr uint16_t MOUTH_SQ   = 24;   // lato quadratino (doppio)
constexpr uint8_t  MOUTH_GAP  = 4;    // gap tra quadratini
// 3 quadrati disposti verticalmente: altezza totale = 3*24 + 2*4 = 80 px
// X centrata su AXIS_X
constexpr int16_t  MOUTH_X    = AXIS_X - MOUTH_SQ / 2;
// Y di partenza centrata nel canvas
constexpr int16_t  MOUTH_Y    = (CH - (3 * MOUTH_SQ + 2 * MOUTH_GAP)) / 2;

/* ------- stato ------- */
int      lastRms            = 0;
bool     wasSilent          = true;
bool     mouthVisible       = false;
uint32_t silenceStart       = 0;
bool     firstInputReceived = false;

/* ------------------------------------------------------------------ */
inline void drawFrame(uint8_t lvlTop, uint8_t lvlMid, uint8_t lvlBot)
{
  canvas->fillScreen(COL_OFF);

  auto drawBar = [&](uint8_t row, uint8_t level) {
    uint16_t y = row * (BAR_H + BAR_SP);
    for (uint8_t s = 0; s < level; ++s) {
      int16_t xL = AXIS_X - (s + 1) * SEG_W;
      int16_t xR = AXIS_X + s * SEG_W + 1;
      if (xL >= 0)               canvas->fillRect(xL, y, SEG_W - 1, BAR_H, COL_ON);
      if (xR + SEG_W - 1 < CW)  canvas->fillRect(xR, y, SEG_W - 1, BAR_H, COL_ON);
    }
  };

  drawBar(0, lvlTop);
  drawBar(1, lvlMid);
  drawBar(2, lvlBot);
}

/* ------------------------------------------------------------------ */
void drawMouth()
{
  canvas->fillScreen(COL_OFF);
  for (uint8_t i = 0; i < 3; i++) {
    int16_t y = MOUTH_Y + i * (MOUTH_SQ + MOUTH_GAP);
    canvas->fillRect(MOUTH_X, y, MOUTH_SQ, MOUTH_SQ, COL_ON);
  }
  canvas->flush();
}

/* ------------------------------------------------------------------ */
void setup()
{
  USBSerial.begin(115200);
  USB.begin();

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, LOW);   // spento finché non arriva il primo input

  canvas->begin();
  tft->fillScreen(COL_OFF);
  canvas->fillScreen(COL_OFF);
  canvas->flush();

  randomSeed(esp_random());
  silenceStart = millis();
}

/* ------------------------------------------------------------------ */
void loop()
{
  // ── Leggi RMS ──────────────────────────────────────────────────────
  while (USBSerial.available()) {
    String line = USBSerial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int val = line.toInt();
      if (val == -1) {
        // segnale di shutdown: spegni tutto
        canvas->fillScreen(COL_OFF);
        canvas->flush();
        digitalWrite(LCD_BL, LOW);
        firstInputReceived = false;
        lastRms = 0;
        mouthVisible = false;
      } else {
        lastRms = constrain(val, 0, 255);
        if (!firstInputReceived) {
          firstInputReceived = true;
          digitalWrite(LCD_BL, HIGH);
        }
      }
    }
  }

  // schermo spento finché non arriva il primo input
  if (!firstInputReceived) {
    delay(FRAME_MS);
    return;
  }

  bool silent = (lastRms < 5);

  // ── Gestione transizioni ───────────────────────────────────────────
  if (!silent) {
    // torna il suono → azzera tutto
    silenceStart  = millis();
    mouthVisible  = false;
  } else if (!wasSilent) {
    // appena entrato nel silenzio → avvia contatore
    silenceStart = millis();
  }

  wasSilent = silent;

  // ── Silenzio ───────────────────────────────────────────────────────
  if (silent) {
    uint32_t elapsed = millis() - silenceStart;

    if (elapsed >= SILENCE_TIMEOUT_MS && !mouthVisible) {
      // primo frame dopo il timeout: disegna la bocca
      drawMouth();
      mouthVisible = true;
    } else if (!mouthVisible) {
      // silenzio ma non ancora timeout: schermo nero
      canvas->fillScreen(COL_OFF);
      canvas->flush();
    }
    // se mouthVisible: non ridisegna nulla, bocca resta ferma

    delay(FRAME_MS);
    return;
  }

  // ── Suono: animazione random originale ────────────────────────────
  uint8_t center = random(0, SEGMENTS + 1);
  uint8_t side   = random(0, center   + 1);

  drawFrame(side, center, side);
  canvas->flush();

  delay(FRAME_MS);
}