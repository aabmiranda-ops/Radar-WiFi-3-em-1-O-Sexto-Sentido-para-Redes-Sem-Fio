/**
 * ============================================================================
 * PROJETO ESP32 MULTIUSO 3-EM-1 (RADAR WIFI / GEOLOCALIZAÇÃO / REDES OCULTAS)
 * ============================================================================
 * Placa Alvo : ESP32 NodeMCU / DevKit de 38 Pinos
 * Display    : OLED 1.3" SH1106 (VDD-GND-SCK-SDA) / Compatível também c/ SSD1306
 *
 * PINAGEM DO HARDWARE (Conforme suas fotos):
 *  - Display OLED I2C :
 *      Pino 4 (SDA) -> G21 (6º pino da direita do ESP32)
 *      Pino 3 (SCK) -> G22 (3º pino da direita do ESP32)
 *      Pino 2 (GND) -> GND (1º pino da direita do ESP32)
 *      Pino 1 (VDD) -> 3V3 (1º pino da esquerda do ESP32)
 *  - Botão Seletor de Modo (SW1): G23 (2º pino da direita, INPUT_PULLUP interno)
 *  - Buzzer Sonoro (BZ1)        : G18 (9º pino da direita, Controle PWM / Tom)
 *  - LED 3 (Forte / Modo 3)     : G17 (11º pino da direita)
 *  - LED 2 (Médio / Modo 2)     : G16 (12º pino da direita)
 *  - LED 1 (Fraco / Modo 1)     : G4  (13º pino da direita)
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// 1. DEFINIÇÕES DE PINOS E CONSTANTES DE HARDWARE
// ============================================================================
#define PIN_OLED_SDA     21
#define PIN_OLED_SCL     22
#define PIN_BUTTON       23
#define PIN_BUZZER       18
#define PIN_LED_1         4
#define PIN_LED_2        16
#define PIN_LED_3        17

// LED interno azul da placa ESP32 (pisca para mostrar que o código está rodando!)
#define PIN_ONBOARD_LED   2

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1

// Controle automático do Display: usa SH1106G (padrão dos displays azuis VDD-GND-SCK-SDA)
// Se você mudar para false, ele usa SSD1306.
#define USE_SH1106_DRIVER  true

#if USE_SH1106_DRIVER
  Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #define COLOR_WHITE SH110X_WHITE
  #define COLOR_BLACK SH110X_BLACK
#else
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #define COLOR_WHITE SSD1306_WHITE
  #define COLOR_BLACK SSD1306_BLACK
#endif

// Limites de RSSI (em dBm) para cálculo de porcentagem de proximidade
#define RSSI_MIN_DBM    -95   // 0% de proximidade (muito longe)
#define RSSI_MAX_DBM    -35   // 100% de proximidade (colado no roteador)

// Intervalo entre varreduras assíncronas de Wi-Fi (ms)
#define WIFI_SCAN_INTERVAL_MS  800

// ============================================================================
// 2. CONFIGURAÇÃO DE REDES ALVO (PERSONALIZE AQUI SEUS ROTEADORES!)
// ============================================================================
const char* TARGET_RADAR_BSSID = "AA:BB:CC:DD:EE:FF";
const bool  RADAR_FALLBACK_TO_STRONGEST = true;

struct RoomAnchor {
  const char* bssid;
  const char* ssid;
  const char* roomName;
};

const RoomAnchor ROOM_ANCHORS[3] = {
  { "11:22:33:44:55:01", "WIFI_SALA",       "Voce esta na Sala"       },
  { "11:22:33:44:55:02", "WIFI_QUARTO",     "Voce esta no Quarto"     },
  { "11:22:33:44:55:03", "WIFI_ESCRITORIO", "Voce esta no Escritorio" }
};

// ============================================================================
// 3. OBJETOS E VARIÁVEIS DE ESTADO GLOBAIS
// ============================================================================
enum OperationMode {
  MODE_PROXIMITY_RADAR = 0,
  MODE_INDOOR_LOCATION = 1,
  MODE_HIDDEN_DETECTOR = 2
};

volatile OperationMode currentMode = MODE_PROXIMITY_RADAR;

bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

bool scanInProgress = false;
unsigned long lastScanTriggerTime = 0;

unsigned long lastBeepToggleTime = 0;
bool buzzerToneActive = false;
uint8_t detectedOledAddr = 0x3C;
bool oledConnected = false;

struct RadarData {
  bool found;
  String ssid;
  String bssid;
  int32_t rssi;
  int proximityPct;
  int32_t channel;
  bool isFallback;
} radarState = { false, "", "", -100, 0, 0, false };

struct GeoData {
  bool anyConfiguredFound;
  int bestRoomIndex;
  int32_t roomRssi[3];
  String topDemoSsid[3];
  int32_t topDemoRssi[3];
  int topDemoCount;
} geoState = { false, -1, {-100, -100, -100}, {"", "", ""}, {-100, -100, -100}, 0 };

#define MAX_HIDDEN_NETS 6
struct HiddenNetInfo {
  String bssid;
  int32_t rssi;
  int32_t channel;
};
HiddenNetInfo hiddenNets[MAX_HIDDEN_NETS];
int hiddenNetsCount = 0;
int totalNetworksLastScan = 0;

// ============================================================================
// 4. SCANNER I2C AUTOMÁTICO (DETECTA SE O OLED ESTÁ EM 0x3C OU 0x3D)
// ============================================================================
uint8_t scanI2CForOLED() {
  Serial.println(F("[I2C] Procurando Display OLED nos pinos G21 (SDA) e G22 (SCK)..."));
  uint8_t foundAddr = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("[I2C] Dispositivo encontrado no endereco 0x"));
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      if (addr == 0x3C || addr == 0x3D) {
        foundAddr = addr;
      } else if (foundAddr == 0) {
        foundAddr = addr;
      }
    }
  }
  if (foundAddr == 0) {
    Serial.println(F("[AVISO I2C] Nenhum dispositivo respondeu! Verifique os 4 fios do OLED:"));
    Serial.println(F("  Pino 1 (VDD) -> 3V3 | Pino 2 (GND) -> GND | Pino 3 (SCK) -> G22 | Pino 4 (SDA) -> G21"));
  }
  return foundAddr;
}

bool initOLEDDisplay() {
  uint8_t addr = scanI2CForOLED();
  if (addr != 0) {
    detectedOledAddr = addr;
  } else {
    detectedOledAddr = 0x3C;
  }

#if USE_SH1106_DRIVER
  bool ok = display.begin(detectedOledAddr, true);
#else
  bool ok = display.begin(SSD1306_SWITCHCAPVCC, detectedOledAddr);
#endif
  if (ok && addr != 0) {
    oledConnected = true;
    display.clearDisplay();
    display.setTextColor(COLOR_WHITE);
    display.display();
  }
  return oledConnected;
}

// ============================================================================
// 5. FUNÇÕES AUXILIARES DE HARDWARE (BUZZER, LEDS E CONVERSÕES)
// ============================================================================
void playBuzzerTone(uint32_t freqHz) {
  if (freqHz == 0) {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
  } else {
    tone(PIN_BUZZER, freqHz);
  }
}

void beepModeChange(OperationMode newMode) {
  uint32_t freq = 1200 + ((int)newMode * 400);
  playBuzzerTone(freq);
  delay(70);
  playBuzzerTone(0);
}

int rssiToPercentage(int32_t rssi) {
  if (rssi <= RSSI_MIN_DBM) return 0;
  if (rssi >= RSSI_MAX_DBM) return 100;
  return map(rssi, RSSI_MIN_DBM, RSSI_MAX_DBM, 0, 100);
}

void updateStatusLEDs() {
  if (currentMode == MODE_PROXIMITY_RADAR) {
    if (!radarState.found) {
      digitalWrite(PIN_LED_1, LOW);
      digitalWrite(PIN_LED_2, LOW);
      digitalWrite(PIN_LED_3, LOW);
    } else {
      digitalWrite(PIN_LED_1, (radarState.proximityPct >= 15) ? HIGH : LOW);
      digitalWrite(PIN_LED_2, (radarState.proximityPct >= 50) ? HIGH : LOW);
      digitalWrite(PIN_LED_3, (radarState.proximityPct >= 80) ? HIGH : LOW);
    }
  } else if (currentMode == MODE_INDOOR_LOCATION) {
    int activeIdx = geoState.anyConfiguredFound ? geoState.bestRoomIndex : (geoState.topDemoCount > 0 ? 0 : -1);
    digitalWrite(PIN_LED_1, (activeIdx == 0) ? HIGH : LOW);
    digitalWrite(PIN_LED_2, (activeIdx == 1) ? HIGH : LOW);
    digitalWrite(PIN_LED_3, (activeIdx == 2) ? HIGH : LOW);
  } else if (currentMode == MODE_HIDDEN_DETECTOR) {
    digitalWrite(PIN_LED_3, HIGH);
    bool hasHidden = (hiddenNetsCount > 0);
    digitalWrite(PIN_LED_1, hasHidden ? HIGH : LOW);
    digitalWrite(PIN_LED_2, (hiddenNetsCount > 1) ? HIGH : LOW);
  }
}

void handleRadarBuzzerNonBlocking() {
  if (currentMode != MODE_PROXIMITY_RADAR || !radarState.found || radarState.proximityPct < 10) {
    if (buzzerToneActive) {
      playBuzzerTone(0);
      buzzerToneActive = false;
    }
    return;
  }

  unsigned long now = millis();
  unsigned long beepPeriodMs = map(radarState.proximityPct, 10, 100, 700, 75);
  unsigned long beepOnDurationMs = 45;
  uint32_t beepFreqHz = map(radarState.proximityPct, 10, 100, 900, 2600);

  if (buzzerToneActive) {
    if (now - lastBeepToggleTime >= beepOnDurationMs) {
      playBuzzerTone(0);
      buzzerToneActive = false;
      lastBeepToggleTime = now;
    }
  } else {
    if (now - lastBeepToggleTime >= beepPeriodMs) {
      playBuzzerTone(beepFreqHz);
      buzzerToneActive = true;
      lastBeepToggleTime = now;
    }
  }
}

// ============================================================================
// 6. PROCESSAMENTO DOS DADOS DO ESCANEAMENTO WI-FI
// ============================================================================
bool isSsidHidden(const String& ssid) {
  if (ssid.length() == 0) return true;
  for (size_t i = 0; i < ssid.length(); i++) {
    if (ssid[i] != '\0' && ssid[i] != ' ') {
      return false;
    }
  }
  return true;
}

void processWiFiScanResults(int numNetworks) {
  totalNetworksLastScan = numNetworks;
  // Pisca rapidamente o LED interno azul da placa ESP32 a cada scan concluído
  digitalWrite(PIN_ONBOARD_LED, !digitalRead(PIN_ONBOARD_LED));

  radarState.found = false;
  radarState.rssi = -100;
  radarState.proximityPct = 0;
  radarState.isFallback = false;

  int strongestIndex = -1;
  int32_t strongestRssi = -120;

  for (int i = 0; i < numNetworks; i++) {
    String bssidStr = WiFi.BSSIDstr(i);
    int32_t rssiVal = WiFi.RSSI(i);

    if (rssiVal > strongestRssi) {
      strongestRssi = rssiVal;
      strongestIndex = i;
    }

    if (bssidStr.equalsIgnoreCase(TARGET_RADAR_BSSID)) {
      radarState.found = true;
      radarState.ssid = WiFi.SSID(i);
      radarState.bssid = bssidStr;
      radarState.rssi = rssiVal;
      radarState.channel = WiFi.channel(i);
      radarState.proximityPct = rssiToPercentage(rssiVal);
      radarState.isFallback = false;
    }
  }

  if (!radarState.found && RADAR_FALLBACK_TO_STRONGEST && strongestIndex >= 0) {
    radarState.found = true;
    radarState.ssid = WiFi.SSID(strongestIndex);
    if (radarState.ssid.length() == 0) radarState.ssid = "<Oculta>";
    radarState.bssid = WiFi.BSSIDstr(strongestIndex);
    radarState.rssi = WiFi.RSSI(strongestIndex);
    radarState.channel = WiFi.channel(strongestIndex);
    radarState.proximityPct = rssiToPercentage(radarState.rssi);
    radarState.isFallback = true;
  }

  geoState.anyConfiguredFound = false;
  geoState.bestRoomIndex = -1;
  int32_t bestConfiguredRssi = -120;

  for (int r = 0; r < 3; r++) {
    geoState.roomRssi[r] = -100;
    for (int i = 0; i < numNetworks; i++) {
      String bssidStr = WiFi.BSSIDstr(i);
      String ssidStr  = WiFi.SSID(i);
      if (bssidStr.equalsIgnoreCase(ROOM_ANCHORS[r].bssid) ||
          (strlen(ROOM_ANCHORS[r].ssid) > 0 && ssidStr.equalsIgnoreCase(ROOM_ANCHORS[r].ssid))) {
        int32_t rssi = WiFi.RSSI(i);
        if (rssi > geoState.roomRssi[r]) {
          geoState.roomRssi[r] = rssi;
        }
        geoState.anyConfiguredFound = true;
        if (rssi > bestConfiguredRssi) {
          bestConfiguredRssi = rssi;
          geoState.bestRoomIndex = r;
        }
      }
    }
  }

  geoState.topDemoCount = min(numNetworks, 3);
  for (int k = 0; k < geoState.topDemoCount; k++) {
    String s = WiFi.SSID(k);
    if (s.length() == 0) s = WiFi.BSSIDstr(k).substring(9);
    geoState.topDemoSsid[k] = s;
    geoState.topDemoRssi[k] = WiFi.RSSI(k);
  }

  hiddenNetsCount = 0;
  for (int i = 0; i < numNetworks && hiddenNetsCount < MAX_HIDDEN_NETS; i++) {
    String ssidStr = WiFi.SSID(i);
    if (isSsidHidden(ssidStr)) {
      hiddenNets[hiddenNetsCount].bssid   = WiFi.BSSIDstr(i);
      hiddenNets[hiddenNetsCount].rssi    = WiFi.RSSI(i);
      hiddenNets[hiddenNetsCount].channel = WiFi.channel(i);
      hiddenNetsCount++;
    }
  }

  WiFi.scanDelete();
}

void manageAsyncWiFiScan() {
  int16_t scanStatus = WiFi.scanComplete();

  if (scanStatus == WIFI_SCAN_RUNNING) {
    return;
  }

  if (scanStatus >= 0) {
    processWiFiScanResults(scanStatus);
    scanInProgress = false;
    lastScanTriggerTime = millis();
  } else if (scanStatus == WIFI_SCAN_FAILED || !scanInProgress) {
    if (millis() - lastScanTriggerTime >= WIFI_SCAN_INTERVAL_MS) {
      WiFi.scanNetworks(true, true);
      scanInProgress = true;
      lastScanTriggerTime = millis();
    }
  }
}

// ============================================================================
// 7. TELAS DO DISPLAY OLED (INTERFACE GRÁFICA PARA OS 3 MODOS)
// ============================================================================
void drawHeader(const char* title) {
  display.fillRect(0, 0, SCREEN_WIDTH, 13, COLOR_WHITE);
  display.setTextColor(COLOR_BLACK);
  display.setTextSize(1);
  display.setCursor(3, 3);
  display.print(title);
  display.setTextColor(COLOR_WHITE);
}

void renderMode1Radar() {
  drawHeader("M1: RADAR PROXIMIDADE");

  if (!radarState.found) {
    display.setCursor(0, 20);
    display.println("Buscando alvo...");
    display.setCursor(0, 32);
    display.print("MAC: ");
    display.println(TARGET_RADAR_BSSID);
    return;
  }

  display.setCursor(0, 16);
  display.print(radarState.isFallback ? "Alvo(Auto): " : "Alvo: ");
  display.println(radarState.ssid.substring(0, 11));

  display.setCursor(0, 26);
  display.print(radarState.bssid);
  display.print(" C");
  display.println(radarState.channel);

  display.setCursor(0, 37);
  display.print("Sinal: ");
  display.print(radarState.rssi);
  display.print("dBm (");
  display.print(radarState.proximityPct);
  display.println("%)");

  display.drawRect(0, 49, SCREEN_WIDTH, 14, COLOR_WHITE);
  int barWidth = map(radarState.proximityPct, 0, 100, 0, SCREEN_WIDTH - 4);
  if (barWidth > 0) {
    display.fillRect(2, 51, barWidth, 10, COLOR_WHITE);
  }
}

void renderMode2Geolocation() {
  drawHeader("M2: GEOLOCALIZACAO");

  if (geoState.anyConfiguredFound && geoState.bestRoomIndex >= 0) {
    display.setCursor(0, 16);
    display.println(ROOM_ANCHORS[geoState.bestRoomIndex].roomName);
    display.drawFastHLine(0, 26, SCREEN_WIDTH, COLOR_WHITE);

    for (int r = 0; r < 3; r++) {
      display.setCursor(0, 30 + (r * 11));
      display.print(r == geoState.bestRoomIndex ? "> " : "  ");
      display.print("AP");
      display.print(r + 1);
      display.print(": ");
      if (geoState.roomRssi[r] <= -100) {
        display.println("Ausente");
      } else {
        display.print(geoState.roomRssi[r]);
        display.println(" dBm");
      }
    }
  } else {
    display.setCursor(0, 16);
    if (geoState.topDemoCount > 0) {
      display.print("Local: ");
      display.println(geoState.topDemoSsid[0].substring(0, 14));
      display.drawFastHLine(0, 26, SCREEN_WIDTH, COLOR_WHITE);

      for (int k = 0; k < geoState.topDemoCount; k++) {
        display.setCursor(0, 30 + (k * 11));
        display.print(k == 0 ? "> " : "  ");
        display.print(geoState.topDemoSsid[k].substring(0, 10));
        display.setCursor(82, 30 + (k * 11));
        display.print(geoState.topDemoRssi[k]);
        display.println("dBm");
      }
    } else {
      display.setCursor(0, 28);
      display.println("Escaneando APs...");
    }
  }
}

void renderMode3HiddenDetector() {
  drawHeader("M3: REDES OCULTAS");

  display.setCursor(0, 16);
  display.print("Ocultas: ");
  display.print(hiddenNetsCount);
  display.print(" / Total: ");
  display.println(totalNetworksLastScan);
  display.drawFastHLine(0, 25, SCREEN_WIDTH, COLOR_WHITE);

  if (hiddenNetsCount == 0) {
    display.setCursor(0, 34);
    display.println("Nenhuma rede Wi-Fi");
    display.println("oculta detectada.");
    return;
  }

  int maxLines = min(hiddenNetsCount, 3);
  for (int i = 0; i < maxLines; i++) {
    int yPos = 28 + (i * 12);
    display.setCursor(0, yPos);
    display.print(hiddenNets[i].bssid);
    display.setCursor(104, yPos);
    display.print("C");
    display.println(hiddenNets[i].channel);
  }
}

void updateOLEDDisplay() {
  // Se o display foi conectado depois de ligar o ESP32 (hot-plug na protoboard), reconecta automaticamente!
  Wire.beginTransmission(detectedOledAddr);
  if (Wire.endTransmission() != 0) {
    oledConnected = false;
    return;
  } else if (!oledConnected) {
    initOLEDDisplay();
  }

  display.clearDisplay();

  switch (currentMode) {
    case MODE_PROXIMITY_RADAR:
      renderMode1Radar();
      break;
    case MODE_INDOOR_LOCATION:
      renderMode2Geolocation();
      break;
    case MODE_HIDDEN_DETECTOR:
      renderMode3HiddenDetector();
      break;
  }

  display.display();
}

// ============================================================================
// 8. LEITURA DO BOTÃO COM DEBOUNCE (ALTERNÂNCIA DOS 3 MODOS)
// ============================================================================
void handleModeButton() {
  bool reading = digitalRead(PIN_BUTTON);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading != buttonStableState) {
      buttonStableState = reading;

      if (buttonStableState == LOW) {
        currentMode = (OperationMode)(((int)currentMode + 1) % 3);

        Serial.print(F("[MODO] Alternado para Modo "));
        Serial.println((int)currentMode + 1);

        beepModeChange(currentMode);
        updateStatusLEDs();
        updateOLEDDisplay();
      }
    }
  }

  lastButtonReading = reading;
}

// ============================================================================
// 9. SETUP E LOOP PRINCIPAL
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n===================================================="));
  Serial.println(F("   INICIANDO PROJETO ESP32 MULTIUSO 3-EM-1 (WIFI)   "));
  Serial.println(F("===================================================="));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_ONBOARD_LED, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, LOW);
  digitalWrite(PIN_ONBOARD_LED, HIGH);

  // Inicializa Barramento I2C nos pinos G21 (SDA) e G22 (SCK)
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

  if (initOLEDDisplay()) {
    display.clearDisplay();
    display.setTextColor(COLOR_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 8);
    display.println(F("WIFI MODO 3-EM-1"));
    display.setCursor(16, 22);
    display.println(F("RASTREADOR ESP32"));
    display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, COLOR_WHITE);
    display.setCursor(8, 44);
    display.println(F("Iniciando Radio..."));
    display.display();
  }

  // Teste rápido dos LEDs e Buzzer ao ligar
  digitalWrite(PIN_LED_1, HIGH); playBuzzerTone(1000); delay(90);
  digitalWrite(PIN_LED_2, HIGH); playBuzzerTone(1500); delay(90);
  digitalWrite(PIN_LED_3, HIGH); playBuzzerTone(2000); delay(90);
  playBuzzerTone(0);
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  WiFi.scanNetworks(true, true);
  scanInProgress = true;
  lastScanTriggerTime = millis();

  Serial.println(F("[OK] Sistema pronto! Pressione o botao no G23 para alternar modos."));
}

void loop() {
  handleModeButton();
  manageAsyncWiFiScan();
  handleRadarBuzzerNonBlocking();
  updateStatusLEDs();

  static unsigned long lastDisplayRefresh = 0;
  if (millis() - lastDisplayRefresh >= 250) {
    lastDisplayRefresh = millis();
    updateOLEDDisplay();
  }
}
