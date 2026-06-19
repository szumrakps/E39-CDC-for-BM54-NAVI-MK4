#include <Arduino.h>
#include "ESP_I2S.h"
#include "BluetoothA2DPSink.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =====================================================
// ===================== I-BUS =========================
// =====================================================

#define IBUS_RX 21
#define IBUS_TX 19

HardwareSerial IBUS(2);

// ===== RX BUFFER =====
uint8_t buf[32];
uint8_t idx = 0;
uint8_t frameLen = 0;

// ===== STATE =====
uint8_t state = 0x02; // Startujemy jako paused
uint8_t lastCmd = 0x89;

uint8_t trackDec = 1;
uint8_t trackHex = 0x01;

uint8_t discs = 0x3F;
uint8_t disc  = 0x01;

bool cdcInitialized = false;
bool isCdcActive = false; 

bool pendingAnnounce = false;
unsigned long announceTime = 0;
uint8_t announceCount = 0;

// Bufory na metadane
char currentTitle[65] = "NO TITLE";
char currentArtist[65] = "NO ARTIST"; 
uint8_t titleOffset = 0; 

// Mutex do bezpiecznego przesyłania tekstu między rdzeniami
SemaphoreHandle_t metaMutex;

bool pendingPowerStatus = false;
unsigned long powerStatusTime = 0;
uint8_t powerStatusCount = 0;

bool pendingTitle = false;
unsigned long titleTime = 0;
bool trackChanged = false;

uint8_t textSendCount = 0; 
unsigned long lastTextRefresh = 0;
static bool textSent = false;

volatile bool textSeqStart = false; // volatile, bo modyfikowane w innym rdzeniu
unsigned long lastBtCmd = 0;

unsigned long lastIBUSActivity = 0;
unsigned long lastRadioPing = 0; 

bool cdcReentrySeq = false;
uint8_t cdcReentryStep = 0;
unsigned long cdcReentryTime = 0;

volatile bool audioLock = false;
volatile unsigned long audioLockTime = 0;

// Flagi komunikacji między rdzeniami
volatile bool btPlayPending = false;
volatile bool btPausePending = false;
volatile bool btNextPending = false;
volatile bool btPrevPending = false;

enum TextState {
    TEXT_IDLE,
    TEXT_INIT,
    TEXT_SEND_LINE1,
    TEXT_SEND_LINE2,
    TEXT_END
};

TextState textState = TEXT_IDLE;
unsigned long textTime = 0;

// =====================================================
// ===================== BT / I2S ======================
// =====================================================

const uint8_t I2S_SCK   = 26;
const uint8_t I2S_WS    = 25;
const uint8_t I2S_SDOUT = 22;

I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);
TaskHandle_t AudioTaskHandle;

// Deklaracje funkcji
uint8_t calcXOR(uint8_t *d, int len);
void sendFrame(uint8_t *f, uint8_t len, bool critical = false);
void sendStatus();
void sendEvent(uint8_t ev);
void sendText();
void sendArtist(); 
void sendEndFrame();
bool busIdle();
void processFrame();
void metadata_callback(uint8_t id, const uint8_t *text);

// =====================================================
// ===================== XOR ===========================
// =====================================================

uint8_t calcXOR(uint8_t *d, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= d[i];
    return cs;
}

// =====================================================
// ================== I-BUS TX =========================
// =====================================================

void sendFrame(uint8_t *f, uint8_t len, bool critical) {
    if (audioLock && !critical) return;

    unsigned long start = millis();
    while (!busIdle()) {
        if (millis() - start > 20) return; 
    }

    IBUS.write(f, len);
    lastIBUSActivity = millis();
}

void sendStatus() {
    uint8_t f[16];
    f[0]=0x76; f[1]=0x0E; f[2]=0x68; f[3]=0x39;
    f[4]=state;
    f[5]=lastCmd;
    f[6]=0x00;
    f[7]=discs;
    f[8]=0x00;
    f[9]=disc;
    f[10]=trackHex;
    f[11]=0x00;
    f[12]=0x01;
    f[13]=0x01;
    f[14]=trackDec;
    f[15]=calcXOR(f,15);

    sendFrame(f, 16, true); 
}

void sendEvent(uint8_t ev) {
    uint8_t f[16];
    f[0]=0x76; f[1]=0x0E; f[2]=0x68; f[3]=0x39;
    f[4]=ev;
    f[5]=lastCmd;
    f[6]=0x00;
    f[7]=discs;
    f[8]=0x00;
    f[9]=disc;
    f[10]=trackHex;
    f[11]=0x00;
    f[12]=0x01;
    f[13]=0x01;
    f[14]=trackDec;
    f[15]=calcXOR(f,15);

    sendFrame(f, 16, true); 
}

// =====================================================
// ================= PARSER ============================
// =====================================================

void processFrame() {
    if (frameLen < 4) return;

    if (buf[0]==0x68 && buf[2]==0xF0 && buf[3]==0x4A) {
        pendingPowerStatus = true;
        powerStatusTime = millis();
        powerStatusCount = 0;
        return;
    }

    if (buf[0]==0x80 && buf[2]==0xBF && buf[3]==0x11) {
        uint8_t acc = buf[4];
        if (acc == 0x01 || acc == 0x03) {
            if (!cdcInitialized) {
                pendingAnnounce = true;
                announceTime = millis();
                announceCount = 0;
                cdcInitialized = true;
            }
        } else if (acc == 0x00) {
            cdcInitialized = false;
            isCdcActive = false;
        }
        return;
    }

    if (buf[0]==0x68 && buf[2]==0x3B && buf[3]==0xA5 &&
        buf[4]==0x63 && buf[5]==0x01 && buf[6]==0x00) {
        if (textSendCount < 3) {
            textState = TEXT_INIT;
            textTime = millis();
        }
        return;
    }

    if (buf[0]==0x68 && buf[2]==0x76 && buf[3]==0x38 && buf[4]==0x00) {
        lastRadioPing = millis(); 
        cdcInitialized = true;
        sendStatus();
        return;
    }

    if (buf[0]==0x68 && buf[2]==0x76 && buf[3]==0x38) {
        uint8_t cmd = buf[4];
        uint8_t sub = buf[5];    

        if (cmd == 0x03) {
            isCdcActive = true;
            pendingTitle = true;
            titleTime = millis();
            
            pendingAnnounce = true;
            announceCount = 0;
            announceTime = millis(); 

            state = 0x01; 
            lastCmd = 0x82;
            
            cdcReentrySeq = true;
            cdcReentryStep = 0;
            cdcReentryTime = millis(); 

            textSendCount = 0; 
            return;
        }

        if (cmd == 0x01) {
            isCdcActive = false;
            cdcReentrySeq = false; 
            state = 0x01; 
            lastCmd = 0x82;
            btPausePending = true; // Zlecenie pauzy do Rdzenia 0
            sendStatus();
            return;
        }

        if (cmd == 0x0A) {
            if (sub == 0x00) {
                trackDec++;
                if (trackDec > 99) trackDec = 1;
                trackHex = ((trackDec / 10) << 4) | (trackDec % 10);
                trackChanged = true;
                btNextPending = true; // Zlecenie NEXT do Rdzenia 0
                sendEvent(0x07);

                pendingPowerStatus = true;
                powerStatusTime = millis();
                powerStatusCount = 0;
                pendingTitle = true;
                titleTime = millis();

                textSendCount = 0; 
            }
            if (sub == 0x01) {
                if (trackDec <= 1) trackDec = 99;
                else trackDec--;
                trackHex = ((trackDec / 10) << 4) | (trackDec % 10);
                trackChanged = true;
                btPrevPending = true; // Zlecenie PREV do Rdzenia 0
                sendEvent(0x07);

                pendingPowerStatus = true;
                powerStatusTime = millis();
                powerStatusCount = 0;
                pendingTitle = true;
                titleTime = millis();

                textSendCount = 0; 
            }
            state = 0x02;
        }

        if (cmd == 0x82) { state = 0x01; lastCmd = 0x82; }
        if (cmd == 0x89) { state = 0x02; lastCmd = 0x89; }
        if (cmd == 0x8C) { state = 0x00; lastCmd = 0x8C; }
    }
}

// =====================================================
// ================= METADATA CALLBACK =================
// =====================================================

void metadata_callback(uint8_t id, const uint8_t *text) {
    if (id == ESP_AVRC_MD_ATTR_TITLE) {
        xSemaphoreTake(metaMutex, portMAX_DELAY); // Blokada zapisu dla rdzenia 1
        strncpy(currentTitle, (const char*)text, 64);
        currentTitle[64] = '\0';

        for (int i = 0; currentTitle[i]; i++) {
            if (currentTitle[i] >= 'a' && currentTitle[i] <= 'z') {
                currentTitle[i] -= 32;
            }
        }
        xSemaphoreGive(metaMutex); // Zwolnienie blokady

        textSendCount = 0; 
        if (isCdcActive) textSeqStart = true;
        btPlayPending = true; // Odpal play dopiero po otrzymaniu metadanych!
    }
    
    if (id == ESP_AVRC_MD_ATTR_ARTIST) {
        xSemaphoreTake(metaMutex, portMAX_DELAY); // Blokada zapisu
        strncpy(currentArtist, (const char*)text, 64);
        currentArtist[64] = '\0';

        for (int i = 0; currentArtist[i]; i++) {
            if (currentArtist[i] >= 'a' && currentArtist[i] <= 'z') {
                currentArtist[i] -= 32;
            }
        }
        xSemaphoreGive(metaMutex); // Zwolnienie blokady

        if (isCdcActive && textSendCount < 3) textSeqStart = true;
    }
}

void bt_connection_state_changed(esp_a2d_connection_state_t bt_state, void *ptr) {
    if (bt_state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        audioLock = true;
        audioLockTime = millis();

        state = 0x02; // Podłączenie ustawia stan na pauzę, Play dopiero po metadanych
        if (isCdcActive) sendStatus();
    }
}

// =====================================================
// ============== AUDIO TASK (CORE 0) ==================
// =====================================================

void audioTaskCode(void * pvParameters) {
    // Inicjalizacja Audio na osobnym rdzeniu
    i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT);
    i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);
    
    a2dp_sink.set_auto_reconnect(true);
    a2dp_sink.set_on_connection_state_changed(bt_connection_state_changed);
    a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
    a2dp_sink.set_avrc_metadata_callback(metadata_callback);
    a2dp_sink.start("BMW CDC BT", true);

    for(;;) {
        // Odbieranie żądań z rdzenia I-BUS
        if (btNextPending) {
            audioLock = true;
            audioLockTime = millis();
            a2dp_sink.next();
            btNextPending = false;
        }
        if (btPrevPending) {
            audioLock = true;
            audioLockTime = millis();
            a2dp_sink.previous();
            btPrevPending = false;
        }
        if (btPlayPending) {
            a2dp_sink.play();
            btPlayPending = false;
        }
        if (btPausePending) {
            a2dp_sink.pause();
            btPausePending = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // Zapobiega zablokowaniu Watchdoga
    }
}

// =====================================================
// ===================== SETUP =========================
// =====================================================

void setup() {
    Serial.begin(115200);
    IBUS.begin(9600, SERIAL_8E1, IBUS_RX, IBUS_TX);
    IBUS.setRxBufferSize(256);

    metaMutex = xSemaphoreCreateMutex();

    pendingAnnounce = true;
    announceTime = millis();
    announceCount = 0;
    lastRadioPing = millis();

    // Tworzenie zadania Audio na rdzeniu 0 (PRO_CPU)
    xTaskCreatePinnedToCore(
        audioTaskCode,   /* Funkcja Taska */
        "AudioTask",     /* Nazwa */
        10000,           /* Wielkość stosu */
        NULL,            /* Parametry */
        1,               /* Priorytet */
        &AudioTaskHandle,/* Uchwyt */
        0                /* Przypięcie do rdzenia 0 */
    );
    
    // RDZEŃ 1: Setup dla I-BUS domyślnie jest już zakończony i przechodzi do pętli loop()
}

bool busIdle() {
    return (millis() - lastIBUSActivity) > 15; 
}

// =====================================================
// ================== LOOP (CORE 1) ====================
// =====================================================

void loop() {

    while (IBUS.available()) {
        lastIBUSActivity = millis();  
        uint8_t b = IBUS.read();

        if (idx == 0) { buf[0] = b; idx = 1; continue; }
        if (idx == 1) {
            frameLen = b;
            if (frameLen < 2 || frameLen > 40) { idx = 0; continue; }
            buf[1] = b; idx = 2; continue;
        }

        buf[idx++] = b;

        if (idx >= frameLen + 2) {
            if (calcXOR(buf, idx - 1) == buf[idx - 1]) {
                processFrame();
            }
            idx = 0;
        }
    }

    if (!isCdcActive && (millis() - lastRadioPing > 10000)) {
        pendingAnnounce = true;
        announceCount = 0;
        announceTime = millis();
        lastRadioPing = millis(); 
    }

    if (pendingTitle && millis() - titleTime > 300) {
        if (isCdcActive && textSendCount < 3) textSeqStart = true;
        pendingTitle = false;
    }

    if (audioLock && millis() - audioLockTime > 80) {
        audioLock = false;
    }

    if (pendingAnnounce && millis() - announceTime > 60) {
        uint8_t f[6] = {0x76, 0x04, 0xFF, 0x02, 0x01, 0x8E};
        sendFrame(f, 6, true); 
        announceCount++;
        announceTime = millis();

        if (announceCount >= 4) { 
            pendingAnnounce = false;
            sendStatus(); // Wyśle status 0x02, czyli pauzę zaraz po annonsense
        }
    }

    if (pendingPowerStatus && millis() - powerStatusTime > 50) {
        sendStatus();
        powerStatusCount++;
        powerStatusTime = millis();

        if (powerStatusCount >= 2) {
            pendingPowerStatus = false;
        }
    }

// --- MASZYNA STANÓW TEKSTU ---
    if (textSeqStart) {
        textSeqStart = false;
        textState = TEXT_INIT;
        textTime = millis();
    }

    if (textState != TEXT_IDLE) { 
        switch (textState) {
            case TEXT_INIT:
                if (millis() - textTime >= 30) {
                    if (!busIdle()) {
                        textTime = millis(); 
                        break;   
                    }
                    sendArtist(); 
                    if (!isCdcActive) {
                        textState = TEXT_IDLE;
                    } else {
                        textState = TEXT_SEND_LINE1;
                        textTime = millis();
                    }
                }
                break;

            case TEXT_SEND_LINE1:
                if (millis() - textTime >= 30) {
                    if (!busIdle()) {
                        textTime = millis(); 
                        break;   
                    }
                    sendText(); 
                    textState = TEXT_SEND_LINE2;
                    textTime = millis();
                }
                break;
                        
            case TEXT_SEND_LINE2:
                if (millis() - textTime >= 30) {
                    if (!busIdle()) {
                        textTime = millis(); 
                        break;   
                    }
                    sendEndFrame(); 
                    textState = TEXT_END;
                    textTime = millis();
                }
                break;

            case TEXT_END:
                if (millis() - textTime >= 30) {
                    textSendCount++; 
                    textState = TEXT_IDLE;
                }
                break;
        }
    }

    if (cdcReentrySeq && millis() - cdcReentryTime > 200) { 
        if (pendingAnnounce) return; 

        if (cdcReentryStep == 0) {
            state = 0x01; lastCmd = 0x82;
            sendStatus();
            cdcReentryStep = 1;
            cdcReentryTime = millis();
        }
        else if (cdcReentryStep == 1) {
            state = 0x02; lastCmd = 0x89;
            sendStatus();
            btPlayPending = true; // Zlecenie play do rdzenia Audio po sekwencji powrotu
            cdcReentrySeq = false; 
        }
    }
}

// =====================================================
// ================= TEXT GENERATION ===================
// =====================================================

void sendText() {
    xSemaphoreTake(metaMutex, portMAX_DELAY);
    uint8_t len = strlen(currentTitle);
    const uint8_t MAX_FIRST_LEN = 30; 

    if (len > MAX_FIRST_LEN) {
        int splitIdx = MAX_FIRST_LEN;
        while (splitIdx > 0 && currentTitle[splitIdx] != ' ') {
            splitIdx--;
        }
        if (splitIdx == 0) splitIdx = MAX_FIRST_LEN;

        titleOffset = (currentTitle[splitIdx] == ' ') ? splitIdx + 1 : splitIdx;
        len = splitIdx;
    } else {
        titleOffset = 0; 
    }

    uint8_t f[64]; 
    f[0] = 0x76; f[1] = 4 + len; f[2] = 0x68; f[3] = 0x3F; f[4] = 0x44;

    for (int i = 0; i < len; i++) {
        f[5 + i] = currentTitle[i];
    }
    xSemaphoreGive(metaMutex);

    uint8_t total = 5 + len;
    f[total] = calcXOR(f, total);
    sendFrame(f, total + 1);
}

void sendEndFrame() {
    xSemaphoreTake(metaMutex, portMAX_DELAY);
    if (titleOffset > 0 && titleOffset < strlen(currentTitle)) {
        uint8_t remLen = strlen(currentTitle) - titleOffset;
        if (remLen > 30) remLen = 30; 

        uint8_t f[64]; 
        f[0] = 0x76; f[1] = 4 + remLen; f[2] = 0x68; f[3] = 0x3F; f[4] = 0x48; 

        for (int i = 0; i < remLen; i++) {
            f[5 + i] = currentTitle[titleOffset + i];
        }
        xSemaphoreGive(metaMutex);

        uint8_t total = 5 + remLen;
        f[total] = calcXOR(f, total);
        sendFrame(f, total + 1);
    } else {
        xSemaphoreGive(metaMutex);
        uint8_t f[7] = {0x76, 0x05, 0x68, 0x3F, 0x08, 0x00, 0x00};
        f[6] = calcXOR(f, 6);
        sendFrame(f, 7);
    }
}

void sendArtist() {
    xSemaphoreTake(metaMutex, portMAX_DELAY);
    uint8_t len = strlen(currentArtist);
    if (len > 30) len = 30; 

    uint8_t f[64]; 
    f[0] = 0x76; f[1] = 4 + len; f[2] = 0x68; f[3] = 0x3F; f[4] = 0x42; 

    for (int i = 0; i < len; i++) {
        f[5 + i] = currentArtist[i];
    }
    xSemaphoreGive(metaMutex);

    uint8_t total = 5 + len;
    f[total] = calcXOR(f, total);
    sendFrame(f, total + 1);
}