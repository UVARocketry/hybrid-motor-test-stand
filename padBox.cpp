// --- SHARED SETTINGS (COPY TO BOTH SKETCHES) ---
#include <SPI.h>
#include <LoRa.h>

// Packet Structure
struct RadioPacket {
  uint16_t data;      
  uint8_t  checksum;  
};

// Bit Definitions
const uint16_t MASK_VALVES = 0x007F; // Bits 0-6 (Switches 1-7)
const uint16_t MASK_KILL   = 0x0080; // Bit 7   (Switch 8)
const uint16_t MASK_STALL  = 0x0100; // Bit 8   (Feedback)
const uint16_t MASK_FULL   = 0x0200; // Bit 9   (Feedback)

// Frequency
const long LORA_FREQUENCY = 915E6; 

// CRC-8 Implementation
uint8_t calculateCRC(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    uint8_t extract = *data++;
    for (uint8_t tempI = 8; tempI; tempI--) {
      uint8_t sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) crc ^= 0x8C;
      extract >>= 1;
    }
  }
  return crc;
}

// --- PAD BOX CODE ---
// [Include Shared Settings Here]

// --- PIN DEFINITIONS ---
// Ball Valves 
const int PIN_BV1 = 3; // D3 (Safe: LoRa IRQ moved to D2)
const int PIN_BV2 = 4; // D4
const int PIN_BV3 = 5; // D5
const int PIN_BV4 = 6; // D6
const int PIN_BV5 = 7; // D7

// 24V Switches (Aux Valves) - FLIPPED ORDER 
const int PIN_24V_SW1 = A0; // Was A3
const int PIN_24V_SW2 = A1; // Was A2
const int PIN_24V_SW3 = A2; // Was A1
const int PIN_24V_SW4 = A3; // Was A0

// Sensors 
const int PIN_LOAD_CELL_P = A5; // D19/A5
const int PIN_CURRENT     = A7; // D21/A7

// LoRa Pins [cite: 5]
const int LORA_CS  = 10;
const int LORA_RST = 9;
const int LORA_IRQ = 2; // User specified: G0 is Pin 5 (D2)

// --- CONFIGURATION ---
const int FAILSAFE_TIMEOUT_MS = 1000;

// CALIBRATION REQUIRED:
// 8A Stall Current[cite: 1].
// You must test what value (0-1023) your sensor reads at 8A.
const int STALL_CURRENT_RAW = 400; 
const unsigned long MAX_TRAVEL_TIME_MS = 2000; 

// Load Cell Full Tank Threshold (0-1023)
const int FULL_TANK_RAW = 800; 

// Global Variables
uint16_t lastReceivedData = 0xFFFF;
unsigned long lastPacketTime = 0;
bool connectionActive = false;

// Stall Logic
unsigned long highCurrentStartTime = 0;
bool stallDetected = false;

void setup() {
  Serial.begin(9600); 

  // Setup Outputs
  pinMode(PIN_BV1, OUTPUT); pinMode(PIN_BV2, OUTPUT);
  pinMode(PIN_BV3, OUTPUT); pinMode(PIN_BV4, OUTPUT);
  pinMode(PIN_BV5, OUTPUT);
  
  pinMode(PIN_24V_SW1, OUTPUT); pinMode(PIN_24V_SW2, OUTPUT);
  pinMode(PIN_24V_SW3, OUTPUT); pinMode(PIN_24V_SW4, OUTPUT);

  // Initialize LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed.");
    while (1);
  }
  
  triggerFailsafe();
}

void loop() {
  // 1. Stall Detection (Main Valve assumed to be BV1 or BV5?)
  // This checks global current. If ANY valve movement spikes current too long:
  if (analogRead(PIN_CURRENT) > STALL_CURRENT_RAW) {
    if (highCurrentStartTime == 0) {
      highCurrentStartTime = millis();
    }
    if (millis() - highCurrentStartTime > MAX_TRAVEL_TIME_MS) {
      stallDetected = true;
    }
  } else {
    highCurrentStartTime = 0;
  }

  // 2. Radio Handling
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(RadioPacket)) {
    RadioPacket rxPacket;
    LoRa.readBytes((uint8_t*)&rxPacket, sizeof(rxPacket));

    if (calculateCRC((uint8_t*)&rxPacket.data, sizeof(rxPacket.data)) == rxPacket.checksum) {
      connectionActive = true;
      lastPacketTime = millis();

      uint16_t cmd = rxPacket.data & (MASK_VALVES | MASK_KILL);
      uint16_t lastCmd = lastReceivedData & (MASK_VALVES | MASK_KILL);

      // Double Confirmation Logic
      if (cmd == lastCmd) {
        executeCommand(cmd);
      }
      lastReceivedData = rxPacket.data;

      sendConfirmation(cmd);
    }
  }

  // 3. Failsafe Timer
  if (connectionActive && (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS)) {
    triggerFailsafe();
    connectionActive = false;
  }
}

void executeCommand(uint16_t cmd) {
  if (cmd & MASK_KILL) {
    triggerFailsafe();
    return;
  }

  // Map Bits 0-4 to Ball Valves 1-5
  digitalWrite(PIN_BV1, (cmd & (1 << 0)) ? HIGH : LOW);
  digitalWrite(PIN_BV2, (cmd & (1 << 1)) ? HIGH : LOW);
  digitalWrite(PIN_BV3, (cmd & (1 << 2)) ? HIGH : LOW);
  digitalWrite(PIN_BV4, (cmd & (1 << 3)) ? HIGH : LOW);
  digitalWrite(PIN_BV5, (cmd & (1 << 4)) ? HIGH : LOW);

  // Map Bits 5-6 to 24V Switches 1-2
  digitalWrite(PIN_24V_SW1, (cmd & (1 << 5)) ? HIGH : LOW);
  digitalWrite(PIN_24V_SW2, (cmd & (1 << 6)) ? HIGH : LOW);
}

void sendConfirmation(uint16_t echo) {
  uint16_t reply = echo;

  if (stallDetected) reply |= MASK_STALL;
  
  if (analogRead(PIN_LOAD_CELL_P) >= FULL_TANK_RAW) {
    reply |= MASK_FULL;
  }

  RadioPacket txPacket;
  txPacket.data = reply;
  txPacket.checksum = calculateCRC((uint8_t*)&txPacket.data, sizeof(txPacket.data));
  
  delay(2);
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&txPacket, sizeof(txPacket));
  LoRa.endPacket();
}

void triggerFailsafe() {
  // Close everything
  digitalWrite(PIN_BV1, LOW);
  digitalWrite(PIN_BV2, LOW);
  digitalWrite(PIN_BV3, LOW);
  digitalWrite(PIN_BV4, LOW);
  digitalWrite(PIN_BV5, LOW);
  
  digitalWrite(PIN_24V_SW1, LOW);
  digitalWrite(PIN_24V_SW2, LOW);
  digitalWrite(PIN_24V_SW3, LOW);
  digitalWrite(PIN_24V_SW4, LOW);

  // SAFETY: If you need a specific valve to OPEN (Vent), add that here.
  // Example: digitalWrite(PIN_BV5, HIGH); 
}