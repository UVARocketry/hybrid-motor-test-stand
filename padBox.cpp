// Initialize Packages
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

// Packet Structure
struct RadioPacket {
  uint16_t data;      
  uint16_t  checksum;  
};

// Bit Definitions
const uint16_t MASK_VALVES = 0x007F; // Bits 0-6 (Switches 1-7)
const uint16_t MASK_KILL   = 0x0080; // Bit 7   (Switch 8)
const uint16_t MASK_STALL  = 0x0100; // Bit 8   (Feedback)
const uint16_t MASK_FULL   = 0x0200; // Bit 9   (Feedback)

// Frequency
const long LORA_FREQUENCY = 915E6; 

// CRC-16 Implementation
uint16_t calculateCRC(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;   // Initial value (common for CRC-16-IBM)

  while (len--) {
    uint8_t extract = *data++;
    for (uint8_t tempI = 8; tempI; tempI--) {
      uint16_t sum = (crc ^ extract) & 0x0001;
      crc >>= 1;
      if (sum) {
        crc ^= 0xA001;    // CRC-16-IBM polynomial
      }
      extract >>= 1;
    }
  }
  return crc;
}

// --- PAD BOX CODE ---

// --- PIN DEFINITIONS ---
// Ball Valves 
const int PIN_BV1 = 6; // D3 (Safe: LoRa IRQ moved to D2)
const int PIN_BV2 = 7; // D4
const int PIN_BV3 = 8; // D5
const int PIN_BV4 = 9; // D6
const int PIN_BV5 = 10; // D7

const uint8_t valveBits[5] = {0, 1, 2, 3, 4};
const uint8_t valvePins[5] = {
  PIN_BV1, PIN_BV2, PIN_BV3, PIN_BV4, PIN_BV5
};
  
Servo valves[5];
bool valveState[5] = {false};

// 24V Switches
const int PIN_24V_SW1 = 19; // D14/A0
const int PIN_24V_SW2 = 20; // D15/A1
const int PIN_24V_SW3 = 21; // D16/A2
const int PIN_24V_SW4 = 22; // D17/A3

// Sensors 
const int PIN_LC_P = 24; // D19/A5
const int PIN_LC_N = 23; // D18/A4
const int PIN_CURRENT = 26; // D21/A7

// LoRa Pins
const int PIN_LORA_CS  = 13; // D10
const int PIN_LORA_RST = 12; // D9
const int PIN_LORA_G0 = 5; // D2

// --- CONFIGURATION ---
// Failsafe Timeout
const int FAILSAFE_TIMEOUT = 1000;

// Stall Current Threshold
const int STALL_CURRENT = 573; // 0-1023 (Current = 6A)
const int MAX_STALL_TIME = 1000; 

// Load Cell Full Tank Threshold
const int FULL_TANK_RAW = 800; // 0-1023 (Weight = #lb)

// Global Variables
uint16_t lastReceivedData = 0xFFFF;
unsigned long lastPacketTime = 0;
bool connectionActive = false;

// Stall Logic
unsigned long highCurrentStartTime = 0;
bool stallDetected = false;

// Valve PWM Control
const int valveOpenHighTime = 1500;
const int valveClosedHighTime = 500;

void setup() {
  Serial.begin(9600); 

  // Setup Outputs
  pinMode(PIN_BV1, OUTPUT); pinMode(PIN_BV2, OUTPUT);
  pinMode(PIN_BV3, OUTPUT); pinMode(PIN_BV4, OUTPUT);
  pinMode(PIN_BV5, OUTPUT);

  for (int i = 0; i < 5; i++) {
    valves[i].attach(valvePins[i]);
    valves[i].writeMicroseconds(valveClosedHighTime);
  }

  pinMode(PIN_24V_SW1, OUTPUT); pinMode(PIN_24V_SW2, OUTPUT);
  pinMode(PIN_24V_SW3, OUTPUT); pinMode(PIN_24V_SW4, OUTPUT);

  // Initialize LoRa
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_G0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed.");
    while (1);
  }
  
  triggerFailsafe();
}

void loop() {
  // 1. Stall Detection (Main Valve assumed to be BV1 or BV5?)
  // This checks global current. If ANY valve movement spikes current too long:
  if (analogRead(PIN_CURRENT) > STALL_CURRENT) {
    if (highCurrentStartTime == 0) {
      highCurrentStartTime = millis();
    }
    if (millis() - highCurrentStartTime > MAX_STALL_TIME) {
      stallDetected = true;
    }
  } else {
    highCurrentStartTime = 0;
    stallDetected = false;
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
  if (connectionActive && (millis() - lastPacketTime > FAILSAFE_TIMEOUT)) {
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
  for (int i = 0; i < 5; i++) {
    bool cmdOpen = cmd & (1 << valveBits[i]);
    if (cmdOpen && !valveState[i]) {
      valves[i].writeMicroseconds(valveOpenHighTime);
      valveState[i] = true;
    }
    else if (!cmdOpen && valveState[i]) {
      valves[i].writeMicroseconds(valveClosedHighTime);
      valveState[i] = false;
    }
  }

  // Map Bits 5-6 to 24V Switches 1-2
  digitalWrite(PIN_24V_SW1, (cmd & (1 << 5)) ? HIGH : LOW);
  digitalWrite(PIN_24V_SW2, (cmd & (1 << 6)) ? HIGH : LOW);
}

void sendConfirmation(uint16_t echo) {
  uint16_t reply = echo;

  if (stallDetected) {
    reply |= MASK_STALL;
  }

  int load = analogRead(PIN_LC_P) - analogRead(PIN_LC_N);
  if (load >= FULL_TANK_RAW) {
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
  for (int i = 0; i < 5; i++) {
    valves[i].writeMicroseconds(valveClosedHighTime);
    valveState[i] = false;
  }
  
  digitalWrite(PIN_24V_SW1, HIGH); //Assuming Vent Valve
  digitalWrite(PIN_24V_SW2, LOW);
  digitalWrite(PIN_24V_SW3, LOW);
  digitalWrite(PIN_24V_SW4, LOW);
}
