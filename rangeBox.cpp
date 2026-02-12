// --- SHARED SETTINGS (COPY TO BOTH SKETCHES) ---
#include <SPI.h>
#include <LoRa.h>

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
      };
      extract >>= 1;
    }
  }
  return crc;
}


// --- RANGE BOX CODE ---

// --- PIN DEFINITIONS ---
// Switches 1-8
const int PIN_SW1 = 19; // D14/A0
const int PIN_SW2 = 20; // D15/A1
const int PIN_SW3 = 21; // D16/A2
const int PIN_SW4 = 22; // D17/A3 
const int PIN_SW5 = 7; // D4
const int PIN_SW6 = 8; // D5
const int PIN_SW7 = 9; // D6
const int PIN_SW8 = 10; // D7

// LEDs 
const int PIN_LED1_DESYNC = 5; // D2 (LED 1)
const int PIN_LED2_STALL  = 1; // D1 (LED 2)
const int PIN_LED3_FULL   = 2; // D0 (LED 3)

// LoRa Pins 
const int PIN_LORA_CS  = 13; // D10
const int PIN_LORA_RST = 12; // D9
const int PIN_LORA_G0 = 6; // D3

// --- CONFIGURATION ---
const int TIMEOUT = 50;
const int MAX_MISSED_ACKS = 5;
int missedAckCount = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup Switch Inputs
  pinMode(PIN_SW1, INPUT_PULLUP); pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_SW3, INPUT_PULLUP); pinMode(PIN_SW4, INPUT_PULLUP);
  pinMode(PIN_SW5, INPUT_PULLUP); pinMode(PIN_SW6, INPUT_PULLUP);
  pinMode(PIN_SW7, INPUT_PULLUP); pinMode(PIN_SW8, INPUT_PULLUP);

  // Setup LED Outputs
  pinMode(PIN_LED1_DESYNC, OUTPUT);
  pinMode(PIN_LED2_STALL,  OUTPUT);
  pinMode(PIN_LED3_FULL,   OUTPUT);

  // Initialize LoRa
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_G0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    while(1) {
      digitalWrite(PIN_LED_DESYNC, HIGH); delay(100);
      digitalWrite(PIN_LED_DESYNC, LOW); delay(100);
    }
  }
}

void loop() {
  // 1. Read Inputs & Build Packet
  uint16_t commandData = 0;

  if (digitalRead(PIN_SW1) == LOW) commandData |= (1 << 0);
  if (digitalRead(PIN_SW2) == LOW) commandData |= (1 << 1);
  if (digitalRead(PIN_SW3) == LOW) commandData |= (1 << 2);
  if (digitalRead(PIN_SW4) == LOW) commandData |= (1 << 3);
  if (digitalRead(PIN_SW5) == LOW) commandData |= (1 << 4);
  if (digitalRead(PIN_SW6) == LOW) commandData |= (1 << 5);
  if (digitalRead(PIN_SW7) == LOW) commandData |= (1 << 6);
  
  // Kill Switch
  if (digitalRead(PIN_SW8) == LOW) commandData |= MASK_KILL;

  // 2. Send Packet
  RadioPacket txPacket;
  txPacket.data = commandData;
  txPacket.checksum = calculateCRC((uint8_t*)&txPacket.data, sizeof(txPacket.data));

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&txPacket, sizeof(txPacket));
  LoRa.endPacket();

  // 3. Wait for Confirmation
  bool responseReceived = false;
  unsigned long waitStart = millis();

  while (millis() - waitStart < TIMEOUT) {
    int packetSize = LoRa.parsePacket();
    if (packetSize == sizeof(RadioPacket)) {
      RadioPacket rxPacket;
      LoRa.readBytes((uint8_t*)&rxPacket, sizeof(rxPacket));

      if (calculateCRC((uint8_t*)&rxPacket.data, sizeof(rxPacket.data)) == rxPacket.checksum) {
        uint16_t sentCmd = commandData & (MASK_VALVES | MASK_KILL);
        uint16_t recvEcho = rxPacket.data & (MASK_VALVES | MASK_KILL);

        if (sentCmd == recvEcho) {
          responseReceived = true;
          missedAckCount = 0;
          digitalWrite(PIN_LED_DESYNC, LOW);
          
          digitalWrite(PIN_LED_STALL, (rxPacket.data & MASK_STALL) ? HIGH : LOW);
          digitalWrite(PIN_LED_FULL,  (rxPacket.data & MASK_FULL)  ? HIGH : LOW);
          break;
        }
      }
    }
  }

  if (!responseReceived) {
    missedAckCount++;
    if (missedAckCount >= MAX_MISSED_ACKS) digitalWrite(PIN_LED_DESYNC, HIGH);
  }
  
  delay(10); 

}
