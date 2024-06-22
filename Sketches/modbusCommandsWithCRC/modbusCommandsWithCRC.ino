#include <HardwareSerial.h>
#include <string.h>

const int mdDeRe = 15;  
const int rxPin = 16;
const int txPin = 17;    

HardwareSerial mySerial(2);

// CRC calculation function
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Function to append CRC to the message
void appendCRC(uint8_t* message, size_t length) {
    uint16_t crc = calculateCRC16(message, length);
    message[length] = crc & 0xFF;       // Low byte of CRC
    message[length + 1] = (crc >> 8) & 0xFF; // High byte of CRC
}

void setup() {
  pinMode(mdDeRe, OUTPUT);    
  digitalWrite(mdDeRe, LOW); // Set to receive mode initially
 
  mySerial.begin(9600, SERIAL_8N1, rxPin, txPin);
  Serial.begin(9600);

  initializeMotor();
}

void loop() {

}

void initializeMotor(){
  uint8_t readPeakCurrent[] = {0x01, 0x03, 0x01, 0x91, 0x00, 0x01}; 
  uint8_t motorEnable[] = {0x01, 0x06, 0x18, 0x01, 0x11, 0x22};
  uint8_t setVelocityMode[] = {0x01, 0x06, 0x00, 0x30, 0x00, 0x02}; 
  //uint8_t setSpeed60[] = {0x01, 0x06, 0x01, 0xE1, 0x00, 0x3C};
  uint8_t setSpeed200[] = {0x01, 0x06, 0x01, 0xE1, 0x00, 0xC8};
  //uint8_t setPulse200[] = {0x01, 0x06, 0x01, 0x03, 0x00,0xC8};
  uint8_t startMotor[] = {0x01, 0x06, 0x18, 0x01, 0x40, 0x01};

  // Define the lengths of the original messages
  size_t readPeakCurrentLength = sizeof(readPeakCurrent);
  size_t motorEnableLength = sizeof(motorEnable);
  size_t setVelocityModeLength = sizeof(setVelocityMode);
  //size_t setSpeed60Length = sizeof(setSpeed60);
  size_t setSpeed200Length = sizeof(setSpeed200);
  //size_t setPulse200Length = sizeof(setPulse200);
  size_t startMotorLength = sizeof(startMotor);

  // Create buffers with space for CRC
  uint8_t readPeakCurrentWithCRC[readPeakCurrentLength + 2];
  uint8_t motorEnableWithCRC[motorEnableLength + 2];
  uint8_t setVelocityModeWithCRC[setVelocityModeLength + 2];
  //uint8_t setSpeed60WithCRC[setSpeed60Length + 2];
  uint8_t setSpeed200WithCRC[setSpeed200Length + 2];
  //uint8_t setPulse200WithCRC[setPulse200Length + 2];
  uint8_t startMotorWithCRC[startMotorLength + 2];

  // Copy original messages to buffers
  memcpy(readPeakCurrentWithCRC, readPeakCurrent, readPeakCurrentLength);
  memcpy(motorEnableWithCRC, motorEnable, motorEnableLength);
  memcpy(setVelocityModeWithCRC, setVelocityMode, setVelocityModeLength);
  //memcpy(setSpeed60WithCRC, setSpeed60, setSpeed60Length);
  memcpy(setSpeed200WithCRC, setSpeed200, setSpeed200Length);
  //memcpy(setPulse200WithCRC, setPulse200, setPulse200Length);
  memcpy(startMotorWithCRC, startMotor, startMotorLength);

  // Calculate and append CRC
  appendCRC(readPeakCurrentWithCRC, readPeakCurrentLength);
  appendCRC(motorEnableWithCRC, motorEnableLength);
  appendCRC(setVelocityModeWithCRC, setVelocityModeLength);
  //appendCRC(setSpeed60WithCRC, setSpeed60Length);
  appendCRC(setSpeed200WithCRC, setSpeed200Length);
  //appendCRC(setPulse200WithCRC, setPulse200Length);
  appendCRC(startMotorWithCRC, startMotorLength);

  // Array of messages to be sent
  uint8_t* messages[] = {
    readPeakCurrentWithCRC,
    motorEnableWithCRC,
    setVelocityModeWithCRC,
    //setSpeed60WithCRC,
    setSpeed200WithCRC,
    //setPulse200WithCRC,
    startMotorWithCRC
  };

  // Array of message lengths
  size_t messageLengths[] = {
    readPeakCurrentLength + 2,
    motorEnableLength + 2,
    setVelocityModeLength + 2,
    //setSpeed60Length + 2,
    setSpeed200Length + 2,
    //setPulse200Length + 2,
    startMotorLength + 2
  };

  // Send each message
  for (int i = 0; i < 5; i++) {
    // Set RS485 to transmit mode
    Serial.println("Setting RS485 to transmit mode...");
    digitalWrite(mdDeRe, HIGH); 
    delay(10); // Ensure the pin state is stable

    // Send Modbus message
    Serial.println("Sending Modbus message...");
    mySerial.write(messages[i], messageLengths[i]);
    mySerial.flush(); // Ensure all data is sent
    delay(100); // Short delay to ensure message is sent completely

    // Set RS485 to receive mode
    digitalWrite(mdDeRe, LOW); 
    delay(10); // Short delay to allow for mode switch

    // Read response from slave
    Serial.println("Checking for response...");
    if (mySerial.available()) {
      Serial.println("Data received:");
      while (mySerial.available() > 0) {
        byte incomingByte = mySerial.read();
        Serial.print(incomingByte, HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("No data received.");
    }

    delay(5000); // Delay before sending the next message
  }
}
