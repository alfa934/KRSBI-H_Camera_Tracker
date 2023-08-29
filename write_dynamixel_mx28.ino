#include <Dynamixel2Arduino.h>

#define LED1 6
#define LED2 3

#define DXL_SERIAL   Serial
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID1 = 1;  // Sesuaikan dengan ID servo
const uint8_t DXL_ID2 = 20; // Sesuaikan dengan ID servo
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  // Set Port baudrate to 2000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(2000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 0); // Nanti diganti ke 0
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 0); // Nanti diganti ke 0

  // pinMode for LED1 and LED2
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  int V1 = analogRead(A1);
  int V2 = analogRead(A2);

  int R1 = map(V1, 0, 1023, 0, 90);
  int R2 = map(V2, 0, 1023, 0, 180);

  int L1 = map(V1, 0, 1023, 0 ,255);
  int L2 = map(V2, 0, 1023, 0 ,255);

  delay(1);  // delay in between reads for stability

  dxl.torqueOn(DXL_ID1);
  dxl.torqueOn(DXL_ID2);

  dxl.setGoalPosition(DXL_ID1, R1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, R2, UNIT_DEGREE);

  analogWrite(LED1, L1);
  analogWrite(LED2, L2);

  delay(50);

  dxl.torqueOff(DXL_ID1);
  dxl.torqueOff(DXL_ID2);
}
