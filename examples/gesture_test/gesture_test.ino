// Basic demo for accelerometer readings from the APDS9500
#include <Wire.h>
#include <Adafruit_APDS9500.h>

Adafruit_APDS9500 apds;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit APDS9500 test!");

  // Try to initialize!
  if (! apds.begin()) {
    Serial.println("Failed to find APDS9500 chip");
    while (1) { delay(10); }
  }
  Serial.println("APDS9500 Found!");

}

void loop() {

  uint16_t gesture_flags = apds.getDetectedGestures();
  if(gesture_flags == -1){
    Serial.println("Issue reading gesture state");
  }

  if(gesture_flags & 0x01) Serial.println("UP event detected");
  if(gesture_flags & 0x02) Serial.println("DOWN event detected");
  if(gesture_flags & 0x04) Serial.println("LEFT event detected");
  if(gesture_flags & 0x08) Serial.println("RIGHT event detected");
  if(gesture_flags & 0x10) Serial.println("FORWARD event detected");
  if(gesture_flags & 0x20) Serial.println("BACKWARD event detected");
  if(gesture_flags & 0x40) Serial.println("CLOCKWISE event detected");
  if(gesture_flags & 0x80) Serial.println("COUNTERCLOCKWISE event detected");

  if((gesture_flags >> 8) & 0x01) Serial.println("WAVE event detected");
  if((gesture_flags >> 8) & 0x02) Serial.println("PROXIMITY event detected");
  if((gesture_flags >> 8) & 0x04) Serial.println("HAS OBJECT event detected");
  if((gesture_flags >> 8) & 0x08) Serial.println("WAKE UP TRIGGER event detected");
  if((gesture_flags >> 8) & 0x80) Serial.println("NO OBJECT event detected");

  delay(100);
}
