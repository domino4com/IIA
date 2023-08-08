#include <ArduinoJson.h>
#include <Wire.h>
#ifndef I2C_SDA
#define I2C_SDA SDA
#endif
#ifndef I2C_SCL
#define I2C_SCL SCL
#endif

// Specifics
#include <IIA.h>
IIA input;
float x, y, z;
char s[] = "X: %.2f g, Y: %.2f g, Z: %.2f g\n";

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.printf("\nIIA Example Test\n");

    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();

    if (input.begin()) {
        Serial.println("IIA initialized successfully.");
    } else {
        Serial.println("Failed to initialize IIA!");
        exit(0);
    }
}

void loop() {
    if (input.getData(x, y, z)) {
        Serial.printf(s, x, y, z);
    } else {
        Serial.println("Failed to get IIA data.");
    }

    delay(1000);
}