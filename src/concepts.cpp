#include <Arduino.h>

#include <EEPROM.h>

/**
 * Software dingen:
 * - 3x input fout, tijdslot
 * -
 *
 */

int currentPincode = -1; // Represents the current pincode of the safe.

typedef struct {
    int value;
} Pincode;

/**
 * This enum type definition will be used to determine if the safe is either closed or open.
 * Using this enum, we will have a clear way to tell in which state the safe currently is
 */
typedef enum {
    SAFE_OPEN, SAFE_CLOSED, BLOCKED
} State;

void writePincode(Pincode pincode) {
    for (int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0);
    EEPROM.put(0, pincode);
}

void resetPincode() {
    EEPROM.put(0, -1);
}

Pincode readPincode() {
    Pincode pincode = {.value = -1};
    EEPROM.get(0, pincode);

    return pincode;
}

void setup() {
    Serial.begin(9600);
    while (!Serial) { ; /* wait for serial port to connect.*/ }

    if (readPincode().value != -1) {
        Serial.print("Found a pincode! Value: ");
        Serial.println(readPincode().value);
    } else {
        Pincode pincode = {
                .value = 9922
        };

        Serial.println("Pincode is -1");
        writePincode(pincode);
    }
}

void loop() {
    /* empty loop function */
}