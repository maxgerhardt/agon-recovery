#include "serial.h"
#include "message.h"


void displayMessage(const char *msg) {
    Serial.printf(msg);
}

void displayError(const char *msg) {
    displayMessage(msg);
}
