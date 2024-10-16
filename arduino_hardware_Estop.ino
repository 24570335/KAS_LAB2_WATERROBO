const int buttonPin = 5;  // Pin where the button is connected
int buttonState = 0;      // Variable to hold the button state
unsigned long counter = 0;
bool wait_after_button_press = false;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // Read the state of the button:
  buttonState = digitalRead(buttonPin);

  // Check if the button is pressed (assuming active HIGH)
  if (buttonState == HIGH || wait_after_button_press == true) {
    // Print "Stop" to the serial port when the button is pressed:
    if (!wait_after_button_press) {
      // Button has just been pressed
      Serial.println("Stop");
      Serial.write(4);
      wait_after_button_press = true;
      counter = millis();  // Resetting counter when button is first pressed
    }

    // If 1 second has passed, reset the button interrupt
    if (millis() - counter >= 1000) {
      wait_after_button_press = false;
    }
  }

  // Sends a message every second even if the button isn't pressed - to keep matlab on
  if (millis() - counter >= 1000) {
    Serial.println(" ");  // Send message every second
    counter = millis();  // Reset the counter for the next message
  }
  
}
