// Button test: prints the name of the pressed button to the serial monitor (9600 baud).
// Open it as a separate sketch (its own folder).
const int pins[]   = {2, 4, 7, 8, 12, 13, A0, A1, A2};
const char* names[] = {"Base left", "Base right", "Shoulder forward", "Shoulder back",
                       "Elbow down", "Elbow up", "Gripper open", "Gripper close", "Auto task"};
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 9; i++) pinMode(pins[i], INPUT_PULLUP);
  Serial.println("Button test started");
}
void loop() {
  for (int i = 0; i < 9; i++)
    if (digitalRead(pins[i]) == LOW) { Serial.print(names[i]); Serial.println(" pressed"); delay(200); }
}
