

const int BUTTON_A = 25;
const int BUTTON_B = 26;
const int BUTTON_C = 27;
const int BUTTON_STOP = 33;

const unsigned long DEBOUNCE_MS = 200;

int lastStateA = HIGH;
int lastStateB = HIGH;
int lastStateC = HIGH;
int lastStateStop = HIGH;

unsigned long lastPressA = 0;
unsigned long lastPressB = 0;
unsigned long lastPressC = 0;
unsigned long lastPressStop = 0;

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_STOP, INPUT_PULLUP);

  delay(500);
  Serial.println("ESP32_DRONES_READY");
}

void loop() {

  int stateA = digitalRead(BUTTON_A);
  int stateB = digitalRead(BUTTON_B);
  int stateC = digitalRead(BUTTON_C);
  int stateStop = digitalRead(BUTTON_STOP);

  // A
  if (lastStateA == HIGH && stateA == LOW &&
      millis() - lastPressA > DEBOUNCE_MS) {

    Serial.println("A");
    lastPressA = millis();
  }

  // B
  if (lastStateB == HIGH && stateB == LOW &&
      millis() - lastPressB > DEBOUNCE_MS) {

    Serial.println("B");
    lastPressB = millis();
  }

  // C
  if (lastStateC == HIGH && stateC == LOW &&
      millis() - lastPressC > DEBOUNCE_MS) {

    Serial.println("C");
    lastPressC = millis();
  }

  // STOP
  if (lastStateStop == HIGH && stateStop == LOW &&
      millis() - lastPressStop > DEBOUNCE_MS) {

    Serial.println("STOP");
    lastPressStop = millis();
  }

  lastStateA = stateA;
  lastStateB = stateB;
  lastStateC = stateC;
  lastStateStop = stateStop;

  delay(10);
}
