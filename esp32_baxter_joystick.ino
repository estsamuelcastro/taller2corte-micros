/*
  ESP32 - BAXTER + JOYSTICK

  VCC -> 3.3V
  GND -> GND
  VRx -> GPIO 34
  VRy -> GPIO 35
  SW  -> GPIO 27
*/

const int JOY_X = 34;
const int JOY_Y = 35;
const int JOY_SW = 27;

int centerX = 2048;
int centerY = 2048;

const float DEADZONE = 0.12;
const unsigned long SEND_MS = 30;

unsigned long lastSend = 0;

float normalizeAxis(
  int value,
  int center
)
{
  float x;

  if (value >= center)
  {
    x = (float)(value - center)
        / (4095.0 - center);
  }
  else
  {
    x = (float)(value - center)
        / center;
  }

  if (fabs(x) < DEADZONE)
  {
    x = 0.0;
  }
  else
  {
    if (x > 0)
    {
      x = (x - DEADZONE)
          / (1.0 - DEADZONE);
    }
    else
    {
      x = (x + DEADZONE)
          / (1.0 - DEADZONE);
    }
  }

  return constrain(x, -1.0, 1.0);
}

void calibrateJoystick()
{
  long sumX = 0;
  long sumY = 0;

  const int samples = 250;

  for (int i = 0; i < samples; i++)
  {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(4);
  }

  centerX = sumX / samples;
  centerY = sumY / samples;

  Serial.println("CALIBRACION_OK");
  Serial.print("CENTER_X=");
  Serial.println(centerX);
  Serial.print("CENTER_Y=");
  Serial.println(centerY);
}

void setup()
{
  Serial.begin(115200);

  pinMode(
    JOY_SW,
    INPUT_PULLUP
  );

  delay(1000);

  Serial.println(
    "BAXTER_JOYSTICK_START"
  );

  Serial.println(
    "NO MOVER JOYSTICK..."
  );

  calibrateJoystick();

  Serial.println("READY");
}

void loop()
{
  if (millis() - lastSend >= SEND_MS)
  {
    lastSend = millis();

    int rawX =
      analogRead(JOY_X);

    int rawY =
      analogRead(JOY_Y);

    int sw =
      digitalRead(JOY_SW);

    float x =
      normalizeAxis(
        rawX,
        centerX
      );

    float z =
      normalizeAxis(
        rawY,
        centerY
      );

    // Arriba = Z positiva
    z = -z;

    Serial.print("JOY,");
    Serial.print(x, 3);
    Serial.print(",");
    Serial.print(z, 3);
    Serial.print(",");
    Serial.println(
      sw == LOW ? 1 : 0
    );
  }
}
