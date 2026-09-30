/*
  ACTIVIDAD C - ESP32 + JOYSTICK PARA MOVILIDAD DEL ATLAS

  JOYSTICK:
    VCC -> 3.3V
    GND -> GND
    VRx -> GPIO 34
    VRy -> GPIO 35
    SW  -> GPIO 27

  VRy controla:
      adelante / atrás

  VRx controla:
      giro izquierda / derecha

  SW controla:
      STOP / reanudar
*/

const int JOY_X = 34;
const int JOY_Y = 35;
const int JOY_SW = 27;

int centerX = 2048;
int centerY = 2048;

const float DEADZONE = 0.12;

const unsigned long SEND_INTERVAL = 30;

unsigned long lastSend = 0;


// ============================================================
// NORMALIZACIÓN
// ============================================================

float normalizeAxis(
  int rawValue,
  int center
)
{
  float value;

  if (rawValue >= center)
  {
    value =
      (float)(rawValue - center)
      /
      (4095.0 - center);
  }
  else
  {
    value =
      (float)(rawValue - center)
      /
      center;
  }

  if (fabs(value) < DEADZONE)
  {
    value = 0.0;
  }
  else
  {
    if (value > 0.0)
    {
      value =
        (value - DEADZONE)
        /
        (1.0 - DEADZONE);
    }
    else
    {
      value =
        (value + DEADZONE)
        /
        (1.0 - DEADZONE);
    }
  }

  return constrain(
    value,
    -1.0,
    1.0
  );
}


// ============================================================
// CALIBRAR
// ============================================================

void calibrateJoystick()
{
  long totalX = 0;
  long totalY = 0;

  const int samples = 250;

  for (int i = 0; i < samples; i++)
  {
    totalX += analogRead(JOY_X);
    totalY += analogRead(JOY_Y);

    delay(4);
  }

  centerX =
    totalX / samples;

  centerY =
    totalY / samples;

  Serial.println(
    "CALIBRACION_OK"
  );

  Serial.print(
    "CENTER_X="
  );

  Serial.println(
    centerX
  );

  Serial.print(
    "CENTER_Y="
  );

  Serial.println(
    centerY
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );

  pinMode(
    JOY_SW,
    INPUT_PULLUP
  );

  delay(1000);

  Serial.println(
    "ATLAS_CONTROLLER_START"
  );

  Serial.println(
    "NO MOVER JOYSTICK..."
  );

  calibrateJoystick();

  Serial.println(
    "READY"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  if (
    millis() - lastSend
    >= SEND_INTERVAL
  )
  {
    lastSend = millis();

    int rawX =
      analogRead(
        JOY_X
      );

    int rawY =
      analogRead(
        JOY_Y
      );

    int sw =
      digitalRead(
        JOY_SW
      );

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

    // Joystick arriba = avanzar
    z = -z;

    Serial.print(
      "JOY,"
    );

    Serial.print(
      x,
      3
    );

    Serial.print(
      ","
    );

    Serial.print(
      z,
      3
    );

    Serial.print(
      ","
    );

    Serial.println(
      sw == LOW ? 1 : 0
    );
  }
}
