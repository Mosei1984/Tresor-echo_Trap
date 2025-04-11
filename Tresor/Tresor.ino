#include <Keypad.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <NewPing.h>
#include <EEPROM.h>

// ==== Pinbelegung ====
#define TRIG_PIN 6
#define ECHO_PIN 4
#define SERVO_PIN 5
#define ULTRA_DIST_CM 30
#define MAX_ATTEMPTS 3
#define TIMEOUT_SECONDS 30

// ==== Objekte ====
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo;
#define ARDUINO_AVR_UNO
SoftwareSerial softSerial(3,2);
#define FPSerial softSerial
DFRobotDFPlayerMini player;
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

// ==== Keypad-Setup ====
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {10, 9, 8, 7};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ==== PINs und Status ====
String correctPIN = "";
const String masterPIN = "D0000D";
String inputPIN = "";
int attempts = 0;
unsigned long entryStartTime = 0;
bool active = false;
bool unlocked = false;
bool timerExpired = false;

// ==== Alarm-Zustand ====
unsigned long alarmStartTime = 0;
unsigned long lastAlarmPlay = 0;
bool alarmActive = false;
const unsigned long alarmRepeatDelay = 17000;
const unsigned long alarmTotalTime = 30000;

// ==== Setup ====
void setup() {
  lcd.begin(16,2);
  lcd.backlight();
  FPSerial.begin(9600);
  Serial.begin(115200);
  

  if (!player.begin(softSerial, true, true)) {
    lcd.setCursor(0,0); lcd.print("DFPlayer Fehler");
    while (true) delay(0);
  }

  // PIN aus EEPROM laden
  char pinBuffer[16];
  EEPROM.get(0, pinBuffer);
  correctPIN = String(pinBuffer);
  if (correctPIN.length() < 4) correctPIN = "10310#"; // Fallback

  lcd.setCursor(0,0); lcd.print("System bereit");
  lcd.setCursor(0,1); lcd.print("Warte auf Besuch");
}

// ==== Loop ====
void loop() {
  float distance = sonar.ping_cm();

  if (alarmActive) {
    unsigned long currentTime = millis();

    if (currentTime - lastAlarmPlay >= alarmRepeatDelay) {
      player.play(1);
      lastAlarmPlay = currentTime;
    }

    lcd.setCursor(0,1);
    lcd.print("Retry in: ");
    lcd.print((alarmTotalTime - (currentTime - alarmStartTime)) / 1000);
    lcd.print("s ");

    if (currentTime - alarmStartTime >= alarmTotalTime) {
      alarmActive = false;
      activateSystem();
    }

    return;
  }

  if (!active && distance > 0 && distance < ULTRA_DIST_CM) {
    activateSystem();
  }

  if (active) {
    handleKeypad();
    checkTimeout();

    if (timerExpired && distance > 0 && distance < ULTRA_DIST_CM) {
      triggerAlarm(true); return;
    }

    if (timerExpired && (distance == 0 || distance > ULTRA_DIST_CM)) {
      resetSystem(); return;
    }
  }
}

// ==== System aktivieren ====
void activateSystem() {
  active = true;
  timerExpired = false;
  inputPIN = "";
  attempts = 0;
  entryStartTime = millis();

  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Willkommen!");
  lcd.setCursor(0,1); lcd.print("PIN eingeben");
  player.play(2);
}

// ==== System zurücksetzen ====
void resetSystem() {
  active = false;
  timerExpired = false;
  inputPIN = "";
  attempts = 0;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Warte auf Besuch");
}

// ==== Tasteneingabe ====
void handleKeypad() {
  char key = keypad.getKey();
  if (key) {
    if (key == '*') {
      verifyPIN();
    } else if (key == 'A') {
      inputPIN = "";
      lcd.setCursor(0,1); lcd.print("PIN geloescht   ");
    } else {
      inputPIN += key;
      lcd.setCursor(0,1);
      lcd.print("PIN: " + inputPIN + "     ");
    }
  }
}

// ==== PIN prüfen ====
void verifyPIN() {
  if (inputPIN == correctPIN) {
    unlockSafe();
  } else if (inputPIN == masterPIN) {
    adminAccess();
  } else {
    attempts++;
    inputPIN = "";
    lcd.setCursor(0,1); lcd.print("Falscher PIN   ");
    player.play(3);
    delay(1000);

    if (attempts >= MAX_ATTEMPTS) {
      triggerAlarm(true);
    }
  }
}
// ==== Servo langsam bewegen ====
void moveServoSmooth(int startAngle, int endAngle, int stepDelay = 15) {
  servo.attach(SERVO_PIN);
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
  servo.detach();
}

// ==== Tresor öffnen ====
void unlockSafe() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("PIN korrekt");
  lcd.setCursor(0,1); lcd.print("Tresor offen");
  player.play(5);
  moveServoSmooth(0, 180, 50);
  unlocked = true;
  delay(5000);
  moveServoSmooth(180, 0, 50);
  resetSystem();
}

// ==== Admin-Modus & PIN ändern ====
void adminAccess() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Admin-Zugang");
  lcd.setCursor(0,1); lcd.print("PIN aendern...");
  player.play(4);
  delay(2000);

  String newPIN = "";
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Neue PIN (* enden)");

  while (true) {
    char key = keypad.getKey();
    if (key) {
      if (key == '*') break;
      if (key == 'A') {
        newPIN = "";
        lcd.setCursor(0,1); lcd.print("Geloescht       ");
      } else {
        newPIN += key;
        lcd.setCursor(0,1); lcd.print("Neu: " + newPIN + "    ");
      }
    }
  }

  if (newPIN.length() >= 4) {
    correctPIN = newPIN;
    char storeBuffer[16];
    newPIN.toCharArray(storeBuffer, 16);
    EEPROM.put(0, storeBuffer);
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("PIN gespeichert");
  } else {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("PIN zu kurz!");
  }

  delay(2000);
  resetSystem();
}

// ==== Timeout prüfen ====
void checkTimeout() {
  if ((millis() - entryStartTime) > TIMEOUT_SECONDS * 1000) {
    timerExpired = true;
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Zeit abgelaufen");
  }
}

// ==== Alarm auslösen ====
void triggerAlarm(bool playSound) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("ALARM!");

  if (playSound) {
    player.play(1);
    alarmStartTime = millis();
    lastAlarmPlay = millis();
    alarmActive = true;
  }
}

