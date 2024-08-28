#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define SENSOR_PIN  A0  // Analog pin connected to LM35 sensor's output
#define RELAY_PIN   7   // Digital pin connected to the relay module

const float TEMP_THRESHOLD_UPPER = 50;  // Upper threshold of temperature, change to your desired value


OneWire oneWire(SENSOR_PIN);         // Setup a oneWire instance
DallasTemperature sensors(&oneWire); // Pass oneWire to DallasTemperature library


void setup() {
  Serial.begin(9600); // Initialize serial communication
  sensors.begin();    // Initialize the sensor
  pinMode(RELAY_PIN, OUTPUT); // Initialize digital pin as an output
  lcd.init();
  lcd.backlight();
}

void loop() {
  int value1 = analogRead(SENSOR_PIN);
  double tempC = value1 * (5.0 / 1023.0) * 100;
  double tempF = tempC * 9 / 5 + 32;

  lcd.setCursor(0, 0);
  lcd.print("..Temperature..");
  lcd.setCursor(0, 1);
  lcd.print("C:");
  lcd.print(tempC);
  lcd.setCursor(8, 1);
  lcd.print(" F:");
  lcd.print(tempF);

  Serial.print("Temperature C: ");
  Serial.print(tempC);
  Serial.print("'C");
  Serial.print("\t");
  Serial.print("Temperature F: ");
  Serial.print(tempF);
  Serial.println("'F");


  if (tempC < TEMP_THRESHOLD_UPPER) {
    Serial.println("The heating element is turned off");
    digitalWrite(RELAY_PIN, HIGH ); // Turn off the relay
  } else  {
    Serial.println("The heating element is turned on");
    digitalWrite(RELAY_PIN, LOW); // Turn on the relay
  }

  delay(500);
}