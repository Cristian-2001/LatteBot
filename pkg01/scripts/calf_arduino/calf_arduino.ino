#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

const int buttonPin = 3;
const int ledPin = 13;

int dati[5];

size_t DATI = 5;

int weight = 10;
int buttonState = 0;
unsigned long timestamp;

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  timestamp = millis();
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Welcome");

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 17);
  display.println("Setup");
  display.println("in corso");
  display.println("----------");

  display.display();

  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (millis() - timestamp > 500) {
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH && weight != 2) {
      weight = 2;
      digitalWrite(ledPin, LOW);
    }
    else if (buttonState == HIGH) {
      weight = 10;
      digitalWrite(ledPin, HIGH);
    }


    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Kg rimasti");

    display.setTextSize(2);
    display.setTextColor(WHITE);

    // Converti il peso in stringa per calcolare la lunghezza
    String weightText = String(weight) + " kg";
    int16_t x1, y1;
    uint16_t textWidth, textHeight;

    // Calcola le dimensioni del testo
    display.getTextBounds(weightText, 0, 0, &x1, &y1, &textWidth, &textHeight);

    // Centra orizzontalmente (assumendo display 128 pixel di larghezza)
    int cursorX = (128 - textWidth) / 2;

    // Posiziona sulla terza riga (circa y=32 per un display standard)
    int cursorY = 32;

    display.setCursor(cursorX, cursorY);
    display.print(weightText);
    display.display();

    // pacchetto dati
    // FF calf_num weight FE
    Serial.write(0xFF);
    Serial.write(char(0));
    Serial.write(char(weight));

    Serial.write(0xFE);

    timestamp = millis();
  }
}
