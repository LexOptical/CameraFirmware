#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BH1750.h>
#include <EEPROM.h>
#include <I2Cdev.h>
#include <LowPower.h>
#include <MPU6050.h>
#include <SPI.h> // Including all of the libraries needed for TSL2561, OLED, and EEPROM
#include <Wire.h>
#include <elapsedMillis.h>

#define OLED_RESET 4 // OLED reset pin set to GPIO4

Adafruit_SSD1306 display(OLED_RESET);
MPU6050 accelgyro;
BH1750 lightMeter;

enum MENU_MODE {
    APERTURE = 0,
    MANUAL = 1,
    ISO_CHANGE = 2,
    DEBUG = 3
};
enum TIME_DISPLAY_MODE {
    MINUTES = 0,
    FACTIONAL = 1,
    SECONDS = 2,
    OVERLOAD = 3
};

const float APERTURE_TABLE[] = { 1, 1.4, 2, 2.8, 4, 4.5, 5.6, 8, 11, 16, 22, 32, 45, 64, 90 }; // Array of aperture values
const int APERTURES_COUNT = sizeof APERTURE_TABLE / sizeof APERTURE_TABLE[0];
const float FILM_SENSITIVITY_TABLE[] = { 6, 12, 25, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200 }; // Array of sensitivity values
const int SENSITIVITIES_COUNT = sizeof FILM_SENSITIVITY_TABLE / sizeof FILM_SENSITIVITY_TABLE[0];
const int SLEEP_TIMEOUT = 30000;
const int EXPOSURE_CALC_INTERVAL = 100;


// Electromechanical
const int rearCurtainPin = 7; // Shutter which is passively held. Rear curtain. TOP circuit on shutter board
const int frontCurtainPin = 8; // Shutter which must be actively held. Front curtain. BOTTOM circuit on shutter board
const int motorDrivePin = 9; // Raise this high to drive the motor
const int shutterPositionSense = 10; // Determines where the shutter priming arm is

// Buttons
const int button_tl = 5; // Top Left
const int button_tr = 3; // Top Right
const int button_bl = 6; // Bottom Left
const int button_br = 4; // Bottom Right
const int buttonShutter = 2;
const int ledPin = 13; // Shutter button LED

// States
boolean awake = true; // Are we doing anything important right now
boolean antishake = false;
boolean ael = false;
byte mode = APERTURE;
int sensitivityIndex = 6; // Remember the last sensitivity value saved to EEPROM (addr = 0)
int apertureIndex = 7; // Remember the last aperture value saved to EEPROM (addr = 1)
uint16_t lux = 0;
float exposureDurationSeconds;

elapsedMillis timeElapsed; // How long it has been since something important happened
elapsedMillis timeSinceExposure; // How long it has been we calculated exposure

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
    loadSettings();

    pinMode(ledPin, OUTPUT);

    pinMode(rearCurtainPin, OUTPUT);
    pinMode(motorDrivePin, OUTPUT);
    pinMode(frontCurtainPin, OUTPUT);

    pinMode(shutterPositionSense, INPUT_PULLUP);

    pinMode(buttonShutter, INPUT_PULLUP);

    pinMode(button_tl, INPUT_PULLUP);
    pinMode(button_tr, INPUT_PULLUP);
    pinMode(button_bl, INPUT_PULLUP);
    pinMode(button_br, INPUT_PULLUP);

    // No current flow
    digitalWrite(frontCurtainPin, LOW);
    digitalWrite(rearCurtainPin, LOW);

    // Make sure we aren't running the motor at power up
    digitalWrite(motorDrivePin, LOW);

    Serial.begin(9600);
    lightMeter.begin();
    accelgyro.initialize();

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Initialize with the I2C addr 0x3C (for the 128x32 OLED)
    display.clearDisplay();
    display.display();
}

void loadSettings()
{
    sensitivityIndex = EEPROM.read(0) > SENSITIVITIES_COUNT-1 ? 0 : EEPROM.read(0);
    apertureIndex = EEPROM.read(1) > APERTURES_COUNT-1 ? 0 : EEPROM.read(1);
}

void writeSettings()
{
    EEPROM.update(0, sensitivityIndex);
    EEPROM.update(1, apertureIndex);
}

void readButtons()
{
    if (digitalRead(buttonShutter) == LOW) {
        shootFrame(exposureDurationSeconds);
        timeElapsed = 0;
    }

    int buttonPushed = 0;
    if (digitalRead(button_tr) == LOW) {
        buttonPushed = button_tr;
    }
    if (digitalRead(button_br) == LOW) {
        buttonPushed = button_br;
    }
    if (digitalRead(button_tl) == LOW) {
        buttonPushed = button_tl;
    }
    if (digitalRead(button_bl) == LOW) {
        buttonPushed = button_bl;
    }

    if (buttonPushed != 0) {
        onButtonPushed(buttonPushed);
        writeSettings();
        delay(95);
        timeElapsed = 0; // Keep awake if user is interacting
    }
}

void onButtonPushed(int button)
{
    if (button == button_tl) {
        mode = (mode + 1) % 4;
        return;
    }
    if (button == button_bl) {
        ael = !ael;
        return;
    }
    switch (mode) {
    case APERTURE:
        switch (button) {
        case button_br:
            if (apertureIndex > 0) {
                apertureIndex--;
            }
            break;
        case button_tr:
            if (apertureIndex < APERTURES_COUNT-1) {
                apertureIndex++;
            }
            break;
        }
        break;
    case ISO_CHANGE:
        switch (button) {
        case button_br:
            if (sensitivityIndex > 0) {
                sensitivityIndex--;
            }
            break;
        case button_tr:
            if (sensitivityIndex < SENSITIVITIES_COUNT-1) {
                sensitivityIndex++;
            }
            break;
        }
        break;
    }
}

void loop()
{
    if (awake) {
        if (timeSinceExposure > EXPOSURE_CALC_INTERVAL) {
            exposureDurationSeconds = calculateExposure();
            updateDisplay(exposureDurationSeconds);
            timeSinceExposure = 0;
        }
        readButtons();
        if (timeElapsed > SLEEP_TIMEOUT) {
            onSleep();
        }
    } else if (digitalRead(buttonShutter) == LOW) {
        onWake();
        delay(500);
    } else {
        LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
    }
}

void shootFrame(float exposureTimeSeconds)
{
    primeShutters();
    if (antishake == true) {
        while (true) {
            if (digitalRead(buttonShutter) != LOW) {
                deprimeShutters();
                return; // The user gave up before we could shoot the frame
            }
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            if (abs(gx) > 10 || abs(gy) > 10 || abs(gx) > 10) {
                continue;
            } else {
                break;
            }
        }
    }
    fireShutters(exposureTimeSeconds);
}

void runMotor()
{
    digitalWrite(motorDrivePin, HIGH); // run the motor
    while (digitalRead(shutterPositionSense) == LOW) {
        // Change this to interupts
    }
    delay(2); // the primer arm depart (debounce)
    while (digitalRead(shutterPositionSense) == HIGH) {
        // Change this to interupts
    }
    digitalWrite(motorDrivePin, LOW); // stop the motor
    delay(100); // let the motor stop

    // Now that the motor has no load we can run it slowly
    // to return the primer to the correct position.
    analogWrite(motorDrivePin, 75);
    while (digitalRead(shutterPositionSense) == HIGH) {
    }
    digitalWrite(motorDrivePin, LOW);

    delay(40); // safety
}

void holdShutters()
{
    digitalWrite(frontCurtainPin, HIGH); // hold the front shutter
    // should pwm the above?
    digitalWrite(rearCurtainPin, LOW); // ensure hold the rear shutter
}

/**
   Bring the shutters into the ready to fire state
*/
void primeShutters()
{
    holdShutters();
    runMotor();
}

/**
   Bring the shutters into the state where they don't have to be actively held open.
*/
void deprimeShutters()
{
    digitalWrite(rearCurtainPin, HIGH); // Drop rear curtain
    delay(10); // activation time time for coil
    digitalWrite(rearCurtainPin, LOW); // Set rear curtain drop off again
    delay(10); // safety
    digitalWrite(frontCurtainPin, LOW); // Drop front curtain
}

/**
   Take a photo!
*/
void fireShutters(float exposureTimeSeconds)
{
    int microseconds = exposureTimeSeconds * 1000000;
    int milliseconds = exposureTimeSeconds * 1000;
    Serial.print("ms");
    Serial.println(milliseconds);

    digitalWrite(frontCurtainPin, LOW); // Drop front curtain

    // if (microseconds > 16000) {
    delay(milliseconds);
    //} else {
    //   delayMicroseconds(microseconds);
    // }

    digitalWrite(rearCurtainPin, HIGH); // Drop rear curtain
    delay(10); // activation time time for coil
    digitalWrite(rearCurtainPin, LOW); // Stop the power draw (should do inside delay for long shots)
}

void onSleep()
{
    deprimeShutters();
    display.clearDisplay();
    display.display();
    awake = false;
}

void onWake()
{
    awake = true;
    timeElapsed = 0;
}

uint16_t getLux()
{
    lux = lightMeter.readLightLevel();
    return lux;
}

/**
   Exposure time in seconds
*/
float calculateExposure()
{
    if (ael) {
        return;
    }
    uint16_t lux = getLux();
    float currentISO = FILM_SENSITIVITY_TABLE[sensitivityIndex];
    float currentAperture = APERTURE_TABLE[apertureIndex];
    return pow(currentAperture, 2) * 64 / (lux * currentISO); //T = exposure time, in seconds
}

void updateDisplay(float T)
{
    int timeDisplayMode; // State of shutter speed value display (fractional, seconds, minutes)
    int tFractionalDivisor; // Fractional time e.g. 1/1000, where tFractionalDivisor = 1000
    float Ev; // Calculated Exposure Value
    float Tmin; // Time in minutes

    if (T >= 60) {
        timeDisplayMode = TIME_DISPLAY_MODE::MINUTES;
        Tmin = T / 60;
    } else if (T < 0.75) {
        timeDisplayMode = TIME_DISPLAY_MODE::FACTIONAL;
        if (T < 0.000125) {
            timeDisplayMode = 3;
        } else if (T <= 0.000188) {
            tFractionalDivisor = 8000;
        } else if (T <= 0.000375) {
            tFractionalDivisor = 4000;
        } else if (T <= 0.00075) {
            tFractionalDivisor = 2000;
        } else if (T <= 0.0015) {
            tFractionalDivisor = 1000;
        } else if (T <= 0.003) {
            tFractionalDivisor = 500;
        } else if (T <= 0.006) {
            tFractionalDivisor = 250;
        } else if (T <= 0.012333) {
            tFractionalDivisor = 125;
        } else if (T <= 0.025) {
            tFractionalDivisor = 60;
        } else if (T <= 0.05) {
            tFractionalDivisor = 30;
        } else if (T <= 0.095833) {
            tFractionalDivisor = 15;
        } else if (T <= 0.1875) {
            tFractionalDivisor = 8;
        } else if (T <= 0.375) {
            tFractionalDivisor = 4;
        } else if (T <= 0.75) {
            tFractionalDivisor = 2;
        }
    } else if ((T >= 0.75) && (T < 60)) {
        timeDisplayMode = TIME_DISPLAY_MODE::SECONDS;
    }
    if (lux == 0) { // This happens if the sensor is overloaded or senses no light.
        timeDisplayMode = TIME_DISPLAY_MODE::OVERLOAD;
    }
    display.setTextWrap(false);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    if (mode == APERTURE) {
        display.print("F/");
    } else {
        display.print("f/");
    }
    display.println(APERTURE_TABLE[apertureIndex], 1);

    if (ael) {
        display.print("*");
    }

    switch (timeDisplayMode) {
    case TIME_DISPLAY_MODE::MINUTES:
        display.print(Tmin, 1);
        display.println("m");
        break;
    case TIME_DISPLAY_MODE::FACTIONAL:
        display.print("1/");
        display.println(tFractionalDivisor);
        break;
    case TIME_DISPLAY_MODE::SECONDS:
        display.print(T, 1);
        display.println("s");
        break;
    case TIME_DISPLAY_MODE::OVERLOAD:
        display.println("RANGE!");
        break;
    }

    display.println(T, 3);

    display.drawLine(73, 0, 73, 32, WHITE);

    display.setTextSize(1);
    display.setCursor(76, 0);
    if (mode == ISO_CHANGE) {
        display.print("ISO");
    } else {
        display.print("iso");
    }
    display.println(FILM_SENSITIVITY_TABLE[sensitivityIndex], 0);
    display.setCursor(76, 11);
    display.print("EV=");
    Ev = log(pow(APERTURE_TABLE[apertureIndex], 2)) / log(2) + log(1 / T) / log(2);
    display.println(floor(Ev + 0.5), 1);
    display.setCursor(76, 22);
    display.print(lux, 1);
    display.println("Lx");

    display.display();
    display.clearDisplay();
}
