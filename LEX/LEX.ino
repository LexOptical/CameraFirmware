#include <limits.h>
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

elapsedMillis timeSinceInteraction; // How long it has been since something important happened
elapsedMillis timeSinceExposure; // How long it has been we calculated exposure

int16_t ax, ay, az;
int16_t gx, gy, gz;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void setup()
{
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

    loadSettings();
}

void loadSettings()
{
    sensitivityIndex = EEPROM.read(0) > SENSITIVITIES_COUNT-1 ? 0 : EEPROM.read(0);
    apertureIndex = EEPROM.read(1) > APERTURES_COUNT-1 ? 0 : EEPROM.read(1);
    int start = 2;
    int offset = 0;
    EEPROM.get(start + (offset * sizeof(int)), gx_offset);
    offset++;
    EEPROM.get(start + (offset * sizeof(int)), gy_offset);
    offset++;
    EEPROM.get(start + (offset * sizeof(int)), gz_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
}

void writeSettings()
{
    EEPROM.update(0, sensitivityIndex);
    EEPROM.update(1, apertureIndex);
    int start = 2;
    int offset = 0;
    EEPROM.put(start + (offset * sizeof(int)), gx_offset);
    offset++;
    EEPROM.put(start + (offset * sizeof(int)), gy_offset);
    offset++;
    EEPROM.put(start + (offset * sizeof(int)), gz_offset);
}

void readButtons()
{
    if (digitalRead(buttonShutter) == LOW) {
        shootFrame(exposureDurationSeconds);
        timeSinceInteraction = 0;
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
        delay(95);
        timeSinceInteraction = 0; // Keep awake if user is interacting
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
    case DEBUG:
     switch (button) {
        case button_br:
            calibration();
            break;
        case button_tr:
            antishake = !antishake;
            break;
        }
        break;
    }
}

void loop()
{
    if (awake) {
        if (!ael && timeSinceExposure > EXPOSURE_CALC_INTERVAL) {
            exposureDurationSeconds = calculateExposure();
            updateDisplay(exposureDurationSeconds);
            timeSinceExposure = 0;
        }
        readButtons();
        if (timeSinceInteraction > SLEEP_TIMEOUT) {
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
        int movementThreshold = 15; // TODO Calculate this from the focal length of the lens
        while (true) {
            if (digitalRead(buttonShutter) != LOW) {
                deprimeShutters();
                return; // The user gave up before we could shoot the frame
            }
            accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            if (abs(gx) > movementThreshold || abs(gy) > movementThreshold || abs(gz) > movementThreshold) {
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
    unsigned int microseconds = exposureTimeSeconds * 1000000;
    unsigned long milliseconds = exposureTimeSeconds * 1000;
    Serial.print("ms");
    Serial.println(milliseconds);

    digitalWrite(frontCurtainPin, LOW); // Drop front curtain

    if (milliseconds >= UINT_MAX / 1000) {
        delay(milliseconds);
    } else {
        delayMicroseconds(microseconds);
    }

    digitalWrite(rearCurtainPin, HIGH); // Drop rear curtain
    delay(10); // activation time time for coil
    digitalWrite(rearCurtainPin, LOW); // Stop the power draw (should do inside delay for long shots)
}

void onSleep()
{
    deprimeShutters();
    display.clearDisplay();   
    display.display();
    writeSettings();
    awake = false;
}

void onWake()
{
    awake = true;
    timeSinceInteraction = 0;
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
    uint16_t lux = getLux();
    float currentISO = FILM_SENSITIVITY_TABLE[sensitivityIndex];
    float currentAperture = APERTURE_TABLE[apertureIndex];
    return pow(currentAperture, 2) * 64 / (lux * currentISO); //T = exposure time, in seconds
}

void debugDisplay(){
    display.setCursor(0,0);
    //display.print(lux, 1);
    //display.println("Lx");
    display.setTextWrap(true);
    display.println(gx);
    display.println(gy);
    display.println(gz);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if (abs(gx) > 150 || abs(gy) > 150 || abs(gz) > 150) {
            display.print("Moving");
    } else {
            display.print("Static");
    }
    display.print(" antishake:");
    display.print(antishake);

    display.display();
    display.clearDisplay();
}

void updateDisplay(float T)
{
    if (mode == DEBUG) {
        return debugDisplay();
    }

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



int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=80;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=100;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;

void meansensors() {
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

    while (i<(buffersize+101)){
        // read raw accel/gyro measurements from device
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
        buff_ax=buff_ax+ax;
        buff_ay=buff_ay+ay;
        buff_az=buff_az+az;
        buff_gx=buff_gx+gx;
        buff_gy=buff_gy+gy;
        buff_gz=buff_gz+gz;
        }
        if (i==(buffersize+100)){
        mean_ax=buff_ax/buffersize;
        mean_ay=buff_ay/buffersize;
        mean_az=buff_az/buffersize;
        mean_gx=buff_gx/buffersize;
        mean_gy=buff_gy/buffersize;
        mean_gz=buff_gz/buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}

void calibration() {

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("CALIBRATING, WAIT");
    display.display();
    display.clearDisplay();

    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;

    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;

    while (true){
        int ready=0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);

        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();

        /*
        if (abs(mean_ax)<=acel_deadzone) ready++;
        else ax_offset=ax_offset-mean_ax/acel_deadzone;

        if (abs(mean_ay)<=acel_deadzone) ready++;
        else ay_offset=ay_offset-mean_ay/acel_deadzone;

        if (abs(16384-mean_az)<=acel_deadzone) ready++;
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
        */
        
        if (abs(mean_gx)<=giro_deadzone) ready++;
        else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

        if (abs(mean_gy)<=giro_deadzone) ready++;
        else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

        if (abs(mean_gz)<=giro_deadzone) ready++;
        else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);


        display.setCursor(0, 0);
        display.println("CALIBRATING, WAIT");
        display.print(ready);
        display.println(" ready");
       /* 
        display.print(ax_offset);
        display.print(",");
        display.print(ay_offset);
        display.print(",");
        display.println(az_offset);
*/
        display.print(gx_offset);
        display.print(",");
        display.print(gy_offset);
        display.print(",");
        display.println(gz_offset);

        
        display.display();
        display.clearDisplay();

        if (ready==3) break;


    }
    writeSettings();
}
