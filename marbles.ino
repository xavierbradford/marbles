#include <ESP32Encoder.h>
#include <Wire.h>
#include <Servo.h>

#include <numbers>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);

ESP32Encoder encoder;
// 11 PPR * 4 (quadrature) * 46.8 (gear ratio for 12V 130RPM JGA25-371) = 2059.2
const float NOMINAL_T = 1975.0;
const float ALPHA_PHASE = 1.0; // phase tuning gain
const float BETA_PERIOD = 1.0; // period tuning gain
float period = 1975.0;         // we will update this using alpha-beta filter
float phase = 0.0;
// position in rotations, phase-adjusted so .0 is at the peak of any marbles passing by.
float getScrewPosition()
{
    return encoder.getCount() / period + phase;
}
bool calibrated = false;

Servo myservo; // create Servo object to control a servo

void setup(void)
{
    Serial.begin(115200);

    delay(1000);
    Serial.println("looking 4 sensor!!!");

    Wire.begin(42, 41);

    if (tcs.begin())
    {
        Serial.println("Found sensor");
    }
    else
    {
        Serial.println("No TCS34725 found ... check your connections");
        while (1)
            ;
    }

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(10, 11);
    encoder.clearCount();

    myservo.attach(9); // attaches the servo on pin 9 to the Servo object
}

void updateEstimator(int lux)
{
    static bool rotationMayHaveMarble = true;
    static int previousRotation = 0;
    static float lastPos = 0.0;
    static float sinCorr = 0.0;
    static float cosCorr = 0.0;
    float screwPos = getScrewPosition();
    float deltaPos = screwPos - lastPos;

    // check for a rotation being finished
    if (screwPos - previousRotation > 1)
    {
        previousRotation = (int)floor(screwPos);
        // we freaking did another rotation
        // AI code
        if (rotationMayHaveMarble)
        {
            float phaseErrorThreads = std::atan2(-sinCorr, std::max(cosCorr, 1e-3f)) / (2.0 * std::numbers::pi);

            float oldPeriod = period;
            period = std::clamp(oldPeriod - BETA_PERIOD * phaseErrorThreads, NOMINAL_T * 0.5f, NOMINAL_T * 2.0f);

            float leverArmCompensation = encoder.getCount() * ((1.0f / oldPeriod) - (1.0f / period));
            phase += leverArmCompensation + (ALPHA_PHASE * phaseErrorThreads);

            if (abs(BETA_PERIOD * phaseErrorThreads) < 0.01 && abs(ALPHA_PHASE * phaseErrorThreads) < 0.01)
            {
                calibrated = true;
            }
        }

        rotationMayHaveMarble = true;
        sinCorr = 0.0;
        cosCorr = 0.0;
    }

    if (deltaPos <= 0.0)
    {
        return; // we don't want to integrate with respect to this. also don't update lastPos.
    }

    // the lux doesn't go below 30 if there is a marble.
    if (lux < 30)
    {
        rotationMayHaveMarble = false;
    }

    // // clip it if it's peaking
    // if (lux > 95) {
    //   lux = 95;
    // }

    // update the integration
    float centeredLux = lux - 250.0;
    sinCorr += centeredLux * 100.0 * sin(2.0 * std::numbers::pi * getScrewPosition()) * deltaPos;
    cosCorr += centeredLux * 100.0 * cos(2.0 * std::numbers::pi * getScrewPosition()) * deltaPos;

    // print everything
    Serial.print(lux);
    Serial.print(" ");
    Serial.print(calibrated * 100.0);
    Serial.print(" ");
    Serial.println(250.0 + 100.0 * cos(2.0 * std::numbers::pi * getScrewPosition()));

    lastPos = screwPos;
}

void loop(void)
{
    // get lux from colour sensor
    uint16_t r, g, b, c, marbleness;
    tcs.getRawData(&r, &g, &b, &c);
    marbleness = r - g / 4 - b / 4;

    updateEstimator(marbleness);
}
