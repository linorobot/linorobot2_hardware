#include <Arduino.h>
#include <driver/dac.h>

// Based on original work from Helmut Weber (https://github.com/MacLeod-D/ESP32-ADC)
// that he described at https://esp32.com/viewtopic.php?f=19&t=2881&start=30#p47663
// Modified with bug-fixed by Henry Cheung
//
// Build a ESP32 ADC Lookup table to correct ESP32 ADC linearity issue
// Run this sketch to build your own LUT for each of your ESP32, copy and paste the
// generated LUT to your sketch for using it, see example sketch on how to use it
//
// Version 2.0 - switch to use analogRead() instead of esp-idf function adcStart()
// Version 1.0 - original adoptation and bug fix based on Helmut Weber code

// #define GRAPH      // uncomment this for print on Serial Plotter
// #define FLOAT_LUT     // uncomment this if you need float LUT

#include "config.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#define ADC_PIN BATTERY_PIN    // GPIO 35 = A7, uses any valid Ax pin as you wish

float Results[4097];
float Res2[4096*5];

void dumpResults() {
  for (int i=0; i<4096; i++) {
    if (i % 16 == 0) {
      Serial.println();
      Serial.print(i); Serial.print(" - ");
    }
    Serial.print(Results[i], 2); Serial.print(", ");
  }
  Serial.println();
}

void dumpRes2() {
  Serial.println(F("Dump Res2 data..."));
  for (int i=0; i<(5*4096); i++) {
      if (i % 16 == 0) {
        Serial.println(); Serial.print(i); Serial.print(" - ");
      }
      Serial.print(Res2[i],3); Serial.print(", ");
    }
    Serial.println();
}

void setup() {
    dac_output_enable(DAC_CHANNEL_1);    // pin 25
    dac_output_voltage(DAC_CHANNEL_1, 0);
    analogReadResolution(12);
    Serial.begin(BAUDRATE);
    delay(1000);
}

void loop() {

    Serial.print(F("Test Linearity "));
    for (int j=0; j<500; j++) {
      if (j % 100 == 0) Serial.print(".");
      for (int i=0;i<256;i++) {
          dac_output_voltage(DAC_CHANNEL_1, (i & 0xff));
          delayMicroseconds(100);
          Results[i*16]=0.9*Results[i*16] + 0.1*analogRead(ADC_PIN);
      }
    }
    Serial.println();
    // dumpResults();

    Serial.println(F("Calculate interpolated values .."));
    Results[4096] = 4095.0;
    for (int i=0; i<256; i++) {
       for (int j=1; j<16; j++) {
          Results[i*16+j] = Results[i*16] + (Results[(i+1)*16] - Results[(i)*16])*(float)j / (float)16.0;
       }
    }
    // dumpResults();

    Serial.println(F("Generating LUT .."));
    for (int i=0; i<4096; i++) {
        Results[i]=0.5 + Results[i];
    }
    // dumpResults();

    Results[4096]=4095.5000;
    for (int i=0; i<4096; i++) {
       for (int j=0; j<5; j++) {
          Res2[i*5+j] = Results[i] + (Results[(i+1)] - Results[i]) * (float)j / (float)10.0;
       }
    }
    // dumpRes2();

    for (int i=1; i<4096; i++) {
        int index;
        float minDiff=99999.0;
        for (int j=0; j<(5*4096); j++) {
            float diff=fabs((float)(i) - Res2[j]);
            if(diff<minDiff) {
                minDiff=diff;
                index=j;
            }
        }
        Results[i]=(float)index;
    }
    // dumpResults();

    for (int i=0; i<(4096); i++) {
        Results[i]/=5;
    }

#ifdef GRAPH

    while(1) {
      for (int i=2; i<256; i++) {
        dac_output_voltage(DAC_CHANNEL_1, (i & 0xff));
        delayMicroseconds(100);
        float r = Results[analogRead(ADC_PIN)];
        Serial.print(i*16); Serial.print(" "); Serial.println(r);
      }
    }

#else

    Serial.println();

#ifdef FLOAT_LUT
    Serial.println("const float ADC_LUT[4096] = { 0,");
    for (int i=1; i<4095; i++) {
       Serial.print(Results[i],4); Serial.print(",");
       if ((i%15)==0) Serial.println();
    }
    Serial.println(Results[4095]);
    Serial.println("};");
#else
    Serial.println("const int16_t ADC_LUT[4096] = { 0,");
    for (int i=1; i<4095; i++) {
      Serial.print((int)Results[i]); Serial.print(",");
      if ((i%15)==0) Serial.println();
    }
    Serial.println((int)Results[4095]);
    Serial.println("};");
#endif
    while(1);
#endif

}