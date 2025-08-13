/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Based on https://github.com/AK-Homberger/NMEA2000-Data-Sender Version 0.7, 28.01.2022, AK-Homberger
// Based on theory of operation detailed in https://bikerglen.com/blog/building-a-synchro-to-digital-converter/

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include "esp_mac.h"
#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <memory>
#include <N2kMessages.h>

#define ENABLE_DEBUG_LOG 0 // Debug log

int NodeAddress;  // To store last Node Address

Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {130306L, // Wind Data (Speed and Direction)
                                                 0
                                                 };


// Wind speed data

#define WindSpeed_Window 16 // Sliding average window size >= 1
#define WindSpeed_Divisor 100 // Divide pulses to compute speed in m/s
#define WindSpeedPin 33  // Wind Speed is measured as interrupt on GPIO 33

volatile uint64_t StartValue = 0;                  // First interrupt value
volatile uint64_t PeriodCount = 0;                // period in counts of 0.001 of a second
volatile unsigned long Last_int_time = 0;
hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t


// Wind angle data

#define WindAngle_Window 16                  // Sliding average window size >= 1

volatile float Excite = 0;                     // Excitation signal, -1 or +1, 0 = undefined
float S1_S3 = 0;                    // S1 - S3 (Yellow - Blue)
float S3_S2 = 0;                    // S3 - S2 (Blue - Black) 
float theta = 0;                    // Angle for computation
float sinIn = 0;                    // Resolver format S1_S3
float cosIn = 0;                    // Resolver format combined
float delta = 0;                    // Error term
float demod = 0;                    // Demodulated error term
uint64_t samples = 0;                 // Count samples

const int S1_S3_offset = 123; // ADC offset to correct for input at 0
const int S3_S2_offset = 123; // ADC offset to correct for input at 0

// Declare array of ADC pins that will be used for ADC Continuous mode - ONLY ADC1 pins are supported
// Number of selected pins can be from 1 to ALL ADC1 pins.
uint8_t ADCpins[] = {34, 35};  // S1_S3=34, S3_S2=35

// Calculate how many pins are declared in the array - needed as input for the setup function of ADC Continuous
uint8_t ADCpins_count = 2;

// Direction Excitation is connected GPIO 14 (TMS)
const int ExcitePin = 14;

// Global Data

#define WindSendOffset 0
#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent

float WindSpeed = 0;  // m/s
float WindAngle = 0;  // radians

void debug_log(char* str) {
#if ENABLE_DEBUG_LOG == 1
  Serial.println(str);
#endif
}

// Wind Speed Event Interrupt
// Enters on falling edge
//=======================================
void IRAM_ATTR handleWindInterrupt()
{
  Last_int_time = millis();
  PeriodCount = Last_int_time - StartValue;         // period count between rising edges in 0.001 of a second
  StartValue = Last_int_time;                       // puts latest reading as start for next calculation
}


void setup() {

  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;

  // Init USB serial port
  Serial.begin(115200);

  // Init WindSpeed measure
  
  pinMode(WindSpeedPin, INPUT_PULLUP);                                                // sets pin high
  attachInterrupt(digitalPinToInterrupt(WindSpeedPin), handleWindInterrupt, FALLING); // attaches pin to interrupt on Falling Edge

  // Init Excitation Voltage Sign Detection
  
  pinMode(ExcitePin, INPUT_PULLUP);                                            // sets pin high
  //attachInterrupt(digitalPinToInterrupt(ExcitePin), handleExciteInterrupt, CHANGE); // attaches pin to interrupt on rising and falling edge

 
 // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega

  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetN2kCANSendFrameBufSize(250);

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "My Wind Data Module",  // Manufacturer's Model ID
                                 "1.0.0.0 (2025-07-31)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2025-07-31)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
                                130, // Device function=Atmospheric. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=External Environment. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2045 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 33);  // Read stored last NodeAddress, default 33
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();

  delay(200);
}



// Calculate Wind Speed from number of interupts per time

double ReadWindSpeed() {
  double WindSpeed = 0;

  if (PeriodCount != 0) {                         // 0 means no signals measured
    WindSpeed = 1000.00 / (PeriodCount * WindSpeed_Divisor);            // PeriodCount in 0.001 of a second  
  }  
  if (millis() > Last_int_time + 500) WindSpeed = 0;       // No signals WindSpeed=0;
  return (WindSpeed);
}

// Compute Wind Angle from theta

double ComputeWindAngle() {
  double WindAngle = 0;

  WindAngle = theta;
  
  return(WindAngle);
}


bool IsTimeToUpdate(unsigned long NextUpdate) {
  return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
  return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
  while ( NextUpdate < millis() ) NextUpdate += Period;
}



void SendN2kWindData(double WindSpeed, double WindAngle) {
  static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, WindSendOffset);
  tN2kMsg N2kMsg;

  if ( IsTimeToUpdate(SlowDataUpdated) ) {
    SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);

    Serial.printf("Wind Speed  :%4.1f m/s \n", WindSpeed);
    Serial.printf("Wind Angle  :%4.4f radians \n", WindAngle);
    Serial.printf("S1_S3  :%4.4f \n", S1_S3);
    Serial.printf("S3_S2  :%4.4f \n", S3_S2);
    Serial.printf("sinIn  :%4.4f \n", sinIn);
    Serial.printf("cosIn  :%4.4f \n", cosIn);
    Serial.printf("theta  :%4.4f \n", theta);
    Serial.printf("delta  :%4.4f \n", delta);
    Serial.printf("Excite  :%4.4f \n", Excite);
    Serial.printf("demod  :%4.4f \n", demod);
    Serial.printf("samples  :%8i \n", samples);
    samples = 0; // Reset to get rough frequency

    // SetN2kWindSpeed(tN2kMsg &N2kMsg, unsigned char SID, double WindSpeed, double WindAngle, tN2kWindReference WindReference)
    SetN2kWindSpeed(N2kMsg, 0, WindSpeed, WindAngle, N2kWind_Apparent);
    NMEA2000.SendMsg(N2kMsg);
  }
}



void loop() {
  // Run full speed to get ADC samples for angle computations
  samples = samples + 1;
  S1_S3 = (analogRead(ADCpins[0]) + S1_S3_offset - 2048.0) / 4096.0;
  S3_S2 = (analogRead(ADCpins[1]) + S3_S2_offset - 2048.0) / 4096.0;
    
  // Scott transform the inputs
  sinIn = S1_S3;
  // cosIn = 2/sqrt(3) * (S3_S2 + 0.5 * S1_S3)
  cosIn = 1.1574 * (S3_S2 + 0.5 * S1_S3);
      
  // Compute error using the identity described in chapter 3 of the Analog Devices handbook
  delta = sinIn * cos(theta) - cosIn * sin(theta);

  // Read excitation voltage for sign
  if (digitalRead(ExcitePin) == LOW) {
    Excite = -1.0;
  } else {
    Excite = 1.0;
  }
 
  // Demodulate AC error term
  demod = Excite * delta;
    
  // Apply gain to demodulated error and integrate
  // theta = theta + 1/64*demod
  theta = theta + 0.015625 * demod;
    
  // Wrap from -pi to +pi
  theta = fmod((theta + PI),(2 * PI)) - PI;
  
  WindSpeed = ((WindSpeed * (WindSpeed_Window - 1)) + (ReadWindSpeed())) / WindSpeed_Window; // This implements a low pass filter to eliminate spikes

  WindAngle = ((WindAngle * (WindAngle_Window - 1)) + (ComputeWindAngle())) / WindAngle_Window; // This implements a low pass filter to eliminate spikes

  SendN2kWindData(WindSpeed, WindAngle);

  NMEA2000.ParseMessages();
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) {
    Serial.read();
  }

}
