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

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include "esp_mac.h"
#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <memory>
#include <N2kMessages.h>

#define ENABLE_DEBUG_LOG 0 // Debug log

#define ADC_Calibration_Value1 250.0 // For resistor measure 5 Volt and 180 Ohm equals 100% plus 1K resistor.
#define ADC_Calibration_Value2 34.3  // The real value depends on the true resistor values for the ADC input (100K / 27 K).
#define ADC_Calibration_Value3 34.3  // The real value depends on the true resistor values for the ADC input (100K / 27 K).

int NodeAddress;  // To store last Node Address

Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {130306L, // Wind Data (Speed and Direction)
                                                 0
                                                 };


// Wind speed data

#define WindSpeed_Window 16 // Sliding average window size >= 1
#define WindSpeed_Divisor 100000 // Divide pulses to compute speed in m/s
#define WindSpeed_Pin 33  // Wind Speed is measured as interrupt on GPIO 33

volatile uint64_t StartValue = 0;                  // First interrupt value
volatile uint64_t PeriodCount = 0;                // period in counts of 0.000001 of a second
unsigned long Last_int_time = 0;
hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?


// Wind angle data

#define WindAngle_Window 16 // Sliding average window size >= 1
int DirPhase = 0;                                  // Phase to read, 0-2

// Direction Excitation is connected GPIO 14 (TMS)
const int Excite_Pin = 14;

// Direction Phase 0 is connected GPIO 34 (Analog ADC1_CH6)
// Direction Phase 1 is connected GPIO 35 (Analog ADC1_CH7)
// Direction Phase 2 is connected GPIO 39 (Analog ADC1_CH3)
const int ADCpin[3] = {34,35,39};

uint16_t DirPhaseVal[3]; // 0-3.3v = 0-4095
const double Phase0Max = 4096;
const double Phase0Min = 0;
const double Phase1Max = 4096;
const double Phase1Min = 0;
const double Phase2Max = 4096;
const double Phase2Min = 0;

// Global Data

#define WindSendOffset 0
#define SlowDataUpdatePeriod 1000  // Time between CAN Messages sent

float WindSpeed = 0;  // m/s
float WindAngle = 0;  // radians
float Phase0 = 0;  // count
float Phase1 = 0;  // count
float Phase2 = 0;  // count

// Serial port 2 config (GPIO 16)
const int baudrate = 38400;
const int rs_config = SERIAL_8N1;


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
  portENTER_CRITICAL_ISR(&mux);
  uint64_t TempVal = timerRead(timer);        // value of timer at interrupt
  PeriodCount = TempVal - StartValue;         // period count between rising edges in 0.000001 of a second
  StartValue = TempVal;                       // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
  Last_int_time = millis();
}

void IRAM_ATTR handleExciteInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  DirPhaseVal[DirPhase] = ReadVoltage(ADCpin[DirPhase]);
  DirPhase = DirPhase + 1;
  if (DirPhase == 3) {
    DirPhase = 0;
  }
  portEXIT_CRITICAL_ISR(&mux);
}


void setup() {

  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;

  // Init USB serial port
  Serial.begin(115200);

  // Init WindSpeed measure
  
  pinMode(WindSpeed_Pin, INPUT_PULLUP);                                            // sets pin high
  attachInterrupt(digitalPinToInterrupt(WindSpeed_Pin), handleWindInterrupt, FALLING); // attaches pin to interrupt on Falling Edge
  timer = timerBegin(1000000);                                                // this returns a pointer to the hw_timer_t global variable
  timerStart(timer);                                                              // starts the timer

  // Init Excitation Voltage Peak Detection
  
  pinMode(Excite_Pin, INPUT_PULLUP);                                            // sets pin high
  attachInterrupt(digitalPinToInterrupt(Excite_Pin), handleExciteInterrupt, RISING); // attaches pin to interrupt on Rising Edge

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
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2045 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 33);  // Read stored last NodeAddress, default 33
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);

  NMEA2000.Open();

  delay(200);
}



// Calculate Wind Speed from number of interupts per time

double ReadWindSpeed() {
  double WindSpeed = 0;

  portENTER_CRITICAL(&mux);
  if (PeriodCount != 0) {                            // 0 means no signals measured
    WindSpeed = 1000000.00 / PeriodCount;            // PeriodCount in 0.000001 of a second  
  }  
  portEXIT_CRITICAL(&mux);
  WindSpeed = WindSpeed / WindSpeed_Divisor;
  if (millis() > Last_int_time + 2000) WindSpeed = 0;       // No signals WindSpeed=0;
  return (WindSpeed);
}

// Compute Wind Angle from relative phase voltages

double ComputeWindAngle() {
  double WindAngle = 0;

  portENTER_CRITICAL(&mux);
  Phase0 = DirPhaseVal[0] - ((Phase0Max - Phase0Min) / 2);
  Phase1 = DirPhaseVal[1] - ((Phase1Max - Phase1Min) / 2);
  Phase2 = DirPhaseVal[2] - ((Phase2Max - Phase2Min) / 2);
//  DirPhaseVal[0] = 0;
//  DirPhaseVal[1] = 0;
//  DirPhaseVal[2] = 0;
  portEXIT_CRITICAL(&mux);

  double Phase0Angle = asin(Phase0/4096);
  double Phase1Angle = asin(Phase1/4096) + (120.0/180.0*3.14159);
  double Phase2Angle = asin(Phase2/4096) + (240.0/180.0*3.14159);
  
  WindAngle = (Phase0Angle + Phase1Angle + Phase2Angle) / 3.0;
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
    Serial.printf("DirPhase  :%1i \n", DirPhase);
    Serial.printf("Phase0  :%4.0f \n", Phase0);
    Serial.printf("Phase1  :%4.0f \n", Phase1);
    Serial.printf("Phase2  :%4.0f \n", Phase2);

    // SetN2kWindSpeed(tN2kMsg &N2kMsg, unsigned char SID, double WindSpeed, double WindAngle, tN2kWindReference WindReference)
    SetN2kWindSpeed(N2kMsg, 0, WindSpeed, WindAngle, N2kWind_Apparent);
    NMEA2000.SendMsg(N2kMsg);
  }
}


// ReadVoltage is used to improve the linearity of the ESP32 ADC see: https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function

double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1000;
} // Added an improved polynomial, use either, comment out as required



void loop() {
  unsigned int size;

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
