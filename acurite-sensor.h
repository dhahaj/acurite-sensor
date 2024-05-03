#pragma once

#include "esphome.h"
#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/components/wifi/wifi_component.h"

#define RING_BUFFER_SIZE 256

#define SYNC_LENGTH 2200

#define SYNC_HIGH 600
#define SYNC_LOW 600
#define BIT1_HIGH 400
#define BIT1_LOW 220
#define BIT0_HIGH 220
#define BIT0_LOW 400
#define BUFFER_SIZE 512
#define DATAPIN 14
#define LEDPIN 2

/// @brief RF Sensor component
class RFSensor : public esphome::Component
{
public:
  esphome::sensor::Sensor *temperature_sensor = new esphome::sensor::Sensor();
  esphome::sensor::Sensor *humidity_sensor = new esphome::sensor::Sensor();

private:
  volatile unsigned long timings[RING_BUFFER_SIZE];
  volatile unsigned int syncIndex1 = 0; // index of the first sync signal
  volatile unsigned int syncIndex2 = 0; // index of the second sync signal
  volatile bool received = false;
  volatile bool interruptFlag = false;
  volatile unsigned long _temp, _hum;
  char buffer[BUFFER_SIZE];
  uint32_t lastTime = 0;
  bool flag = true;

  // static RFSensor *instance; // Static instance pointer

  /// @brief Interrupt handler
  // static void IRAM_ATTR interrupt_handler()
  // {
  //   if (instance)
  //   {
  //     instance->handle_interrupt();
  //   }
  // }

  /// @brief Handle interrupt
  // void handle_interrupt()
  // {
  //   static unsigned long lastTime = 0;
  //   static unsigned int ringIndex = 0;
  //   static unsigned int syncCount = 0;

  //   // ignore if we haven't processed the previous received signal
  //   if (received)
  //     return;

  //   interruptFlag = true;

  //   // calculating timing since last change
  //   unsigned long time = micros();
  //   unsigned long duration = time - lastTime;
  //   lastTime = time;

  //   // store data in ring buffer
  //   ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  //   timings[ringIndex] = duration;

  //   // detect sync signal
  //   if (isSync(ringIndex))
  //   {
  //     syncCount++;
  //     // first time sync is seen, record buffer index
  //     if (syncCount == 1)
  //     {
  //       syncIndex1 = (ringIndex + 1) % RING_BUFFER_SIZE;
  //     }
  //     else if (syncCount == 2)
  //     {
  //       // second time sync is seen, start bit conversion
  //       syncCount = 0;
  //       syncIndex2 = (ringIndex + 1) % RING_BUFFER_SIZE;
  //       unsigned int changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2 + RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
  //       // changeCount must be 122 -- 60 bits x 2 + 2 for sync
  //       if (changeCount != 122)
  //       {
  //         received = false;
  //         syncIndex1 = 0;
  //         syncIndex2 = 0;
  //       }
  //       else
  //       {
  //         received = true;
  //       }
  //     }
  //   }
  //   interruptFlag = false;
  // }

public:
  // bool isSync(unsigned int idx);

  // detect if a sync signal is present
  bool isSync(unsigned int idx)
  {
    // check if we've received 4 squarewaves of matching timing
    int i;
    for (i = 0; i < 8; i += 2)
    {
      unsigned long t1 = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
      unsigned long t0 = timings[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];
      if (t0 < (SYNC_HIGH - 100) || t0 > (SYNC_HIGH + 100) || t1 < (SYNC_LOW - 100) || t1 > (SYNC_LOW + 100))
        return false;
    }

    // check if there is a long sync period prior to the 4 squarewaves
    unsigned long t = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];

    if (t < (SYNC_LENGTH - 400) || t > (SYNC_LENGTH + 400) || digitalRead(DATAPIN) != HIGH)
      return false;

    return true;
  }

  IRAM_ATTR void ISR()
  {
    static unsigned long duration = 0, lastTime = 0;
    static unsigned int ringIndex = 0, syncCount = 0;

    // ignore if we haven't processed the previous received signal
    if (received == true)
      return;

    // calculating timing since last change
    long time = micros();
    duration = time - lastTime;
    lastTime = time;

    // store data in ring buffer
    ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
    timings[ringIndex] = duration;

    // detect sync signal
    if (isSync(ringIndex))
    {
      syncCount++;
      // first time sync is seen, record buffer index
      if (syncCount == 1)
        syncIndex1 = (ringIndex + 1) % RING_BUFFER_SIZE;
      else if (syncCount == 2)
      {
        // second time sync is seen, start bit conversion
        syncCount = 0;
        syncIndex2 = (ringIndex + 1) % RING_BUFFER_SIZE;
        unsigned int changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2 + RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
        // changeCount must be 122 -- 60 bits x 2 + 2 for sync
        if (changeCount != 122)
        {
          received = false;
          syncIndex1 = 0;
          syncIndex2 = 0;
        }
        else
          received = true;
      }
    }
  }

  void handleReceive()
  {
    detachInterrupt(digitalPinToInterrupt(DATAPIN));
    // extract humidity value
    unsigned long humidity, temp;
    unsigned int startIndex, stopIndex;
    bool fail = false;
    startIndex = (syncIndex1 + (3 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex =  (syncIndex1 + (3 * 8 + 8) * 2) % RING_BUFFER_SIZE;

    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      humidity = (humidity << 1) + bit;
      if (bit < 0)
        fail = true;
    }


    if (fail)
      Serial.println(F("Decoding error."));
    else {
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.print("\% / ");
    }

    fail = false;

    // most significant 4 bits
    startIndex = (syncIndex1 + (4 * 8 + 4) * 2) % RING_BUFFER_SIZE;
    stopIndex  = (syncIndex1 + (4 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)  fail = true;
    }

    // least significant 7 bits
    startIndex = (syncIndex1 + (5 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex  = (syncIndex1 + (5 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)
        fail = true;
    }

    if (fail)
      Serial.println(F("Decoding error."));
    else
    {
      Serial.print(F("Temperature: "));
      Serial.print((int)((temp - 1024) / 10 + 1.9 + 0.5)); // round to the nearest integer
      Serial.write(176);    // degree symbol
      Serial.print(F("C/"));
      Serial.print((int)(((temp - 1024) / 10 + 1.9 + 0.5) * 9 / 5 + 32)); // convert to F
      Serial.write(176);    // degree symbol
      Serial.println(F("F"));
    }

    // delay for 1 second to avoid repetitions
    delay(1000);
    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;

    // re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::ISR, CHANGE);

  }
  
  // detect if a sync signal is present
  // bool isSync(unsigned int idx)
  // {
  //   // check if we've received 4 squarewaves of matching timing
  //   int i;
  //   for (i = 0; i < 8; i += 2)
  //   {
  //     unsigned long t1 = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
  //     unsigned long t0 = timings[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];
  //     if (t0 < (SYNC_HIGH - 100) || t0 > (SYNC_HIGH + 100) || t1 < (SYNC_LOW - 100) || t1 > (SYNC_LOW + 100))
  //       return false;
  //   }

  //   // check if there is a long sync period prior to the 4 squarewaves
  //   unsigned long t = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];

  //   if (t < (SYNC_LENGTH - 400) || t > (SYNC_LENGTH + 400) || digitalRead(DATAPIN) != HIGH)
  //     return false;

  //   return true;
  // }

  int t2b(unsigned int t0, unsigned int t1);
  // unsigned long extractData(unsigned int syncIndex, unsigned int bits, unsigned int startPos, unsigned int size);
  bool check_wifi_status(void);

  // RFSensor()
  // {
  //   instance = this; // Set the instance pointer in the constructor
  // }

  void setup() override
  {
    Serial.begin(115200);
    // pinMode(DATAPIN, INPUT);
    // pinMode(LEDPIN, OUTPUT);
    // digitalWrite(LEDPIN, LOW);

    // attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::interrupt_handler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::ISR, CHANGE);
  }

  // bool connected = false;

  void loop() override
  {
    if (received == true)
    {
      handleReceive();
    }

    // Checking WiFi status here to avoid entering a bootloop
    // if (!check_wifi_status())
    // {
    //   return;
    // }
    // else if (!connected)
    // {
    //   Serial.println("WiFi Connected.");
    //   attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::interrupt_handler, CHANGE); // attach the interrupt
    //   connected = true;
    // }

    // if (!received || interruptFlag)
    // {
    //   return;
    // }

    // digitalWrite(LEDPIN, LOW);

    // // disable interrupt to avoid new data corrupting the buffer
    // detachInterrupt(digitalPinToInterrupt(DATAPIN));

    // unsigned int startIndex, stopIndex;
    // unsigned long humidity = 0;
    // bool fail = false;
    // startIndex = (syncIndex1 + (3 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    // stopIndex = (syncIndex1 + (3 * 8 + 8) * 2) % RING_BUFFER_SIZE;

    // // extract humidity value
    // for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    // {
    //   int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
    //   humidity = (humidity << 1) + bit;
    //   if (bit < 0)
    //     fail = true;
    // }

    // // publish humidity value
    // if (!fail)
    // {
    //   humidity_sensor->publish_state(humidity);
    // }

    // // extract temperature value
    // unsigned long temp = 0;
    // fail = false;

    // // most significant 4 bits
    // startIndex = (syncIndex1 + (4 * 8 + 4) * 2) % RING_BUFFER_SIZE;
    // stopIndex = (syncIndex1 + (4 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    // for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    // {
    //   int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
    //   temp = (temp << 1) + bit;
    //   if (bit < 0)
    //     fail = true;
    // }

    // // least significant 7 bits
    // startIndex = (syncIndex1 + (5 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    // stopIndex = (syncIndex1 + (5 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    // for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    // {
    //   int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
    //   temp = (temp << 1) + bit;
    //   if (bit < 0)
    //     fail = true;
    // }

    // // publish temperature value
    // if (!fail)
    // {
    //   unsigned long result = (int)((temp - 1024) / 10 + 1.9 + 0.5);
    //   temperature_sensor->publish_state(result);
    // }

    // received = false;
    // syncIndex1 = 0;
    // syncIndex2 = 0;

    // digitalWrite(LEDPIN, HIGH);

    // // re-enable interrupt
    // attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::interrupt_handler, CHANGE);
  }
};