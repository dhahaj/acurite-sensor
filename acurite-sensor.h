#ifndef RFSENSOR_H
#define RFSENSOR_H

#include "esphome.h"

#define RING_BUFFER_SIZE 256

#define SYNC_LENGTH 2200

#define SYNC_HIGH 600
#define SYNC_LOW 600
#define BIT1_HIGH 400
#define BIT1_LOW 220
#define BIT0_HIGH 220
#define BIT0_LOW 400

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
  static RFSensor *instance; // Static instance pointer

  /// @brief Interrupt handler
  static void IRAM_ATTR interrupt_handler()
  {
    if (instance)
    {
      instance->handle_interrupt();
    }
  }

  /// @brief Handle interrupt
  void handle_interrupt()
  {
    static unsigned long lastTime = 0;
    static unsigned int ringIndex = 0;
    static unsigned int syncCount = 0;


    // ignore if we haven't processed the previous received signal
    if (received)
      return;

    interruptFlag = true;
    
    // calculating timing since last change
    unsigned long time = micros();
    unsigned long duration = time - lastTime;
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
      {
        syncIndex1 = (ringIndex + 1) % RING_BUFFER_SIZE;
      }
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
        {
          received = true;
        }
      }
    }
    interruptFlag = false;
  }

public:
  bool isSync(unsigned int idx);
  int t2b(unsigned int t0, unsigned int t1);
  unsigned long extractData(unsigned int syncIndex, unsigned int bits, unsigned int startPos, unsigned int size);

  RFSensor()
  {
    instance = this; // Set the instance pointer in the constructor
  }

  void setup() override
  {
    Serial.begin(115200);
    Serial.println("RF Sensor setup");
    pinMode(DATAPIN, INPUT);
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);

    // delay(1000);

    Serial.println("enabling interrupts...");
    attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::interrupt_handler, CHANGE);
  }

  void loop() override
  {
    if (!received || interruptFlag)
    {
      return;
    }

    digitalWrite(LEDPIN, LOW);
    Serial.println("Received data");

    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(digitalPinToInterrupt(DATAPIN));

    unsigned int startIndex, stopIndex;
    unsigned long humidity = 0;
    bool fail = false;
    startIndex = (syncIndex1 + (3 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex = (syncIndex1 + (3 * 8 + 8) * 2) % RING_BUFFER_SIZE;

    // extract humidity value
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      humidity = (humidity << 1) + bit;
      if (bit < 0)
        fail = true;
    }

    // publish humidity value
    if (!fail)
    {
      humidity_sensor->publish_state(humidity);
      Serial.print("Humidity: ");
      Serial.println(humidity);
    }
    else
    {
      Serial.println("Decoding error.");
    }

    // extract temperature value
    unsigned long temp = 0;
    fail = false;

    // // most significant 4 bits
    startIndex = (syncIndex1 + (4 * 8 + 4) * 2) % RING_BUFFER_SIZE;
    stopIndex = (syncIndex1 + (4 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)
        fail = true;
    }

    // // least significant 7 bits
    startIndex = (syncIndex1 + (5 * 8 + 1) * 2) % RING_BUFFER_SIZE;
    stopIndex = (syncIndex1 + (5 * 8 + 8) * 2) % RING_BUFFER_SIZE;
    for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
    {
      int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
      temp = (temp << 1) + bit;
      if (bit < 0)
        fail = true;
    }

    // // publish temperature value
    if (!fail)
    {
      unsigned long result = (int)((temp - 1024) / 10 + 1.9 + 0.5);
      temperature_sensor->publish_state(result);

      Serial.print("Temperature: ");
      Serial.print(result);
      Serial.println("°C");
    }
    else
    {
      Serial.println("Decoding error.");
    }

    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;

    digitalWrite(LEDPIN, HIGH);

    // re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(DATAPIN), RFSensor::interrupt_handler, CHANGE);
  }
};


#endif // RFSENSOR_H