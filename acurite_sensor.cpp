#include "acurite-sensor.h"
#include "esphome/components/wifi/wifi_component.h"

// namespace esphome {
// namespace AcuriteSensor {

    static const char *const TAG = "acurite-sensor";

    AcuriteSensor *AcuriteSensor::instance = nullptr; // Initialize static instance pointer

    void AcuriteSensor::setup(void)
    {
        Serial.begin(115200);
        ESP_LOGV(TAG, "Setting up acurite-sensor.");
        pinMode(DATAPIN, INPUT);
        pinMode(LEDPIN, OUTPUT);
        digitalWrite(LEDPIN, LOW);
    }

    void AcuriteSensor::loop(void)
    {
        if (!check_wifi_status())
        {
            return;
        }
        else if (!connected)
        {
            ESP_LOGV(TAG, "WiFi Connected.");
            attachInterrupt(digitalPinToInterrupt(DATAPIN), AcuriteSensor::interrupt_handler, CHANGE);
            connected = true;
        }

        if (!received || interruptFlag)
        {
            return;
        }

        digitalWrite(LEDPIN, LOW);

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
            int bit = timeToBits(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
            humidity = (humidity << 1) + bit;
            if (bit < 0)
                fail = true;
        }

        // publish humidity value
        if (!fail)
        {
            humidity_sensor->publish_state(humidity);
            ESP_LOGV(TAG, "Humidity: %1u", humidity);
        }
        else
        {
            ESP_LOGV(TAG, "Decoding Error");
        }

        // extract temperature value
        unsigned long temp = 0;
        fail = false;

        // most significant 4 bits
        startIndex = (syncIndex1 + (4 * 8 + 4) * 2) % RING_BUFFER_SIZE;
        stopIndex = (syncIndex1 + (4 * 8 + 8) * 2) % RING_BUFFER_SIZE;
        for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
        {
            int bit = timeToBits(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
            temp = (temp << 1) + bit;
            if (bit < 0)
                fail = true;
        }

        // least significant 7 bits
        startIndex = (syncIndex1 + (5 * 8 + 1) * 2) % RING_BUFFER_SIZE;
        stopIndex = (syncIndex1 + (5 * 8 + 8) * 2) % RING_BUFFER_SIZE;
        for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
        {
            int bit = timeToBits(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
            temp = (temp << 1) + bit;
            if (bit < 0)
                fail = true;
        }

        // publish temperature value
        if (!fail)
        {
            unsigned long result = (int)((temp - 1024) / 10 + 1.9 + 0.5);
            temperature_sensor->publish_state(result);
            ESP_LOGV(TAG, "Temperature: %1u", result);
        }
        else
        {
            ESP_LOGV(TAG, "Decoding Error");
        }

        received = false;
        syncIndex1 = 0;
        syncIndex2 = 0;

        digitalWrite(LEDPIN, HIGH);

        // re-enable interrupt
        attachInterrupt(digitalPinToInterrupt(DATAPIN), AcuriteSensor::interrupt_handler, CHANGE);
    }

    // detect if a sync signal is present
    bool AcuriteSensor::isSync(unsigned int idx)
    {
        // check if we've received 4 squarewaves of matching timing
        int i;
        for (i = 0; i < 8; i += 2)
        {
            unsigned long t1 = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
            unsigned long t0 = timings[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];
            if (t0 < (SYNC_HIGH - 100) || t0 > (SYNC_HIGH + 100) ||
                t1 < (SYNC_LOW - 100) || t1 > (SYNC_LOW + 100))
            {
                return false;
            }
        }

        // check if there is a long sync period prior to the 4 squarewaves
        unsigned long t = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
        if (t < (SYNC_LENGTH - 400) || t > (SYNC_LENGTH + 400) || digitalRead(DATAPIN) != HIGH)
        {
            return false;
        }
        return true;
    }

    int AcuriteSensor::timeToBits(unsigned int t0, unsigned int t1)
    {
        if (t0 > (BIT1_HIGH - 100) && t0 < (BIT1_HIGH + 100) &&
            t1 > (BIT1_LOW - 100) && t1 < (BIT1_LOW + 100))
        {
            return 1;
        }
        else if (t0 > (BIT0_HIGH - 100) && t0 < (BIT0_HIGH + 100) &&
                 t1 > (BIT0_LOW - 100) && t1 < (BIT0_LOW + 100))
        {
            return 0;
        }
        return -1; // undefined
    }

    bool AcuriteSensor::check_wifi_status()
    {
        return esphome::wifi::global_wifi_component->is_connected();
    }

    /// @brief Handle interrupt
  void AcuriteSensor::handle_interrupt()
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

// } // acurite-sensor
// } // esphome

// #include "acurite-sensor.h"
// #include "esphome/components/wifi/wifi_component.h"

// // RFSensor *RFSensor::instance = nullptr; // Initialize static instance pointer

// // detect if a sync signal is present
// // bool RFSensor::isSync(unsigned int idx)
// // {
// //     // check if we've received 4 squarewaves of matching timing
// //     int i;
// //     for (i = 0; i < 8; i += 2)
// //     {
// //         unsigned long t1 = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
// //         unsigned long t0 = timings[(idx + RING_BUFFER_SIZE - i - 1) % RING_BUFFER_SIZE];
// //         if (t0 < (SYNC_HIGH - 100) || t0 > (SYNC_HIGH + 100) ||
// //             t1 < (SYNC_LOW - 100) || t1 > (SYNC_LOW + 100))
// //         {
// //             return false;
// //         }
// //     }

// //     // check if there is a long sync period prior to the 4 squarewaves
// //     unsigned long t = timings[(idx + RING_BUFFER_SIZE - i) % RING_BUFFER_SIZE];
// //     if (t < (SYNC_LENGTH - 400) || t > (SYNC_LENGTH + 400) || digitalRead(DATAPIN) != HIGH)
// //     {
// //         return false;
// //     }
// //     return true;
// // }

// int RFSensor::t2b(unsigned int t0, unsigned int t1)
// {
//     if (t0 > (BIT1_HIGH - 100) && t0 < (BIT1_HIGH + 100) &&
//         t1 > (BIT1_LOW - 100) && t1 < (BIT1_LOW + 100))
//     {
//         return 1;
//     }
//     else if (t0 > (BIT0_HIGH - 100) && t0 < (BIT0_HIGH + 100) &&
//              t1 > (BIT0_LOW - 100) && t1 < (BIT0_LOW + 100))
//     {
//         return 0;
//     }
//     return -1; // undefined
// }

// // unsigned long RFSensor::extractData(unsigned int syncIndex, unsigned int bits, unsigned int startPos, unsigned int size)
// // {
// //     unsigned int startIndex, stopIndex;
// //     unsigned long value = 0;
// //     bool fail = false;
// //     startIndex = (syncIndex + (bits * 8 + startPos) * 2) % RING_BUFFER_SIZE;
// //     stopIndex = (syncIndex + (bits * 8 + size) * 2) % RING_BUFFER_SIZE;

// //     for (int i = startIndex; i != stopIndex; i = (i + 2) % RING_BUFFER_SIZE)
// //     {
// //         int bit = t2b(timings[i], timings[(i + 1) % RING_BUFFER_SIZE]);
// //         value = (value << 1) + bit;
// //         if (bit < 0)
// //             fail = true;
// //     }

// //     if (fail)
// //         return -1;

// //     return value;
// // }

// bool RFSensor::check_wifi_status()
// {
//     return esphome::wifi::global_wifi_component->is_connected();
// }