#include "acurite-sensor.h"
#include "esphome/components/wifi/wifi_component.h"

AcuriteSensor *AcuriteSensor::instance = nullptr; // Initialize static instance pointer

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