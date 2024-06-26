#pragma once
#define min // workaround: nimble's min/max macros conflict with libstdc++
#define max
#include <host/ble_gap.h>
#include <atomic>
#undef max
#undef min

#include "utility/CircularBuffer.h"

namespace Pinetime {
  namespace Controllers {
    class NimbleController;
    class MotionController;

    class MotionService {
    public:
      MotionService(NimbleController& nimble, Controllers::MotionController& motionController);
      void Init();
      int OnStepCountRequested(uint16_t attributeHandle, ble_gatt_access_ctxt* context);
      void OnNewStepCountValue(uint32_t stepCount);
      void OnNewMotionValues(int16_t *fifo, uint16_t nFifo);

      void SubscribeNotification(uint16_t attributeHandle);
      void UnsubscribeNotification(uint16_t attributeHandle);
      bool IsMotionNotificationSubscribed() const;


    private:
      NimbleController& nimble;
      Controllers::MotionController& motionController;

      struct ble_gatt_chr_def characteristicDefinition[4];
      struct ble_gatt_svc_def serviceDefinition[2];
      int16_t *data;
      int16_t nData;

      uint16_t stepCountHandle;
      uint16_t motionValuesHandle;
      uint16_t osdStatusHandle;
      std::atomic_bool stepCountNoficationEnabled {false};
      std::atomic_bool motionValuesNoficationEnabled {false};

      // A small buffer
      static constexpr uint8_t accBufSize = 9; // The number of measurements to acquire before notifying subscribers of new data.
      //Utility::CircularBuffer<int16_t, accBufSize> accBuf = {};
      int16_t accBuf[accBufSize] = {};
      uint16_t accBufIdx = 0;
      uint16_t testVal = 0;
    };
  }
}
