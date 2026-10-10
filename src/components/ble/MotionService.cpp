#include "components/ble/MotionService.h"
#include <cstring>
#include "components/motion/MotionController.h"
#include "components/ble/NimbleController.h"
#include <nrf_log.h>

using namespace Pinetime::Controllers;

/**
 ***********************************************************************************************************************************
 * Notes on How Accelerometer Data is retrieved and processed for OpenSeizureDetector.
 * 
 * SystemTask::Work() calls SystemTask::UpdateMotion() every loop.
 * SystemTask::UpdateMotion() calls BMA421::Process() 
 * BMA421::Process() reads the stored data from the FIFO on the chip and returns a struct containing the data 
 *    (struct Values in BMA421.h).  The returned struct contains the step count, an array fifo containing the data, 
 *    and nFifo, which is the number of elements in the fifo array.
 * SystemTask::UpdateMotion() calls MotionController::Update() with the new data received from BMA421::Process().
 * MotionController::Update() calls MotionService::OnNewMotionValues() with the new data (fifo[] and nFifo).
 * MotionService::OnNewMotionValues() creates a copy of the fifo buffer (lastMotionValues[]) and sends a notification to the phone
 *    with the new data.
 *    When the phone receives the notification, it will read the data, which results in 
 *    MotionService::OnStepCountRequested() being called.
 * MotionService::OnStepCountRequested() returns the lastMotionValues[] data.
 * 
 * The use of lastMotionValues[] is to avoid the data being updated betwen the notification being sent and the phone reading the data.
 ***********************************************************************************************************************************
 */


namespace {
  // 0003yyxx-78fc-48fe-8e23-433b3a1942d0
  constexpr ble_uuid128_t CharUuid(uint8_t x, uint8_t y) {
    return ble_uuid128_t {.u = {.type = BLE_UUID_TYPE_128},
                          .value = {0xd0, 0x42, 0x19, 0x3a, 0x3b, 0x43, 0x23, 0x8e, 0xfe, 0x48, 0xfc, 0x78, x, y, 0x03, 0x00}};
  }

  // 00030000-78fc-48fe-8e23-433b3a1942d0
  constexpr ble_uuid128_t BaseUuid() {
    return CharUuid(0x00, 0x00);
  }

  constexpr ble_uuid128_t motionServiceUuid {BaseUuid()};
  constexpr ble_uuid128_t stepCountCharUuid {CharUuid(0x01, 0x00)};
  constexpr ble_uuid128_t motionValuesCharUuid {CharUuid(0x02, 0x00)};
  constexpr ble_uuid128_t osdStatusCharUuid {CharUuid(0x78, 0x00)};    // 0x78 is 120 for OSD

  int MotionServiceCallback(uint16_t /*conn_handle*/, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt, void* arg) {
    auto* motionService = static_cast<MotionService*>(arg);
    return motionService->OnStepCountRequested(attr_handle, ctxt);
  }

}

// TODO Refactoring - remove dependency to SystemTask
MotionService::MotionService(NimbleController& nimble, Controllers::MotionController& motionController)
  : nimble {nimble},
    motionController {motionController},
    characteristicDefinition {{.uuid = &stepCountCharUuid.u,
                               .access_cb = MotionServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &stepCountHandle},
                              {.uuid = &motionValuesCharUuid.u,
                               .access_cb = MotionServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                               .val_handle = &motionValuesHandle},
                              {.uuid = &osdStatusCharUuid.u,
                               .access_cb = MotionServiceCallback,
                               .arg = this,
                               .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                               .val_handle = &osdStatusHandle},

                              {0}},
    serviceDefinition {
      {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &motionServiceUuid.u, .characteristics = characteristicDefinition},
      {0},
    } {
  // TODO refactor to prevent this loop dependency (service depends on controller and controller depends on service)
  motionController.SetService(this);
}

void MotionService::Init() {
  int res = 0;
  res = ble_gatts_count_cfg(serviceDefinition);
  ASSERT(res == 0);

  res = ble_gatts_add_svcs(serviceDefinition);
  ASSERT(res == 0);

  testVal = 0;
}

int MotionService::OnStepCountRequested(uint16_t attributeHandle, ble_gatt_access_ctxt* context) {
  int res = 0;
  if (attributeHandle == stepCountHandle) {
    NRF_LOG_INFO("Motion-stepcount : handle = %d", stepCountHandle);
    uint32_t buffer = motionController.NbSteps();

    res = os_mbuf_append(context->om, &buffer, 4);
    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  } else if (attributeHandle == motionValuesHandle) {
    // Serve the most recently notified samples. The snapshot is written by
    // OnNewMotionValues() (SystemTask context) and only read here (NimBLE host
    // task); the old code dereferenced a raw pointer into the live Bma421 FIFO
    // and hard-capped the payload at 3 frames.
    res = os_mbuf_append(context->om, lastMotionValues, lastMotionValuesCount * 3 * sizeof(int16_t));
    return (res == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  } else if (attributeHandle == osdStatusHandle) {
    int8_t statusVal = -1;
    res = os_mbuf_copydata(context->om, 0, 1, &statusVal);
    motionController.osdStatus = statusVal;
    motionController.osdStatusTime = xTaskGetTickCount();
    return res;
  }
  return 0;
}

void MotionService::OnNewStepCountValue(uint32_t stepCount) {
  if (!stepCountNoficationEnabled)
    return;

  uint16_t connectionHandle = nimble.connHandle();

  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  // Allocate only once we know the mbuf will be consumed. Allocating before
  // the connection check leaked one mbuf per call while disconnected; with
  // MSYS_1_BLOCK_COUNT == 12 that could exhaust the pool and provoke a NimBLE
  // host reset (doc/AccelerometerSampleRateAnalysis.md §8.5).
  uint32_t buffer = stepCount;
  auto* om = ble_hs_mbuf_from_flat(&buffer, 4);
  if (om == nullptr) {
    return;
  }

  ble_gattc_notify_custom(connectionHandle, stepCountHandle, om);
}

void MotionService::OnNewMotionValues(const int16_t* fifo, uint16_t nFifo) {
  if (!motionValuesNoficationEnabled || fifo == nullptr || nFifo == 0) {
    return;
  }

  uint16_t connectionHandle = nimble.connHandle();
  if (connectionHandle == 0 || connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
    return;
  }

  // Build the notification payload here and hand it to ble_gattc_notify_custom()
  // instead of letting the NimBLE host task pull the data back through the
  // access callback (which capped every notification at 3 frames and read a
  // raw pointer into the live Bma421 FIFO).
  // See doc/AccelerometerSampleRateAnalysis.md §8.1.

  // A notification must fit in a single ATT payload: negotiated MTU - 3 bytes,
  // MTU-3 bytes is because 3 bytes are needed for the packet header, so the available payload bytes are mtu-3.
  // The fallback of 20 bytes is in case the MTU has not been negotiated correctly - 20 is the default MTU of 23 - 3 header bytes.
  // The maximum number of bytes is rounded down to whole 6-byte frames (x,y,z), and bounded by the snapshot buffer.
  uint16_t mtu = ble_att_mtu(connectionHandle);
  uint16_t maxBytes = (mtu > 3) ? static_cast<uint16_t>(mtu - 3) : 20;
  maxBytes -= maxBytes % (3 * sizeof(int16_t));
  uint16_t nSend = nFifo;
  if (nSend > maxMotionValueFrames) {
    nSend = maxMotionValueFrames;
  }
  if (static_cast<uint32_t>(nSend) * 6 > maxBytes) {
    nSend = maxBytes / 6;
  }
  if (nSend == 0) {
    motionNotifyFailCount++;
    return;
  }

  const uint16_t payloadBytes = nSend * 3 * sizeof(int16_t);

  // Snapshot for the GATT READ path before handing the data to NimBLE.
  std::memcpy(lastMotionValues, fifo, payloadBytes);
  lastMotionValuesCount = nSend;

  auto* om = ble_hs_mbuf_from_flat(fifo, payloadBytes);
  if (om == nullptr) {
    motionNotifyFailCount++;
    return;
  }

  // Ownership of om passes to NimBLE on both success and failure
  // (ble_gattc_notify_custom frees it if the send fails).
  int rc = ble_gattc_notify_custom(connectionHandle, motionValuesHandle, om);
  if (rc != 0) {
    motionNotifyFailCount++;
    NRF_LOG_INFO("Motion notify failed: rc=%d (fail count=%d)", rc, motionNotifyFailCount);
  }
}

void MotionService::SubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == stepCountHandle)
    stepCountNoficationEnabled = true;
  else if (attributeHandle == motionValuesHandle)
    motionValuesNoficationEnabled = true;
}

void MotionService::UnsubscribeNotification(uint16_t attributeHandle) {
  if (attributeHandle == stepCountHandle)
    stepCountNoficationEnabled = false;
  else if (attributeHandle == motionValuesHandle)
    motionValuesNoficationEnabled = false;
}

bool MotionService::IsMotionNotificationSubscribed() const {
  return motionValuesNoficationEnabled;
}

