#pragma once
#include <cstdint>

namespace Pinetime {
  namespace Applications {
    namespace Display {
      enum class Messages : uint8_t {
        GoToSleep,
        GoToRunning,
        UpdateDateTime,
        UpdateBleConnection,
        TouchEvent,
        ButtonPushed,
        ButtonLongPressed,
        ButtonLongerPressed,
        ButtonDoubleClicked,
        NewNotification,
        TimerDone,
        BleFirmwareUpdateStarted,
        DimScreen,
        NotifyDeviceActivity,
        RestoreBrightness,
        ShowPairingKey,
        AlarmTriggered,
        Chime,
        BleRadioEnableToggle,
        OnChargingEvent,
      };
    }
  }
}
