# BLE advertising failure analysis (watch stops being discoverable)

**Firmware analysed:** PineTimeSD **v0.7** (`9ed594bb`, `project(pinetime VERSION 230.0.7)`) — the current production release.
**Date:** 2026-10-03
**Related:** [Android_Pebble_SD issue #293](https://github.com/OpenSeizureDetector/Android_Pebble_SD/issues/293), `Android_Pebble_SD/doc/BLE2_Reconnection_Analysis.md`

## 1. Summary

After a long period out of Bluetooth range, the watch sometimes **stops advertising entirely** and cannot be
reconnected by the phone. The only recovery is to toggle Bluetooth off/on **on the watch**.

The root cause is structural: advertising in this firmware is a self-chaining series of 2-second bursts, and
*every* link in that chain is guarded by application-level state flags. If a single guard misfires — or if
`ble_gap_adv_start()` returns an error — the chain stops and **nothing ever restarts it**. There is no
advertising watchdog, and `ble_gap_adv_active()` is never consulted.

Three independent holes can break the chain (§4), and one further finding makes hole #2 completely silent:
**all `ASSERT()` calls in `NimbleController.cpp` are compiled out of the Release build** (§3).

`EnableRadio()` is the only code path that unconditionally clears the stale state *and* force-starts
advertising — which is exactly why toggling Bluetooth on the watch recovers it (§5).

## 2. Version verification

Concern: the working tree is on branch `ppg_improvements` (`fce434be`, version `230.0.8`), so it was necessary
to confirm the analysis applies to production v0.7 and not to post-release upstream sync.

```
$ git diff --name-only v0.7..HEAD | grep -E "src/(components/ble|systemtask|motion|displayapp)"
(no output)
```

The only source changes since v0.7 are:

```
src/components/heartrate/Ppg.cpp   | 156 +++
src/components/heartrate/Ppg.h     |  33 +++
```

plus build/packaging files (`docker/`, `index.html`, `index.json`, `pineTimeSdFw/*`, `.devcontainer/` removal).

**`src/components/ble/`, `src/systemtask/`, `src/components/motion/` and `src/displayapp/` are byte-identical
between v0.7 and HEAD.** Everything below therefore applies to the shipping v0.7 firmware. The
`230.0.8` link map in `build/output/src/` is valid evidence for v0.7 BLE behaviour, because neither the BLE
sources nor the build configuration changed.

## 3. Finding: `ASSERT()` is a no-op in production builds

`StartAdvertising()` ends with:

```cpp
// src/components/ble/NimbleController.cpp:177-178
rc = ble_gap_adv_start(addrType, NULL, 2000, &adv_params, GAPEventCallback, this);
ASSERT(rc == 0);
```

Build configuration:

- `CMakeLists.txt:3` — `set(CMAKE_BUILD_TYPE Release CACHE ...)`, so CMake adds `-DNDEBUG`.
- `src/CMakeLists.txt:825-845` — `-DDEBUG`, `-DDEBUG_NRF_USER` and `-DNRF_LOG_ENABLED=1` are added **only**
  when `CMAKE_BUILD_TYPE STREQUAL "Debug"`.

`ASSERT` comes from the nRF SDK (`nrf_assert.h`, pulled in via `src/CMakeLists.txt:58`), where it is compiled
out unless `DEBUG` is defined. Verified against the linked image:

```
$ grep nrf_assert         build/output/src/pinetime-app-230.0.8.map   →  (no output)
$ grep nrf_assert_handler build/output/src/pinetime-app-230.0.8.map   →  (no output)
```

`nrf_assert.c.o` is not even pulled into the final link. Consequences for the production firmware:

1. **`rc` from `ble_gap_adv_start()` is discarded.** A failed advertising start is silent and never retried.
2. The same applies to `ble_gap_adv_set_fields()`, `ble_gap_adv_rsp_set_fields()` (`:171-175`) and to all the
   `ASSERT`s in `Init()` (`:104-132`).
3. **Release builds emit no logs at all** (`NRF_LOG_ENABLED` is only set under `-DDEBUG`), so a field failure
   leaves no trace on the watch side. Diagnosing which hole fired requires a Debug build.

## 4. The three holes

### How advertising works here

`StartAdvertising()` (`NimbleController.cpp:139-179`) starts a **2000 ms** burst. When it expires, NimBLE
delivers `BLE_GAP_EVENT_ADV_COMPLETE`, whose handler restarts the next burst — a self-sustaining chain:

```cpp
// NimbleController.cpp:183-189
case BLE_GAP_EVENT_ADV_COMPLETE:
  if (bleController.IsRadioEnabled() && !bleController.IsConnected()) {
    StartAdvertising();
  }
  break;
```

If advertising is not running, no `ADV_COMPLETE` can ever be delivered. The chain has exactly one way to be
re-armed: an explicit `StartAdvertising()` call from somewhere else.

Interval profile (`:150-158`): the first 15 bursts use `itvl_min=32 / itvl_max=47` (~20–29 ms, i.e. ~30 s of
fast advertising), after which it settles at `itvl_min=1636 / itvl_max=1651` (~1.02 s) indefinitely.

### Hole A — stale `isConnected` after a NimBLE host reset (best fit for the reported symptom)

- `nimble_on_reset()` (`:55-57`) **only logs**. It does not clear `connectionHandle` or `bleController` state.
- On re-sync, `nimble_on_sync()` (`:59-68`) calls `nptr->StartAdvertising()` but leaves
  `bleController.IsConnected()` **stale-true** (the connection no longer exists).
- 2 s later `ADV_COMPLETE` fires. The guard `!bleController.IsConnected()` is false → `StartAdvertising()` is
  not called.
- No connection exists, so no `BLE_GAP_EVENT_DISCONNECT` will ever fire either.

**Result: the watch advertises for exactly 2 seconds after the host reset, then goes silent forever.**

### Hole B — silent `ble_gap_adv_start()` failure

`ble_gap_adv_start()` (`src/libs/mynewt-nimble/nimble/host/src/ble_gap.c:2395`) can return:

| Return | When |
| --- | --- |
| `BLE_HS_EDISABLED` | `!ble_hs_is_enabled()` — host stopped, or mid-reset before re-sync |
| `BLE_HS_EPREEMPTED` | `ble_gap_is_preempted()` (`ble_gap.c:2427`) — only used for resolving-list access, so unlikely here (public address, no privacy) |
| `BLE_HS_EINVAL` / `EMSGSIZE` | `ble_gap_adv_validate()` / duration conversion — deterministic, would fail every time |
| HCI/controller errors | propagated from the controller |

Because of §3, **any of these is silently ignored and the chain dies with no retry.** The `EDISABLED` case is
the realistic intermittent one: it is exactly the window between a host reset and re-sync.

### Hole C — the `DISCONNECT` restart is conditional

```cpp
// NimbleController.cpp:212-229
case BLE_GAP_EVENT_DISCONNECT:
  if (event->disconnect.conn.sec_state.bonded) PersistBond(event->disconnect.conn);
  currentTimeClient.Reset();
  alertNotificationClient.Reset();
  connectionHandle = BLE_HS_CONN_HANDLE_NONE;
  if (bleController.IsConnected()) {          // <-- guard
    bleController.Disconnect();
    fastAdvCount = 0;
    StartAdvertising();
  }
  break;
```

If `IsConnected()` is already false when the event arrives, advertising is never restarted and `fastAdvCount`
is not reset. The flag can already be false because:

- a previous failed `CONNECT` event ran the `status != 0` branch (`:196-203`), which calls
  `bleController.Disconnect()`;
- `DisableRadio()` (`:410-418`) calls `bleController.Disconnect()` immediately after `ble_gap_terminate()`,
  before the resulting `DISCONNECT` event is processed;
- `EnableRadio()` (`:403-408`) calls `bleController.Disconnect()` unconditionally.

There is also an ordering race in the toggle path: `DisableRadio()` issues `ble_gap_terminate()` and clears the
flag synchronously, so if the user re-enables the radio quickly, `EnableRadio()` → `StartAdvertising()` can run
while the terminate GAP procedure is still in flight (`BLE_HS_EPREEMPTED`), which §3 turns into a silent,
unrecoverable failure.

## 5. Why toggling Bluetooth on the watch is the only recovery

```cpp
// NimbleController.cpp:403-408
void NimbleController::EnableRadio() {
  bleController.EnableRadio();
  bleController.Disconnect();   // clears the stale isConnected flag
  fastAdvCount = 0;             // re-enters fast advertising
  StartAdvertising();           // unconditional
}
```

Reached from `SystemTask.cpp:394-400` (`Messages::BleRadioEnableToggle`). It is the **only** path that both
clears the stale connection state and force-starts advertising regardless of any guard. This matches the
observed field behaviour precisely.

Note also that the hardware watchdog does not help: `watchdog.Setup(7, Run, Pause)` (`SystemTask.cpp:102`) is
reloaded from the `SystemTask` loop (`:419-421`). SystemTask keeps running happily while the BLE host is
stalled or not advertising, so the watchdog never fires. **The watchdog covers SystemTask, not the NimBLE host
task or the advertising chain.**

## 6. Aggravating factors

These do not by themselves cause a permanent failure, but they make reconnection slow and increase the chance
of triggering hole A.

1. **Slow steady-state advertising interval.** After ~30 s the interval is ~1.02 s (`:155-158`), and
   `fastAdvCount` is only reset on disconnect, radio toggle, or `GoToRunning` while disconnected
   (`SystemTask.cpp:221-223`). At marginal RSSI (the Android side measured `-78 dBm` in the field) discovery
   can take a long time.
2. **Small mbuf pool, no flow control.**
   `MYNEWT_VAL_MSYS_1_BLOCK_COUNT = 12`, `MYNEWT_VAL_MSYS_1_BLOCK_SIZE = 292`,
   `MYNEWT_VAL_BLE_HS_FLOW_CTRL = 0`, `MYNEWT_VAL_BLE_MAX_CONNECTIONS = 1`
   (`src/libs/mynewt-nimble/porting/nimble/include/syscfg/syscfg.h:57-62, 376-377, 585-586`).
   During a long dead-link period `MotionService` keeps issuing notifications
   (`MotionService.cpp:104-126`, driven from `SystemTask::UpdateMotion()` at the top of every loop iteration),
   which is exactly the condition that can exhaust mbufs and provoke a host-side error → reset → hole A.
3. **`PersistBond()` runs on the NimBLE host task** inside the `DISCONNECT` handler (`:217-219`, `:420-472`).
   It pushes messages with `portMAX_DELAY` into a 10-deep SystemTask queue (`SystemTask.cpp:87`), calls
   `vTaskDelay(10)`, and performs LittleFS writes on SPI NOR flash — relying on SystemTask having processed
   `GoToRunning` within those 10 ticks (the code itself carries the comment *"This should be fixed in the FS
   driver"*). This can stall the host task before `StartAdvertising()` is reached. It returns early when the
   bond is unchanged, so it is unlikely to be the trigger here, but it is a latent hazard on the critical path.

## 7. What is proven and what is not

**Proven from source and build artefacts:**

- BLE/systemtask/motion code is identical between v0.7 and HEAD (§2).
- `ASSERT` is compiled out of the Release build, so `ble_gap_adv_start()`'s return code is discarded (§3).
- Advertising restart depends entirely on guarded event handlers, with no watchdog and no
  `ble_gap_adv_active()` check (§4).
- `EnableRadio()` is the only unconditional recovery path (§5).

**Not proven (needs a Debug build or instrumentation):**

- Which specific hole fired in the 8100 s field failure — hole A (host reset with stale `isConnected`) is the
  best fit, hole B is the second candidate.
- Whether a NimBLE host reset actually occurred, and what triggered it (mbuf exhaustion is the leading
  hypothesis, not established).
- Production v0.7 emits no `NRF_LOG` output, so there is currently **no way** to observe the watch side in the
  field.

## 8. Recommendations

Ordered by value. Items 1–4 are small, low-risk and together make advertising self-healing regardless of which
hole fires.

### 1. Add an advertising watchdog (highest value)

The `SystemTask` loop already runs at a ≥100 ms cadence (`SystemTask.cpp:188-192`) and already uses this exact
pattern for deferred service discovery (`isBleDiscoveryTimerRunning` / `bleDiscoveryTimer`, `:406-415`). Add a
rate-limited (~1 s) check:

```cpp
// in SystemTask::Work() loop, alongside the existing bleDiscoveryTimer block
if (bleController.IsRadioEnabled() && !bleController.IsConnected() && !nimbleController.IsAdvActive()) {
  nimbleController.StartAdvertising();
}
```

with a thin wrapper in `NimbleController` (`ble_gap_adv_active()` is public API,
`src/libs/mynewt-nimble/nimble/host/include/host/ble_gap.h:1096`):

```cpp
bool IsAdvActive() const { return ble_gap_adv_active() != 0; }
```

This alone closes holes A, B and C, because any broken chain is re-armed within ~1 s.

### 2. Restart advertising unconditionally on disconnect

Replace `NimbleController.cpp:224-228` with:

```cpp
bleController.Disconnect();
fastAdvCount = 0;
if (bleController.IsRadioEnabled()) {
  StartAdvertising();
}
```

### 3. Clear stale connection state on reset and sync

```cpp
void nimble_on_reset(int reason) {
  NRF_LOG_INFO("Nimble lost sync, resetting state; reason=%d", reason);
  if (nptr != nullptr) {
    nptr->ResetConnectionState();   // connectionHandle = BLE_HS_CONN_HANDLE_NONE; bleController.Disconnect();
  }
}

void nimble_on_sync(void) {
  ...
  nptr->ResetConnectionState();
  nptr->fastAdvCount = 0;
  nptr->StartAdvertising();
}
```

### 4. Stop discarding the return code

Replace the compiled-out `ASSERT(rc == 0)` at `:177-178` with real handling:

```cpp
rc = ble_gap_adv_start(addrType, NULL, 2000, &adv_params, GAPEventCallback, this);
if (rc != 0 && rc != BLE_HS_EALREADY) {
  NRF_LOG_ERROR("ble_gap_adv_start failed: rc=%d", rc);   // watchdog will retry
}
```

Do the same for `ble_gap_adv_set_fields()` / `ble_gap_adv_rsp_set_fields()` (`:171-175`).

### 5. Optional / follow-up

- **Use `BLE_HS_FOREVER` advertising** instead of the 2 s burst chain. This removes the `ADV_COMPLETE`
  dependency entirely, which is the largest structural fragility. Trade-off: `RestartFastAdv()`
  (`NimbleController.h:77`) would need an explicit `ble_gap_adv_stop()` + `StartAdvertising()` to change the
  interval.
- **Periodically re-enter fast advertising** while disconnected (e.g. every few minutes reset
  `fastAdvCount = 0`), so the phone finds the watch quickly after a long outage instead of scanning a ~1 s
  interval.
- **Raise `MSYS_1_BLOCK_COUNT`** and/or enable `BLE_HS_FLOW_CTRL` if host resets are confirmed.
- **Move `PersistBond()` off the NimBLE host task** (hand the bond data to SystemTask via a message and do the
  LittleFS work there), removing the `portMAX_DELAY` / SPI-NOR hazard from the disconnect path.

## 9. Verification plan

1. Build a **Debug** firmware (`-DCMAKE_BUILD_TYPE=Debug`) so `NRF_LOG_ENABLED=1`, and capture RTT logs during
   a long out-of-range test. Look for: `Nimble lost sync`, `BLE_GAP_EVENT_ADV_COMPLETE` stopping,
   `BLE_GAP_EVENT_DISCONNECT`, and the new `ble_gap_adv_start failed` message.
2. On the Android side, capture in parallel:
   `adb logcat -v time -s SdDataSourceBLE2 BluetoothCentralManager BluetoothPeripheral`.
   If `discovered` stays flat for the whole failure window, the watch was not advertising (holes A/B/C
   confirmed). If `discovered` increments while `connFailed` increments, the problem is connection
   establishment instead.
3. Apply recommendations 1–4, repeat the overnight/long-range test, and confirm the failure no longer requires
   a watch-side Bluetooth toggle. The watchdog should also make any residual failure self-recover within ~1 s,
   which is itself a useful signal (a visible burst of watchdog restarts in the log points at hole B).

## 10. Reference index

| Item | Location |
| --- | --- |
| `nimble_on_reset` (logs only) | `src/components/ble/NimbleController.cpp:55-57` |
| `nimble_on_sync` → `StartAdvertising()` | `src/components/ble/NimbleController.cpp:59-68` |
| `Init()` | `src/components/ble/NimbleController.cpp:75-137` |
| `StartAdvertising()` (2 s burst, intervals, `fastAdvCount`) | `src/components/ble/NimbleController.cpp:139-179` |
| Discarded `ble_gap_adv_start()` rc + `ASSERT` | `src/components/ble/NimbleController.cpp:177-178` |
| `ADV_COMPLETE` guard (hole A) | `src/components/ble/NimbleController.cpp:183-189` |
| `CONNECT` incl. failed-connect branch | `src/components/ble/NimbleController.cpp:191-210` |
| `DISCONNECT` guard (hole C) | `src/components/ble/NimbleController.cpp:212-229` |
| `SUBSCRIBE` / unsubscribe handling | `src/components/ble/NimbleController.cpp:312-332` |
| `EnableRadio()` (the only recovery path) | `src/components/ble/NimbleController.cpp:403-408` |
| `DisableRadio()` (terminate race) | `src/components/ble/NimbleController.cpp:410-418` |
| `PersistBond()` on host task | `src/components/ble/NimbleController.cpp:420-472` |
| `RestartFastAdv()` | `src/components/ble/NimbleController.h:77` |
| SystemTask queue (depth 10) | `src/systemtask/SystemTask.cpp:87` |
| Watchdog setup / reload | `src/systemtask/SystemTask.cpp:102`, `:419-421` |
| SystemTask loop cadence | `src/systemtask/SystemTask.cpp:188-192` |
| `GoToRunning` → `RestartFastAdv()` | `src/systemtask/SystemTask.cpp:221-223` |
| `BleRadioEnableToggle` handler | `src/systemtask/SystemTask.cpp:394-400` |
| Existing deferred-timer precedent | `src/systemtask/SystemTask.cpp:406-415` |
| `ble_gap_adv_start()` error paths | `src/libs/mynewt-nimble/nimble/host/src/ble_gap.c:2395+` |
| `ble_gap_adv_active()` public API | `src/libs/mynewt-nimble/nimble/host/include/host/ble_gap.h:1096` |
| NimBLE resource config | `src/libs/mynewt-nimble/porting/nimble/include/syscfg/syscfg.h:57-62, 376-377, 585-586` |
| Release build default | `CMakeLists.txt:3` |
| Debug-only defines (`-DDEBUG`, `NRF_LOG_ENABLED`) | `src/CMakeLists.txt:825-845` |
| MotionService notify path | `src/components/ble/MotionService.cpp:104-126` |
