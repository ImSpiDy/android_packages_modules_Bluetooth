/******************************************************************************
 *
 *  Copyright 2009-2016 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "btif_av"

#include "btif/include/btif_av.h"

#include <android_bluetooth_sysprop.h>
#include <base/functional/bind.h>
#include <base/logging.h>
#include <base/strings/stringprintf.h>
#include <frameworks/proto_logging/stats/enums/bluetooth/a2dp/enums.pb.h>
#include <frameworks/proto_logging/stats/enums/bluetooth/enums.pb.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

#include "audio_hal_interface/a2dp_encoding.h"
#include "bta/include/bta_av_api.h"
#include "btif/include/btif_a2dp.h"
#include "btif/include/btif_a2dp_control.h"
#include "btif/include/btif_a2dp_sink.h"
#include "btif/include/btif_a2dp_source.h"
#include "btif/include/btif_av_co.h"
#include "btif/include/btif_common.h"
#include "btif/include/btif_metrics_logging.h"
#include "btif/include/btif_profile_queue.h"
#include "btif/include/btif_rc.h"
#include "btif/include/btif_util.h"
#include "btif/include/stack_manager_t.h"
#include "btif_metrics_logging.h"
#include "common/state_machine.h"
#include "device/include/device_iot_config.h"
#include "hardware/bt_av.h"
#include "include/check.h"
#include "include/hardware/bt_rc.h"
#include "internal_include/bt_trace.h"
#include "os/system_properties.h"
#include "osi/include/alarm.h"
#include "osi/include/allocator.h"
#include "stack/include/avrc_api.h"
#include "stack/include/bt_hdr.h"
#include "stack/include/bt_uuid16.h"
#include "stack/include/btm_ble_api.h"
#include "stack/include/btm_log_history.h"
#include "stack/include/main_thread.h"
#include "types/raw_address.h"

/*****************************************************************************
 *  Constants & Macros
 *****************************************************************************/
static const std::string kBtifAvSourceServiceName = "Advanced Audio Source";
static const std::string kBtifAvSinkServiceName = "Advanced Audio Sink";
static constexpr int kDefaultMaxConnectedAudioDevices = 1;
static constexpr tBTA_AV_HNDL kBtaHandleUnknown = 0;

namespace {
constexpr char kBtmLogHistoryTag[] = "A2DP";
}

/*****************************************************************************
 *  Local type definitions
 *****************************************************************************/

typedef struct {
  int sample_rate;
  int channel_count;
  RawAddress peer_address;
} btif_av_sink_config_req_t;

typedef struct {
  bool use_latency_mode;
} btif_av_start_stream_req_t;

typedef struct {
  bool is_low_latency;
} btif_av_set_latency_req_t;

typedef struct {
  int enc_mode;
} btif_av_codec_mode_change_t;

/**
 * BTIF AV events
 */
typedef enum {
  /* Reuse BTA_AV_XXX_EVT - No need to redefine them here */
  BTIF_AV_CONNECT_REQ_EVT = BTA_AV_MAX_EVT,
  BTIF_AV_DISCONNECT_REQ_EVT,
  BTIF_AV_START_STREAM_REQ_EVT,
  BTIF_AV_STOP_STREAM_REQ_EVT,
  BTIF_AV_SUSPEND_STREAM_REQ_EVT,
  BTIF_AV_SINK_CONFIG_REQ_EVT,
  BTIF_AV_ACL_DISCONNECTED,
  BTIF_AV_OFFLOAD_START_REQ_EVT,
  BTIF_AV_AVRCP_OPEN_EVT,
  BTIF_AV_AVRCP_CLOSE_EVT,
  BTIF_AV_AVRCP_REMOTE_PLAY_EVT,
  BTIF_AV_SET_LATENCY_REQ_EVT,
  BTIF_AV_SET_CODEC_MODE_EVT
} btif_av_sm_event_t;

class BtifAvEvent {
 public:
  BtifAvEvent(uint32_t event, const void* p_data, size_t data_length);
  BtifAvEvent(const BtifAvEvent& other);
  BtifAvEvent() = delete;
  ~BtifAvEvent();
  BtifAvEvent& operator=(const BtifAvEvent& other);

  uint32_t Event() const { return event_; }
  void* Data() const { return data_; }
  size_t DataLength() const { return data_length_; }
  std::string ToString() const;
  static std::string EventName(uint32_t event);

 private:
  void DeepCopy(uint32_t event, const void* p_data, size_t data_length);
  void DeepFree();

  uint32_t event_;
  void* data_;
  size_t data_length_;
};

class BtifAvPeer;
static bt_status_t sink_set_active_device(const RawAddress& peer_address);

static void btif_av_sink_delete_active_peer(void);
static void btif_av_source_delete_active_peer(void);

// Should not need dedicated Suspend state as actual actions are no
// different than Open state. Suspend flags are needed however to prevent
// media task from trying to restart stream during remote Suspend or while
// we are in the process of a local Suspend.
class BtifAvStateMachine : public bluetooth::common::StateMachine {
 public:
  enum {
    kStateIdle,     // AVDTP disconnected
    kStateOpening,  // Opening AVDTP connection
    kStateOpened,   // AVDTP is in OPEN state
    kStateStarted,  // A2DP stream started
    kStateClosing,  // Closing AVDTP connection
  };

  class StateIdle : public State {
   public:
    StateIdle(BtifAvStateMachine& sm)
        : State(sm, kStateIdle), peer_(sm.Peer()) {}
    void OnEnter() override;
    void OnExit() override;
    bool ProcessEvent(uint32_t event, void* p_data) override;

   private:
    BtifAvPeer& peer_;
  };

  class StateOpening : public State {
   public:
    StateOpening(BtifAvStateMachine& sm)
        : State(sm, kStateOpening), peer_(sm.Peer()) {}
    void OnEnter() override;
    void OnExit() override;
    bool ProcessEvent(uint32_t event, void* p_data) override;

   private:
    BtifAvPeer& peer_;
  };

  class StateOpened : public State {
   public:
    StateOpened(BtifAvStateMachine& sm)
        : State(sm, kStateOpened), peer_(sm.Peer()) {}
    void OnEnter() override;
    void OnExit() override;
    bool ProcessEvent(uint32_t event, void* p_data) override;

   private:
    BtifAvPeer& peer_;
  };

  class StateStarted : public State {
   public:
    StateStarted(BtifAvStateMachine& sm)
        : State(sm, kStateStarted), peer_(sm.Peer()) {}
    void OnEnter() override;
    void OnExit() override;
    bool ProcessEvent(uint32_t event, void* p_data) override;

   private:
    BtifAvPeer& peer_;
  };

  class StateClosing : public State {
   public:
    StateClosing(BtifAvStateMachine& sm)
        : State(sm, kStateClosing), peer_(sm.Peer()) {}
    void OnEnter() override;
    void OnExit() override;
    bool ProcessEvent(uint32_t event, void* p_data) override;

   private:
    BtifAvPeer& peer_;
  };

  BtifAvStateMachine(BtifAvPeer& btif_av_peer) : peer_(btif_av_peer) {
    state_idle_ = new StateIdle(*this);
    state_opening_ = new StateOpening(*this);
    state_opened_ = new StateOpened(*this);
    state_started_ = new StateStarted(*this);
    state_closing_ = new StateClosing(*this);

    AddState(state_idle_);
    AddState(state_opening_);
    AddState(state_opened_);
    AddState(state_started_);
    AddState(state_closing_);
    SetInitialState(state_idle_);
  }

  BtifAvPeer& Peer() { return peer_; }

 private:
  BtifAvPeer& peer_;
  StateIdle* state_idle_;
  StateOpening* state_opening_;
  StateOpened* state_opened_;
  StateStarted* state_started_;
  StateClosing* state_closing_;
};

class BtifAvPeer {
 public:
  enum {
    kFlagLocalSuspendPending = 0x1,
    kFlagRemoteSuspend = 0x2,
    kFlagPendingStart = 0x4,
    kFlagPendingStop = 0x8,
  };
  static constexpr uint64_t kTimeoutAvOpenOnRcMs = 2 * 1000;  // 2s

  BtifAvPeer(const RawAddress& peer_address, uint8_t peer_sep,
             tBTA_AV_HNDL bta_handle, uint8_t peer_id);
  ~BtifAvPeer();

  bt_status_t Init();
  void Cleanup();

  /**
   * Check whether the peer can be deleted.
   *
   * @return true if the pair can be deleted, otherwise false
   */
  bool CanBeDeleted() const;

  /**
   * Check whether the peer is the active one.
   *
   * @return true if this peer is the active one
   */
  bool IsActivePeer() const { return (PeerAddress() == ActivePeerAddress()); }

  /**
   * Get the address of the active peer.
   *
   * @return the address of the active peer
   */
  const RawAddress& ActivePeerAddress() const;

  const RawAddress& PeerAddress() const { return peer_address_; }
  bool IsSource() const { return (peer_sep_ == AVDT_TSEP_SRC); }
  bool IsSink() const { return (peer_sep_ == AVDT_TSEP_SNK); }
  uint8_t PeerSep() const { return peer_sep_; }
  void SetSep(uint8_t sep_type) { peer_sep_ = sep_type; }
  /**
   * Get the local device's Service Class UUID
   *
   * @return the local device's Service Class UUID: UUID_SERVCLASS_AUDIO_SOURCE
   * or UUID_SERVCLASS_AUDIO_SINK
   */
  uint16_t LocalUuidServiceClass() const {
    return (IsSink() ? UUID_SERVCLASS_AUDIO_SOURCE : UUID_SERVCLASS_AUDIO_SINK);
  }
  tBTA_AV_HNDL BtaHandle() const { return bta_handle_; }
  void SetBtaHandle(tBTA_AV_HNDL bta_handle) { bta_handle_ = bta_handle; }
  uint8_t PeerId() const { return peer_id_; }

  BtifAvStateMachine& StateMachine() { return state_machine_; }
  const BtifAvStateMachine& StateMachine() const { return state_machine_; }
  alarm_t* AvOpenOnRcTimer() { return av_open_on_rc_timer_; }
  const alarm_t* AvOpenOnRcTimer() const { return av_open_on_rc_timer_; }

  void SetEdr(tBTA_AV_EDR edr) { edr_ = edr; }
  bool IsEdr() const { return (edr_ != 0); }
  bool Is3Mbps() const { return ((edr_ & BTA_AV_EDR_3MBPS) != 0); }

  bool IsConnected() const;
  bool IsStreaming() const;
  bool IsInSilenceMode() const { return is_silenced_; }

  void SetSilence(bool silence) { is_silenced_ = silence; }

  // AVDTP delay reporting in 1/10 milliseconds
  void SetDelayReport(uint16_t delay) { delay_report_ = delay; }
  uint16_t GetDelayReport() const { return delay_report_; }

  void SetMandatoryCodecPreferred(bool preferred) {
    mandatory_codec_preferred_ = preferred;
  }
  bool IsMandatoryCodecPreferred() const { return mandatory_codec_preferred_; }

  /**
   * Check whether any of the flags specified by the bitlags mask is set.
   *
   * @param bitflags_mask the bitflags to check
   * @return true if any of the flags to check is set, otherwise false.
   */
  bool CheckFlags(uint8_t bitflags_mask) const {
    return ((flags_ & bitflags_mask) != 0);
  }

  /**
   * Set only the flags as specified by the bitflags mask.
   *
   * @param bitflags_mask the bitflags to set
   */
  void SetFlags(uint8_t bitflags_mask) { flags_ |= bitflags_mask; }

  /**
   * Clear only the flags as specified by the bitflags mask.
   *
   * @param bitflags_mask the bitflags to clear
   */
  void ClearFlags(uint8_t bitflags_mask) { flags_ &= ~bitflags_mask; }

  /**
   * Clear all flags.
   */
  void ClearAllFlags() { flags_ = 0; }

  /**
   * Get a string representation of the flags that are set.
   */
  std::string FlagsToString() const;

  bool SelfInitiatedConnection() const { return self_initiated_connection_; }
  void SetSelfInitiatedConnection(bool v) { self_initiated_connection_ = v; }

  bool UseLatencyMode() const { return use_latency_mode_; }
  void SetUseLatencyMode(bool use_latency_mode) {
    use_latency_mode_ = use_latency_mode;
  }

 private:
  const RawAddress peer_address_;
  uint8_t peer_sep_;  // SEP type of peer device
  tBTA_AV_HNDL bta_handle_;
  const uint8_t peer_id_;
  BtifAvStateMachine state_machine_;
  alarm_t* av_open_on_rc_timer_;
  tBTA_AV_EDR edr_;
  uint8_t flags_;
  bool self_initiated_connection_;
  bool is_silenced_;
  uint16_t delay_report_;
  bool mandatory_codec_preferred_ = false;
  bool use_latency_mode_ = false;
};

class BtifAvSource {
 public:
  // The PeerId is used as AppId for BTA_AvRegister() purpose
  static constexpr uint8_t kPeerIdMin = 0;
  static constexpr uint8_t kPeerIdMax = BTA_AV_NUM_STRS;

  BtifAvSource()
      : callbacks_(nullptr),
        enabled_(false),
        a2dp_offload_enabled_(false),
        invalid_peer_check_(false),
        max_connected_peers_(kDefaultMaxConnectedAudioDevices) {}
  ~BtifAvSource();

  bt_status_t Init(
      btav_source_callbacks_t* callbacks, int max_connected_audio_devices,
      const std::vector<btav_a2dp_codec_config_t>& codec_priorities,
      const std::vector<btav_a2dp_codec_config_t>& offloading_preference,
      std::vector<btav_a2dp_codec_info_t>* supported_codecs);
  void Cleanup();

  btav_source_callbacks_t* Callbacks() { return callbacks_; }
  bool Enabled() const { return enabled_; }
  bool A2dpOffloadEnabled() const { return a2dp_offload_enabled_; }
  void SetInvalidPeerCheck(bool invalid_peer_check) {
    invalid_peer_check_ = invalid_peer_check;
  }
  BtifAvPeer* FindPeer(const RawAddress& peer_address);
  BtifAvPeer* FindPeerByHandle(tBTA_AV_HNDL bta_handle);
  BtifAvPeer* FindPeerByPeerId(uint8_t peer_id);
  BtifAvPeer* FindOrCreatePeer(const RawAddress& peer_address,
                               tBTA_AV_HNDL bta_handle);

  /**
   * Check whether a connection to a peer is allowed.
   * The check considers the maximum number of connected peers.
   *
   * @param peer_address the peer address to connect to
   * @return true if connection is allowed, otherwise false
   */
  bool AllowedToConnect(const RawAddress& peer_address) const;

  /**
   * Delete a peer.
   *
   * @param peer_address the peer to delete
   * @return true on success, otherwise false
   */
  bool DeletePeer(const RawAddress& peer_address);

  /**
   * Delete all peers that have transitioned to Idle state and can be deleted.
   * If a peer was just created/initialized, then it cannot be deleted yet.
   */
  void DeleteIdlePeers();

  /**
   * Get the active peer.
   *
   * @return the active peer
   */
  const RawAddress& ActivePeer() const { return active_peer_; }

  /**
   * Check whether peer is silenced
   *
   * @param peer_address the peer to check
   * @return true on silence mode enabled, otherwise false
   */
  bool IsPeerSilenced(const RawAddress& peer_address) {
    if (peer_address.IsEmpty()) {
      return false;
    }
    BtifAvPeer* peer = FindPeer(peer_address);
    if (peer == nullptr) {
      LOG_WARN("%s: peer is null", __func__);
      return false;
    }
    if (!peer->IsConnected()) {
      LOG_WARN("%s: peer is not connected", __func__);
      return false;
    }
    return peer->IsInSilenceMode();
  }

  /**
   * Set peer silence mode
   *
   * @param peer_address the peer to set
   * @param silence true on enable silence mode, false on disable
   * @return true on success, otherwise false
   */
  bool SetSilencePeer(const RawAddress& peer_address, const bool silence) {
    if (peer_address.IsEmpty()) {
      return false;
    }
    LOG_INFO("%s: peer: %s", __PRETTY_FUNCTION__,
             ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    BtifAvPeer* peer = FindPeer(peer_address);
    if (peer == nullptr) {
      LOG_WARN("%s: peer is null", __func__);
      return false;
    }
    if (!peer->IsConnected()) {
      LOG_WARN("%s: peer is not connected", __func__);
      return false;
    }
    peer->SetSilence(silence);
    return true;
  }

  /**
   * Set the active peer.
   *
   * @param peer_address the active peer address or RawAddress::kEmpty to
   * reset the active peer
   * @return true on success, otherwise false
   */
  bool SetActivePeer(const RawAddress& peer_address,
                     std::promise<void> peer_ready_promise) {
    LOG(INFO) << __PRETTY_FUNCTION__ << ": peer: "
              << ADDRESS_TO_LOGGABLE_STR(peer_address);

    if (active_peer_ == peer_address) {
      peer_ready_promise.set_value();
      return true;  // Nothing has changed
    }
    if (peer_address.IsEmpty()) {
      LOG_VERBOSE("%s: peer address is empty, shutdown the Audio source",
                  __func__);
      if (!btif_av_src_sink_coexist_enabled() ||
          (btif_av_src_sink_coexist_enabled() &&
           btif_av_sink_active_peer().IsEmpty())) {
        if (!bta_av_co_set_active_peer(peer_address)) {
          LOG(WARNING) << __func__
                       << ": unable to set active peer to empty in BtaAvCo";
        }
      }

      btif_a2dp_source_end_session(active_peer_);
      std::promise<void> shutdown_complete_promise;
      std::future<void> shutdown_complete_future =
          shutdown_complete_promise.get_future();
      btif_a2dp_source_shutdown(std::move(shutdown_complete_promise));
      using namespace std::chrono_literals;
      if (shutdown_complete_future.wait_for(1s) ==
          std::future_status::timeout) {
        LOG_ERROR("Timed out waiting for A2DP source shutdown to complete.");
      }
      active_peer_ = peer_address;
      peer_ready_promise.set_value();
      return true;
    }

    if (btif_av_src_sink_coexist_enabled()) btif_av_sink_delete_active_peer();
    BtifAvPeer* peer = FindPeer(peer_address);
    if (peer == nullptr || !peer->IsConnected()) {
      LOG(ERROR) << __func__ << ": Error setting "
                 << ADDRESS_TO_LOGGABLE_STR(peer_address)
                 << " as active Source peer";
      peer_ready_promise.set_value();
      return false;
    }

    if (!btif_a2dp_source_restart_session(active_peer_, peer_address,
                                          std::move(peer_ready_promise))) {
      // cannot set promise but need to be handled within restart_session
      return false;
    }
    active_peer_ = peer_address;
    return true;
  }

  void DeleteActivePeer(void) {
    std::promise<void> shutdown_complete_promise;
    LOG_VERBOSE("%s", __func__);
    if (btif_av_sink_active_peer().IsEmpty()) {
      if (!bta_av_co_set_active_peer(RawAddress::kEmpty)) {
        LOG_WARN("%s : unable to set active peer to empty in BtaAvCo",
                 __func__);
      }
    } else {
      LOG_WARN("%s : there is an active peer as source role", __func__);
    }
    btif_a2dp_source_end_session(active_peer_);
    btif_a2dp_source_shutdown(std::move(shutdown_complete_promise));
    active_peer_ = RawAddress::kEmpty;
  }

  /**
   * Update source codec configuration for a peer.
   *
   * @param peer_address the address of the peer to update
   * @param codec_preferences the updated codec preferences
   */
  void UpdateCodecConfig(
      const RawAddress& peer_address,
      const std::vector<btav_a2dp_codec_config_t>& codec_preferences,
      std::promise<void> peer_ready_promise) {
    // Restart the session if the codec for the active peer is updated
    if (!peer_address.IsEmpty() && active_peer_ == peer_address) {
      btif_a2dp_source_end_session(active_peer_);
    }

    btif_a2dp_source_encoder_user_config_update_req(
        peer_address, codec_preferences, std::move(peer_ready_promise));
  }

  const std::map<RawAddress, BtifAvPeer*>& Peers() const { return peers_; }

  void RegisterAllBtaHandles();
  void DeregisterAllBtaHandles();
  void BtaHandleRegistered(uint8_t peer_id, tBTA_AV_HNDL bta_handle);
  BtifAvPeer* popPeer(const RawAddress& peer_address);
  void AddPeer(BtifAvPeer* peer);

 private:
  void CleanupAllPeers();

  btav_source_callbacks_t* callbacks_;
  bool enabled_;
  bool a2dp_offload_enabled_;
  bool invalid_peer_check_;  // pending to check at BTA_AV_OPEN_EVT
  int max_connected_peers_;
  std::map<RawAddress, BtifAvPeer*> peers_;
  std::set<RawAddress> silenced_peers_;
  RawAddress active_peer_;
  std::map<uint8_t, tBTA_AV_HNDL> peer_id2bta_handle_;
  std::mutex mutex_;
  // protect for BtifAvSource::peers_
  std::recursive_mutex btifavsource_peer_lock_;
};

class BtifAvSink {
 public:
  // The PeerId is used as AppId for BTA_AvRegister() purpose
  static constexpr uint8_t kPeerIdMin = 0;
  static constexpr uint8_t kPeerIdMax = BTA_AV_NUM_STRS;

  BtifAvSink()
      : callbacks_(nullptr),
        enabled_(false),
        invalid_peer_check_(false),
        max_connected_peers_(kDefaultMaxConnectedAudioDevices) {}
  ~BtifAvSink();

  bt_status_t Init(btav_sink_callbacks_t* callbacks,
                   int max_connected_audio_devices);
  void Cleanup();

  btav_sink_callbacks_t* Callbacks() { return callbacks_; }
  bool Enabled() const { return enabled_; }

  void SetInvalidPeerCheck(bool invalid_peer_check) {
    invalid_peer_check_ = invalid_peer_check;
  }
  BtifAvPeer* FindPeer(const RawAddress& peer_address);
  BtifAvPeer* FindPeerByHandle(tBTA_AV_HNDL bta_handle);
  BtifAvPeer* FindPeerByPeerId(uint8_t peer_id);
  BtifAvPeer* FindOrCreatePeer(const RawAddress& peer_address,
                               tBTA_AV_HNDL bta_handle);

  /**
   * Check whether a connection to a peer is allowed.
   * The check considers the maximum number of connected peers.
   *
   * @param peer_address the peer address to connect to
   * @return true if connection is allowed, otherwise false
   */
  bool AllowedToConnect(const RawAddress& peer_address) const;

  /**
   * Delete a peer.
   *
   * @param peer_address the peer to delete
   * @return true on success, otherwise false
   */
  bool DeletePeer(const RawAddress& peer_address);

  /**
   * Delete all peers that have transitioned to Idle state and can be deleted.
   * If a peer was just created/initialized, then it cannot be deleted yet.
   */
  void DeleteIdlePeers();

  /**
   * Get the active peer.
   *
   * @return the active peer
   */
  const RawAddress& ActivePeer() const { return active_peer_; }

  /**
   * Set the active peer.
   *
   * @param peer_address the active peer address or RawAddress::kEmpty to
   * reset the active peer
   * @return true on success, otherwise false
   */
  bool SetActivePeer(const RawAddress& peer_address,
                     std::promise<void> peer_ready_promise) {
    LOG(INFO) << __PRETTY_FUNCTION__ << ": peer: "
              << ADDRESS_TO_LOGGABLE_STR(peer_address);

    if (active_peer_ == peer_address) {
      peer_ready_promise.set_value();
      return true;  // Nothing has changed
    }
    if (peer_address.IsEmpty()) {
      LOG_VERBOSE("%s: peer address is empty, shutdown the Audio sink",
                  __func__);
      if (!btif_av_src_sink_coexist_enabled() ||
          (btif_av_src_sink_coexist_enabled() &&
           btif_av_source_active_peer().IsEmpty())) {
        if (!bta_av_co_set_active_peer(peer_address)) {
          LOG(WARNING) << __func__
                       << ": unable to set active peer to empty in BtaAvCo";
        }
      }

      btif_a2dp_sink_end_session(active_peer_);
      btif_a2dp_sink_shutdown();
      active_peer_ = peer_address;
      peer_ready_promise.set_value();
      return true;
    }

    if (btif_av_src_sink_coexist_enabled()) {
      btif_av_source_delete_active_peer();
    }
    BtifAvPeer* peer = FindPeer(peer_address);
    if (peer == nullptr || !peer->IsConnected()) {
      LOG(ERROR) << __func__ << ": Error setting "
                 << ADDRESS_TO_LOGGABLE_STR(peer_address)
                 << " as active Sink peer";
      peer_ready_promise.set_value();
      return false;
    }

    if (!btif_a2dp_sink_restart_session(active_peer_, peer_address,
                                        std::move(peer_ready_promise))) {
      // cannot set promise but need to be handled within restart_session
      return false;
    }
    LOG(INFO) << "Setting the active peer to peer address %s"
              << ADDRESS_TO_LOGGABLE_STR(peer_address);
    active_peer_ = peer_address;
    return true;
  }

  void DeleteActivePeer(void) {
    LOG_VERBOSE("%s", __func__);
    if (btif_av_source_active_peer().IsEmpty()) {
      if (!bta_av_co_set_active_peer(RawAddress::kEmpty)) {
        LOG(WARNING) << __func__
                     << ": unable to set active peer to empty in BtaAvCo";
      }
    } else {
      LOG(WARNING) << __func__ << ": there is an active peer as sink role";
    }
    btif_a2dp_sink_end_session(active_peer_);
    btif_a2dp_sink_shutdown();
    active_peer_ = RawAddress::kEmpty;
  }

  const std::map<RawAddress, BtifAvPeer*>& Peers() const { return peers_; }

  void RegisterAllBtaHandles();
  void DeregisterAllBtaHandles();
  void BtaHandleRegistered(uint8_t peer_id, tBTA_AV_HNDL bta_handle);
  BtifAvPeer* popPeer(const RawAddress& peer_address);
  void AddPeer(BtifAvPeer* peer);

 private:
  void CleanupAllPeers();

  btav_sink_callbacks_t* callbacks_;
  bool enabled_;
  bool invalid_peer_check_;  // pending to check at BTA_AV_OPEN_EVT
  int max_connected_peers_;
  std::map<RawAddress, BtifAvPeer*> peers_;
  RawAddress active_peer_;
  std::map<uint8_t, tBTA_AV_HNDL> peer_id2bta_handle_;
};

/*****************************************************************************
 *  Static variables
 *****************************************************************************/
static BtifAvSource btif_av_source;
static BtifAvSink btif_av_sink;

/* Helper macro to avoid code duplication in the state machine handlers */
#define CHECK_RC_EVENT(e, d)       \
  case BTA_AV_RC_OPEN_EVT:         \
  case BTA_AV_RC_BROWSE_OPEN_EVT:  \
  case BTA_AV_RC_CLOSE_EVT:        \
  case BTA_AV_RC_BROWSE_CLOSE_EVT: \
  case BTA_AV_REMOTE_CMD_EVT:      \
  case BTA_AV_VENDOR_CMD_EVT:      \
  case BTA_AV_META_MSG_EVT:        \
  case BTA_AV_RC_FEAT_EVT:         \
  case BTA_AV_RC_PSM_EVT:          \
  case BTA_AV_REMOTE_RSP_EVT: {    \
    btif_rc_handler(e, d);         \
  } break;

static bt_status_t src_disconnect_sink(const RawAddress& peer_address);
static bt_status_t sink_disconnect_src(const RawAddress& peer_address);
static void btif_av_source_dispatch_sm_event(const RawAddress& peer_address,
                                             btif_av_sm_event_t event);
static void btif_av_sink_dispatch_sm_event(const RawAddress& peer_address,
                                           btif_av_sm_event_t event);
static void btif_av_handle_event(uint8_t peer_sep,
                                 const RawAddress& peer_address,
                                 tBTA_AV_HNDL bta_handle,
                                 const BtifAvEvent& btif_av_event);
static void btif_report_connection_state(const RawAddress& peer_address,
                                         btav_connection_state_t state,
                                         const bt_status_t status,
                                         uint8_t error_code);
static void btif_report_audio_state(const RawAddress& peer_address,
                                    btav_audio_state_t state);
static void btif_av_report_sink_audio_config_state(
    const RawAddress& peer_address, int sample_rate, int channel_count);
static void btif_av_query_mandatory_codec_priority(
    const RawAddress& peer_address);
static void btif_av_source_initiate_av_open_timer_timeout(void* data);
static void btif_av_sink_initiate_av_open_timer_timeout(void* data);
static void bta_av_sink_media_callback(const RawAddress& peer_address,
                                       tBTA_AV_EVT event,
                                       tBTA_AV_MEDIA* p_data);
extern bool btif_av_both_enable(void);
extern bool btif_av_is_connected_addr(const RawAddress& peer_address);
extern bool btif_av_peer_is_sink(const RawAddress& peer_address);
extern void btif_rc_check_pending_cmd(const RawAddress& peer_address);
extern void btif_rc_get_addr_by_handle(uint8_t handle, RawAddress& rc_addr);

static BtifAvPeer* btif_av_source_find_peer(const RawAddress& peer_address) {
  return btif_av_source.FindPeer(peer_address);
}
static BtifAvPeer* btif_av_sink_find_peer(const RawAddress& peer_address) {
  return btif_av_sink.FindPeer(peer_address);
}
static BtifAvPeer* btif_av_find_peer(const RawAddress& peer_address) {
  if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
    BtifAvPeer* peer = nullptr;
    peer = btif_av_source_find_peer(peer_address);
    if (nullptr == peer) {
      return btif_av_sink_find_peer(peer_address);
    }
    return peer;
  }
  if (btif_av_source.Enabled()) return btif_av_source_find_peer(peer_address);
  if (btif_av_sink.Enabled()) return btif_av_sink_find_peer(peer_address);
  return nullptr;
}
static BtifAvPeer* btif_av_find_active_peer() {
  if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
    BtifAvPeer* peer = nullptr;
    peer = btif_av_source_find_peer(btif_av_source.ActivePeer());
    if (nullptr == peer) {
      return btif_av_sink_find_peer(btif_av_sink.ActivePeer());
    }
    return peer;
  }
  if (btif_av_source.Enabled())
    return btif_av_source_find_peer(btif_av_source.ActivePeer());
  if (btif_av_sink.Enabled())
    return btif_av_sink_find_peer(btif_av_sink.ActivePeer());
  return nullptr;
}

const RawAddress& btif_av_find_by_handle(tBTA_AV_HNDL bta_handle) {
  BtifAvPeer* peer = nullptr;
  if (btif_av_both_enable()) {
    peer = btif_av_source.FindPeerByHandle(bta_handle);
    if (peer == nullptr) {
      peer = btif_av_sink.FindPeerByHandle(bta_handle);
    }
    if (peer == nullptr) return RawAddress::kEmpty;

    return peer->PeerAddress();
  }
  if (btif_av_source.Enabled())
    peer = btif_av_source.FindPeerByHandle(bta_handle);
  if (btif_av_sink.Enabled()) peer = btif_av_sink.FindPeerByHandle(bta_handle);

  if (peer == nullptr) return RawAddress::kEmpty;

  return peer->PeerAddress();
}

/*****************************************************************************
 * Local helper functions
 *****************************************************************************/

const char* dump_av_sm_event_name(btif_av_sm_event_t event) {
  switch ((int)event) {
    CASE_RETURN_STR(BTA_AV_ENABLE_EVT)
    CASE_RETURN_STR(BTA_AV_REGISTER_EVT)
    CASE_RETURN_STR(BTA_AV_OPEN_EVT)
    CASE_RETURN_STR(BTA_AV_CLOSE_EVT)
    CASE_RETURN_STR(BTA_AV_START_EVT)
    CASE_RETURN_STR(BTA_AV_STOP_EVT)
    CASE_RETURN_STR(BTA_AV_PROTECT_REQ_EVT)
    CASE_RETURN_STR(BTA_AV_PROTECT_RSP_EVT)
    CASE_RETURN_STR(BTA_AV_RC_OPEN_EVT)
    CASE_RETURN_STR(BTA_AV_RC_CLOSE_EVT)
    CASE_RETURN_STR(BTA_AV_RC_BROWSE_OPEN_EVT)
    CASE_RETURN_STR(BTA_AV_RC_BROWSE_CLOSE_EVT)
    CASE_RETURN_STR(BTA_AV_REMOTE_CMD_EVT)
    CASE_RETURN_STR(BTA_AV_REMOTE_RSP_EVT)
    CASE_RETURN_STR(BTA_AV_VENDOR_CMD_EVT)
    CASE_RETURN_STR(BTA_AV_VENDOR_RSP_EVT)
    CASE_RETURN_STR(BTA_AV_RECONFIG_EVT)
    CASE_RETURN_STR(BTA_AV_SUSPEND_EVT)
    CASE_RETURN_STR(BTA_AV_PENDING_EVT)
    CASE_RETURN_STR(BTA_AV_META_MSG_EVT)
    CASE_RETURN_STR(BTA_AV_REJECT_EVT)
    CASE_RETURN_STR(BTA_AV_RC_FEAT_EVT)
    CASE_RETURN_STR(BTA_AV_RC_PSM_EVT)
    CASE_RETURN_STR(BTA_AV_OFFLOAD_START_RSP_EVT)
    CASE_RETURN_STR(BTIF_AV_CONNECT_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_DISCONNECT_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_START_STREAM_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_STOP_STREAM_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_SUSPEND_STREAM_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_SINK_CONFIG_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_ACL_DISCONNECTED)
    CASE_RETURN_STR(BTIF_AV_OFFLOAD_START_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_AVRCP_OPEN_EVT)
    CASE_RETURN_STR(BTIF_AV_AVRCP_CLOSE_EVT)
    CASE_RETURN_STR(BTIF_AV_AVRCP_REMOTE_PLAY_EVT)
    CASE_RETURN_STR(BTIF_AV_SET_LATENCY_REQ_EVT)
    CASE_RETURN_STR(BTIF_AV_SET_CODEC_MODE_EVT)
    default:
      return "UNKNOWN_EVENT";
  }
}

const char* dump_av_sm_event_name(int event) {
  return dump_av_sm_event_name(static_cast<btif_av_sm_event_t>(event));
}

BtifAvEvent::BtifAvEvent(uint32_t event, const void* p_data, size_t data_length)
    : event_(event), data_(nullptr), data_length_(0) {
  DeepCopy(event, p_data, data_length);
}

BtifAvEvent::BtifAvEvent(const BtifAvEvent& other)
    : event_(0), data_(nullptr), data_length_(0) {
  *this = other;
}

BtifAvEvent& BtifAvEvent::operator=(const BtifAvEvent& other) {
  DeepFree();
  DeepCopy(other.Event(), other.Data(), other.DataLength());
  return *this;
}

BtifAvEvent::~BtifAvEvent() { DeepFree(); }

std::string BtifAvEvent::ToString() const {
  return BtifAvEvent::EventName(event_);
}

std::string BtifAvEvent::EventName(uint32_t event) {
  std::string name = dump_av_sm_event_name((btif_av_sm_event_t)event);
  std::stringstream ss_value;
  ss_value << "(0x" << std::hex << event << ")";
  return name + ss_value.str();
}

void BtifAvEvent::DeepCopy(uint32_t event, const void* p_data,
                           size_t data_length) {
  event_ = event;
  data_length_ = data_length;
  if (data_length == 0) {
    data_ = nullptr;
  } else {
    data_ = osi_malloc(data_length_);
    memcpy(data_, p_data, data_length);
  }

  switch (event) {
    case BTA_AV_META_MSG_EVT: {
      CHECK(data_length >= sizeof(tBTA_AV));
      const tBTA_AV* av_src = (const tBTA_AV*)p_data;
      tBTA_AV* av_dest = (tBTA_AV*)data_;
      if (av_src->meta_msg.p_data && av_src->meta_msg.len) {
        av_dest->meta_msg.p_data = (uint8_t*)osi_calloc(av_src->meta_msg.len);
        memcpy(av_dest->meta_msg.p_data, av_src->meta_msg.p_data,
               av_src->meta_msg.len);
      }

      if (av_src->meta_msg.p_msg) {
        av_dest->meta_msg.p_msg = (tAVRC_MSG*)osi_calloc(sizeof(tAVRC_MSG));
        memcpy(av_dest->meta_msg.p_msg, av_src->meta_msg.p_msg,
               sizeof(tAVRC_MSG));

        tAVRC_MSG* p_msg_src = av_src->meta_msg.p_msg;
        tAVRC_MSG* p_msg_dest = av_dest->meta_msg.p_msg;

        if ((p_msg_src->hdr.opcode == AVRC_OP_VENDOR) &&
            (p_msg_src->vendor.p_vendor_data && p_msg_src->vendor.vendor_len)) {
          p_msg_dest->vendor.p_vendor_data =
              (uint8_t*)osi_calloc(p_msg_src->vendor.vendor_len);
          memcpy(p_msg_dest->vendor.p_vendor_data,
                 p_msg_src->vendor.p_vendor_data, p_msg_src->vendor.vendor_len);
        }
        if ((p_msg_src->hdr.opcode == AVRC_OP_BROWSE) &&
            p_msg_src->browse.p_browse_data && p_msg_src->browse.browse_len) {
          p_msg_dest->browse.p_browse_data =
              (uint8_t*)osi_calloc(p_msg_src->browse.browse_len);
          memcpy(p_msg_dest->browse.p_browse_data,
                 p_msg_src->browse.p_browse_data, p_msg_src->browse.browse_len);
        }
      }
    } break;

    default:
      break;
  }
}

void BtifAvEvent::DeepFree() {
  switch (event_) {
    case BTA_AV_META_MSG_EVT: {
      tBTA_AV* av = (tBTA_AV*)data_;
      osi_free_and_reset((void**)&av->meta_msg.p_data);

      if (av->meta_msg.p_msg) {
        if (av->meta_msg.p_msg->hdr.opcode == AVRC_OP_VENDOR) {
          osi_free(av->meta_msg.p_msg->vendor.p_vendor_data);
        }
        if (av->meta_msg.p_msg->hdr.opcode == AVRC_OP_BROWSE) {
          osi_free(av->meta_msg.p_msg->browse.p_browse_data);
        }
        osi_free_and_reset((void**)&av->meta_msg.p_msg);
      }
    } break;

    default:
      break;
  }

  osi_free_and_reset((void**)&data_);
  data_length_ = 0;
}

BtifAvPeer::BtifAvPeer(const RawAddress& peer_address, uint8_t peer_sep,
                       tBTA_AV_HNDL bta_handle, uint8_t peer_id)
    : peer_address_(peer_address),
      peer_sep_(peer_sep),
      bta_handle_(bta_handle),
      peer_id_(peer_id),
      state_machine_(*this),
      av_open_on_rc_timer_(nullptr),
      edr_(0),
      flags_(0),
      self_initiated_connection_(false),
      delay_report_(0) {}

BtifAvPeer::~BtifAvPeer() { alarm_free(av_open_on_rc_timer_); }

std::string BtifAvPeer::FlagsToString() const {
  std::string result;

  if (flags_ & BtifAvPeer::kFlagLocalSuspendPending) {
    if (!result.empty()) result += "|";
    result += "LOCAL_SUSPEND_PENDING";
  }
  if (flags_ & BtifAvPeer::kFlagRemoteSuspend) {
    if (!result.empty()) result += "|";
    result += "REMOTE_SUSPEND";
  }
  if (flags_ & BtifAvPeer::kFlagPendingStart) {
    if (!result.empty()) result += "|";
    result += "PENDING_START";
  }
  if (flags_ & BtifAvPeer::kFlagPendingStop) {
    if (!result.empty()) result += "|";
    result += "PENDING_STOP";
  }
  if (result.empty()) result = "None";

  return base::StringPrintf("0x%x(%s)", flags_, result.c_str());
}

bt_status_t BtifAvPeer::Init() {
  alarm_free(av_open_on_rc_timer_);
  av_open_on_rc_timer_ = alarm_new("btif_av_peer.av_open_on_rc_timer");
  is_silenced_ = false;

  state_machine_.Start();
  return BT_STATUS_SUCCESS;
}

void BtifAvPeer::Cleanup() {
  state_machine_.Quit();
  alarm_free(av_open_on_rc_timer_);
  av_open_on_rc_timer_ = nullptr;
}

bool BtifAvPeer::CanBeDeleted() const {
  return (
      (state_machine_.StateId() == BtifAvStateMachine::kStateIdle) &&
      (state_machine_.PreviousStateId() != BtifAvStateMachine::kStateInvalid));
}

const RawAddress& BtifAvPeer::ActivePeerAddress() const {
  if (IsSource()) {
    return btif_av_sink.ActivePeer();
  }
  if (IsSink()) {
    return btif_av_source.ActivePeer();
  }

  if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
    RawAddress addr = btif_av_sink.ActivePeer();
    if (RawAddress::kEmpty == addr) {
      LOG(WARNING) << __PRETTY_FUNCTION__ << ": A2DP peer "
                   << btif_av_source.ActivePeer() << " is Sink";
      return btif_av_source.ActivePeer();
    }
    LOG(WARNING) << __PRETTY_FUNCTION__ << ": A2DP peer "
                 << btif_av_sink.ActivePeer() << " is Source";
    return btif_av_sink.ActivePeer();
  }
  LOG(FATAL) << __PRETTY_FUNCTION__ << ": A2DP peer "
             << ADDRESS_TO_LOGGABLE_STR(PeerAddress())
             << " is neither Source nor Sink";
  return RawAddress::kEmpty;
}

bool BtifAvPeer::IsConnected() const {
  int state = state_machine_.StateId();
  return ((state == BtifAvStateMachine::kStateOpened) ||
          (state == BtifAvStateMachine::kStateStarted));
}

bool BtifAvPeer::IsStreaming() const {
  int state = state_machine_.StateId();
  return (state == BtifAvStateMachine::kStateStarted);
}

BtifAvSource::~BtifAvSource() { CleanupAllPeers(); }

bt_status_t BtifAvSource::Init(
    btav_source_callbacks_t* callbacks, int max_connected_audio_devices,
    const std::vector<btav_a2dp_codec_config_t>& codec_priorities,
    const std::vector<btav_a2dp_codec_config_t>& offloading_preference,
    std::vector<btav_a2dp_codec_info_t>* supported_codecs) {
  LOG_INFO("%s: max_connected_audio_devices=%d", __PRETTY_FUNCTION__,
           max_connected_audio_devices);
  if (enabled_) return BT_STATUS_SUCCESS;
  CleanupAllPeers();
  max_connected_peers_ = max_connected_audio_devices;

  /* A2DP OFFLOAD */
  a2dp_offload_enabled_ =
      GetInterfaceToProfiles()->config->isA2DPOffloadEnabled();
  LOG_VERBOSE("a2dp_offload.enable = %d", a2dp_offload_enabled_);

  callbacks_ = callbacks;
  if (a2dp_offload_enabled_) {
    tBTM_BLE_VSC_CB vsc_cb = {};
    BTM_BleGetVendorCapabilities(&vsc_cb);
    bool supports_a2dp_hw_offload_v2 =
        vsc_cb.version_supported >= 0x0104 && vsc_cb.a2dp_offload_v2_support;
    bluetooth::audio::a2dp::update_codec_offloading_capabilities(
        offloading_preference, supports_a2dp_hw_offload_v2);
  }
  bta_av_co_init(codec_priorities, supported_codecs);

  if (!btif_a2dp_source_init()) {
    return BT_STATUS_FAIL;
  }
  enabled_ = true;
  btif_enable_service(BTA_A2DP_SOURCE_SERVICE_ID);
  return BT_STATUS_SUCCESS;
}

void BtifAvSource::Cleanup() {
  LOG_INFO("%s", __PRETTY_FUNCTION__);
  if (!enabled_) return;
  enabled_ = false;

  btif_queue_cleanup(UUID_SERVCLASS_AUDIO_SOURCE);

  std::promise<void> peer_ready_promise;
  do_in_main_thread(
      FROM_HERE,
      base::BindOnce(base::IgnoreResult(&BtifAvSource::SetActivePeer),
                     base::Unretained(&btif_av_source), RawAddress::kEmpty,
                     std::move(peer_ready_promise)));
  do_in_main_thread(FROM_HERE, base::BindOnce(&btif_a2dp_source_cleanup));

  btif_disable_service(BTA_A2DP_SOURCE_SERVICE_ID);
  CleanupAllPeers();

  callbacks_ = nullptr;
}

BtifAvPeer* BtifAvSource::FindPeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it != peers_.end()) return it->second;
  return nullptr;
}

BtifAvPeer* BtifAvSource::FindPeerByHandle(tBTA_AV_HNDL bta_handle) {
  for (auto it : peers_) {
    BtifAvPeer* peer = it.second;
    if (peer->BtaHandle() == bta_handle) {
      return peer;
    }
  }
  return nullptr;
}

BtifAvPeer* BtifAvSource::FindPeerByPeerId(uint8_t peer_id) {
  for (auto it : peers_) {
    BtifAvPeer* peer = it.second;
    if (peer->PeerId() == peer_id) {
      return peer;
    }
  }
  return nullptr;
}

BtifAvPeer* BtifAvSource::FindOrCreatePeer(const RawAddress& peer_address,
                                           tBTA_AV_HNDL bta_handle) {
  std::unique_lock<std::mutex> lock1(mutex_);
  LOG_VERBOSE("%s: peer_address=%s bta_handle=0x%x", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address), bta_handle);

  BtifAvPeer* peer = FindPeer(peer_address);
  if (peer != nullptr) return peer;

  // Find next availabie Peer ID to use
  uint8_t peer_id;
  for (peer_id = kPeerIdMin; peer_id < kPeerIdMax; peer_id++) {
    /* because the peer id may be in source cb and we cannot use it */
    if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
      if (FindPeerByPeerId(peer_id) == nullptr &&
          btif_av_sink.FindPeerByPeerId(peer_id) == nullptr)
        break;
    } else {
      if (FindPeerByPeerId(peer_id) == nullptr) break;
    }
  }
  if (peer_id == kPeerIdMax) {
    LOG_ERROR(
        "%s: Cannot create peer for peer_address=%s : "
        "cannot allocate unique Peer ID",
        __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return nullptr;
  }

  // Get the BTA Handle (if known)
  if (bta_handle == kBtaHandleUnknown) {
    auto it = peer_id2bta_handle_.find(peer_id);
    if (it == peer_id2bta_handle_.end() || it->second == kBtaHandleUnknown) {
      LOG_ERROR(
          "%s: Cannot create peer for peer_address=%s : "
          "cannot convert Peer ID=%d to unique BTA Handle",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address), peer_id);
      return nullptr;
    }
    bta_handle = it->second;
  }

  LOG_INFO("%s: Create peer: peer_address=%s bta_handle=0x%x peer_id=%d",
           __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address),
           bta_handle, peer_id);
  peer = new BtifAvPeer(peer_address, AVDT_TSEP_SNK, bta_handle, peer_id);
  peers_.insert(std::make_pair(peer_address, peer));
  peer->Init();
  return peer;
}

bool BtifAvSource::AllowedToConnect(const RawAddress& peer_address) const {
  int connected = 0;
  if (btif_av_src_sink_coexist_enabled() && invalid_peer_check_) {
    LOG_INFO(
        "invalid_peer_check_ so allow to connect here, when"
        " BTA_AV_OPEN_EVT coming, would check again!");
    return true;
  }

  // Count peers that are in the process of connecting or already connected
  for (auto it : peers_) {
    const BtifAvPeer* peer = it.second;
    switch (peer->StateMachine().StateId()) {
      case BtifAvStateMachine::kStateOpening:
      case BtifAvStateMachine::kStateOpened:
      case BtifAvStateMachine::kStateStarted:
        if (peer->PeerAddress() == peer_address) {
          /* we should check if another role is used */
          if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable())
            break;

          return true;  // Already connected or accounted for
        }
        connected++;
        break;
      default:
        break;
    }
  }
  if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
    LOG_INFO("%s: connected=%d, max_connected_peers_=%d, sink_peers=%d",
             __PRETTY_FUNCTION__, connected, max_connected_peers_,
             (int)btif_av_sink.Peers().size());
    /* if source device connected, don't connect sink device */

    if (connected >= max_connected_peers_ || !btif_av_sink.Peers().empty()) {
      return false;
    } else {
      return true;
    }
  }
  return (connected < max_connected_peers_);
}

bool BtifAvSource::DeletePeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it == peers_.end()) return false;
  BtifAvPeer* peer = it->second;
  peer->Cleanup();
  peers_.erase(it);
  delete peer;
  return true;
}

void BtifAvSource::DeleteIdlePeers() {
  for (auto it = peers_.begin(); it != peers_.end();) {
    BtifAvPeer* peer = it->second;
    auto prev_it = it++;
    if (!peer->CanBeDeleted()) continue;
    LOG_INFO("%s: Deleting idle peer: %s bta_handle=0x%x", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), peer->BtaHandle());
    peer->Cleanup();
    peers_.erase(prev_it);
    delete peer;
  }
}

void BtifAvSource::CleanupAllPeers() {
  while (!peers_.empty()) {
    auto it = peers_.begin();
    BtifAvPeer* peer = it->second;
    peer->Cleanup();
    peers_.erase(it);
    delete peer;
  }
}

void BtifAvSource::RegisterAllBtaHandles() {
  for (int peer_id = kPeerIdMin; peer_id < kPeerIdMax; peer_id++) {
    BTA_AvRegister(BTA_AV_CHNL_AUDIO, kBtifAvSourceServiceName.c_str(), peer_id,
                   nullptr, UUID_SERVCLASS_AUDIO_SOURCE);
  }
}

void BtifAvSource::DeregisterAllBtaHandles() {
  for (auto it : peer_id2bta_handle_) {
    tBTA_AV_HNDL bta_handle = it.second;
    BTA_AvDeregister(bta_handle);
  }
  peer_id2bta_handle_.clear();
}

void BtifAvSource::BtaHandleRegistered(uint8_t peer_id,
                                       tBTA_AV_HNDL bta_handle) {
  peer_id2bta_handle_.insert(std::make_pair(peer_id, bta_handle));

  // Set the BTA Handle for the Peer (if exists)
  BtifAvPeer* peer = FindPeerByPeerId(peer_id);
  if (peer != nullptr && peer->BtaHandle() != bta_handle) {
    if (peer->BtaHandle() == kBtaHandleUnknown) {
      LOG_VERBOSE("%s: Assign peer: peer_address=%s bta_handle=0x%x peer_id=%d",
                  __PRETTY_FUNCTION__,
                  ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), bta_handle,
                  peer_id);
    } else {
      LOG_WARN(
          "%s: Correct peer: peer_address=%s bta_handle=0x%x->0x%x peer_id=%d",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
          peer->BtaHandle(), bta_handle, peer_id);
    }
    peer->SetBtaHandle(bta_handle);
  }
}

BtifAvPeer* BtifAvSource::popPeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it == peers_.end()) return nullptr;
  BtifAvPeer* peer = it->second;
  peers_.erase(it);
  LOG_INFO("%s: peer_address=%s, state=%d", __PRETTY_FUNCTION__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
           peer->StateMachine().StateId());
  return peer;
}

void BtifAvSource::AddPeer(BtifAvPeer* peer) {
  LOG_INFO("%s: peer_address=%s, state=%d", __PRETTY_FUNCTION__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
           peer->StateMachine().StateId());
  peers_.insert(std::make_pair(peer->PeerAddress(), peer));
}
BtifAvSink::~BtifAvSink() { CleanupAllPeers(); }

bt_status_t BtifAvSink::Init(btav_sink_callbacks_t* callbacks,
                             int max_connected_audio_devices) {
  LOG_INFO("%s(max_connected_audio_devices=%d)", __PRETTY_FUNCTION__,
           max_connected_audio_devices);
  if (enabled_) return BT_STATUS_SUCCESS;

  CleanupAllPeers();
  max_connected_peers_ = max_connected_audio_devices;
  callbacks_ = callbacks;

  /** source will have this configuration, but sink don't have, so don't
   * overwrite it. */
  if (!btif_av_source.Enabled()) {
    std::vector<btav_a2dp_codec_config_t>
        codec_priorities;  // Default priorities
    std::vector<btav_a2dp_codec_info_t> supported_codecs;
    bta_av_co_init(codec_priorities, &supported_codecs);
  }

  if (!btif_a2dp_sink_init()) {
    return BT_STATUS_FAIL;
  }
  enabled_ = true;
  btif_enable_service(BTA_A2DP_SINK_SERVICE_ID);
  return BT_STATUS_SUCCESS;
}

void BtifAvSink::Cleanup() {
  LOG_INFO("%s", __PRETTY_FUNCTION__);
  if (!enabled_) return;
  enabled_ = false;

  btif_queue_cleanup(UUID_SERVCLASS_AUDIO_SINK);

  std::promise<void> peer_ready_promise;
  do_in_main_thread(
      FROM_HERE,
      base::BindOnce(base::IgnoreResult(&BtifAvSink::SetActivePeer),
                     base::Unretained(&btif_av_sink), RawAddress::kEmpty,
                     std::move(peer_ready_promise)));
  do_in_main_thread(FROM_HERE, base::BindOnce(&btif_a2dp_sink_cleanup));

  btif_disable_service(BTA_A2DP_SINK_SERVICE_ID);
  CleanupAllPeers();

  callbacks_ = nullptr;
}

BtifAvPeer* BtifAvSink::FindPeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it != peers_.end()) return it->second;
  return nullptr;
}

BtifAvPeer* BtifAvSink::FindPeerByHandle(tBTA_AV_HNDL bta_handle) {
  for (auto it : peers_) {
    BtifAvPeer* peer = it.second;
    if (peer->BtaHandle() == bta_handle) {
      return peer;
    }
  }
  return nullptr;
}

BtifAvPeer* BtifAvSink::FindPeerByPeerId(uint8_t peer_id) {
  for (auto it : peers_) {
    BtifAvPeer* peer = it.second;
    if (peer->PeerId() == peer_id) {
      return peer;
    }
  }
  return nullptr;
}

BtifAvPeer* BtifAvSink::FindOrCreatePeer(const RawAddress& peer_address,
                                         tBTA_AV_HNDL bta_handle) {
  LOG_VERBOSE("%s: peer_address=%s bta_handle=0x%x", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address), bta_handle);

  BtifAvPeer* peer = FindPeer(peer_address);
  if (peer != nullptr) return peer;

  // Find next availabie Peer ID to use
  uint8_t peer_id;
  for (peer_id = kPeerIdMin; peer_id < kPeerIdMax; peer_id++) {
    /* because the peer id may be in source cb and we cannot use it */
    if (btif_av_both_enable()) {
      if (FindPeerByPeerId(peer_id) == nullptr &&
          btif_av_source.FindPeerByPeerId(peer_id) == nullptr)
        break;

    } else {
      if (FindPeerByPeerId(peer_id) == nullptr) break;
    }
  }
  if (peer_id == kPeerIdMax) {
    LOG_ERROR(
        "%s: Cannot create peer for peer_address=%s : "
        "cannot allocate unique Peer ID",
        __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return nullptr;
  }

  // Get the BTA Handle (if known)
  if (bta_handle == kBtaHandleUnknown) {
    auto it = peer_id2bta_handle_.find(peer_id);
    if (it == peer_id2bta_handle_.end() || it->second == kBtaHandleUnknown) {
      LOG_ERROR(
          "%s: Cannot create peer for peer_address=%s : "
          "cannot convert Peer ID=%d to unique BTA Handle",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address), peer_id);
      return nullptr;
    }
    bta_handle = it->second;
  }

  LOG_INFO("%s: Create peer: peer_address=%s bta_handle=0x%x peer_id=%d",
           __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_address),
           bta_handle, peer_id);
  peer = new BtifAvPeer(peer_address, AVDT_TSEP_SRC, bta_handle, peer_id);
  peers_.insert(std::make_pair(peer_address, peer));
  peer->Init();
  return peer;
}

bool BtifAvSink::AllowedToConnect(const RawAddress& peer_address) const {
  int connected = 0;

  if (btif_av_src_sink_coexist_enabled() && invalid_peer_check_) {
    LOG_INFO(
        "invalid_peer_check_ so allow to connect here, when"
        " BTA_AV_OPEN_EVT coming, would check again!");
    return true;
  }
  // Count peers that are in the process of connecting or already connected
  for (auto it : peers_) {
    const BtifAvPeer* peer = it.second;
    switch (peer->StateMachine().StateId()) {
      case BtifAvStateMachine::kStateOpening:
      case BtifAvStateMachine::kStateOpened:
      case BtifAvStateMachine::kStateStarted:
        if (peer->PeerAddress() == peer_address) {
          /* we should check if another role is used */
          if (btif_av_both_enable()) break;
          return true;  // Already connected or accounted for
        }
        connected++;
        break;
      case BtifAvStateMachine::kStateClosing:
      case BtifAvStateMachine::kStateIdle:
        if ((btif_a2dp_sink_get_audio_track() != nullptr) &&
          (peer->PeerAddress() != peer_address)) {
          LOG_INFO("%s: there is another peer with audio track(%p), another=%s, peer=%s",
            __PRETTY_FUNCTION__, btif_a2dp_sink_get_audio_track(),
            ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
            ADDRESS_TO_LOGGABLE_CSTR(peer_address));
          connected++;
        }
        break;
      default:
        break;
    }
  }
  if (btif_av_both_enable()) {
    LOG_INFO("connected=%d, max_connected_peers_=%d, source_peers=%d",
             connected, max_connected_peers_,
             (int)btif_av_source.Peers().size());
    /* if source device connected, don't connect sink device */
    return (connected < max_connected_peers_) && btif_av_source.Peers().empty();
  }

  return (connected < max_connected_peers_);
}

bool BtifAvSink::DeletePeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it == peers_.end()) return false;
  BtifAvPeer* peer = it->second;
  peer->Cleanup();
  peers_.erase(it);
  delete peer;
  return true;
}

void BtifAvSink::DeleteIdlePeers() {
  for (auto it = peers_.begin(); it != peers_.end();) {
    BtifAvPeer* peer = it->second;
    auto prev_it = it++;
    if (!peer->CanBeDeleted()) continue;
    LOG_INFO("%s: Deleting idle peer: %s bta_handle=0x%x", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), peer->BtaHandle());
    peer->Cleanup();
    peers_.erase(prev_it);
    delete peer;
  }
}

void BtifAvSink::CleanupAllPeers() {
  while (!peers_.empty()) {
    auto it = peers_.begin();
    BtifAvPeer* peer = it->second;
    peer->Cleanup();
    peers_.erase(it);
    delete peer;
  }
}

void BtifAvSink::RegisterAllBtaHandles() {
  for (int peer_id = kPeerIdMin; peer_id < kPeerIdMax; peer_id++) {
    BTA_AvRegister(BTA_AV_CHNL_AUDIO, kBtifAvSinkServiceName.c_str(), peer_id,
                   bta_av_sink_media_callback, UUID_SERVCLASS_AUDIO_SINK);
  }
}

void BtifAvSink::DeregisterAllBtaHandles() {
  for (auto it : peer_id2bta_handle_) {
    tBTA_AV_HNDL bta_handle = it.second;
    BTA_AvDeregister(bta_handle);
  }
  peer_id2bta_handle_.clear();
}

void BtifAvSink::BtaHandleRegistered(uint8_t peer_id, tBTA_AV_HNDL bta_handle) {
  peer_id2bta_handle_.insert(std::make_pair(peer_id, bta_handle));

  // Set the BTA Handle for the Peer (if exists)
  BtifAvPeer* peer = FindPeerByPeerId(peer_id);
  if (peer != nullptr && peer->BtaHandle() != bta_handle) {
    if (peer->BtaHandle() == kBtaHandleUnknown) {
      LOG_VERBOSE("%s: Assign peer: peer_address=%s bta_handle=0x%x peer_id=%d",
                  __PRETTY_FUNCTION__,
                  ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), bta_handle,
                  peer_id);
    } else {
      LOG_WARN(
          "%s: Correct peer: peer_address=%s bta_handle=0x%x->0x%x peer_id=%d",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
          peer->BtaHandle(), bta_handle, peer_id);
    }
    peer->SetBtaHandle(bta_handle);
  }
}

BtifAvPeer* BtifAvSink::popPeer(const RawAddress& peer_address) {
  auto it = peers_.find(peer_address);
  if (it == peers_.end()) return nullptr;
  BtifAvPeer* peer = it->second;
  peers_.erase(it);
  LOG_INFO("%s: peer_address=%s, state=%d", __PRETTY_FUNCTION__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
           peer->StateMachine().StateId());
  return peer;
}

void BtifAvSink::AddPeer(BtifAvPeer* peer) {
  LOG_INFO("%s: peer_address=%s, state=%d", __PRETTY_FUNCTION__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
           peer->StateMachine().StateId());
  peers_.insert(std::make_pair(peer->PeerAddress(), peer));
}

void BtifAvStateMachine::StateIdle::OnEnter() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  peer_.SetEdr(0);
  peer_.ClearAllFlags();

  // Stop A2DP if this is the active peer
  if (peer_.IsActivePeer() || peer_.ActivePeerAddress().IsEmpty()) {
    btif_a2dp_on_idle(peer_.PeerAddress());
  }

  // Reset the active peer if this was the active peer and
  // the Idle state was reentered
  if (peer_.IsActivePeer() && peer_.CanBeDeleted()) {
    std::promise<void> peer_ready_promise;
    if (peer_.IsSink()) {
      btif_av_source.SetActivePeer(RawAddress::kEmpty,
                                   std::move(peer_ready_promise));
    } else if (peer_.IsSource()) {
      btif_av_sink.SetActivePeer(RawAddress::kEmpty,
                                 std::move(peer_ready_promise));
    }
  }

  // Delete peers that are re-entering the Idle state
  if (peer_.IsSink()) {
    do_in_main_thread(FROM_HERE,
                      base::BindOnce(&BtifAvSource::DeleteIdlePeers,
                                     base::Unretained(&btif_av_source)));
  } else if (peer_.IsSource()) {
    do_in_main_thread(FROM_HERE,
                      base::BindOnce(&BtifAvSink::DeleteIdlePeers,
                                     base::Unretained(&btif_av_sink)));
  }
}

void BtifAvStateMachine::StateIdle::OnExit() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
}

bool BtifAvStateMachine::StateIdle::ProcessEvent(uint32_t event, void* p_data) {
  LOG_VERBOSE(
      "%s: Peer %s : event=%s flags=%s active_peer=%s", __PRETTY_FUNCTION__,
      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
      BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
      logbool(peer_.IsActivePeer()).c_str());

  switch (event) {
    case BTA_AV_ENABLE_EVT:
      break;

    case BTIF_AV_STOP_STREAM_REQ_EVT:
    case BTIF_AV_SUSPEND_STREAM_REQ_EVT:
    case BTIF_AV_ACL_DISCONNECTED:
      // Ignore. Just re-enter Idle so the peer can be deleted
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    case BTIF_AV_DISCONNECT_REQ_EVT:
      if (peer_.BtaHandle() != kBtaHandleUnknown) {
        BTA_AvClose(peer_.BtaHandle());
        if (peer_.IsSource()) {
          BTA_AvCloseRc(peer_.BtaHandle());
        }
      }
      // Re-enter Idle so the peer can be deleted
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    case BTIF_AV_CONNECT_REQ_EVT:
    case BTA_AV_PENDING_EVT: {
      bool can_connect = true;
      peer_.SetSelfInitiatedConnection(event == BTIF_AV_CONNECT_REQ_EVT);
      // Check whether connection is allowed
      if (peer_.IsSink()) {
        can_connect = btif_av_source.AllowedToConnect(peer_.PeerAddress());
        if (!can_connect) src_disconnect_sink(peer_.PeerAddress());
      } else if (peer_.IsSource()) {
        can_connect = btif_av_sink.AllowedToConnect(peer_.PeerAddress());
        if (!can_connect) sink_disconnect_src(peer_.PeerAddress());
      }
      if (!can_connect) {
        LOG_ERROR(
            "%s: Cannot connect to peer %s: too many connected "
            "peers",
            __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
        if (peer_.SelfInitiatedConnection()) {
          btif_queue_advance();
        }
        break;
      }
      btif_av_query_mandatory_codec_priority(peer_.PeerAddress());
      BTA_AvOpen(peer_.PeerAddress(), peer_.BtaHandle(), true,
                 peer_.LocalUuidServiceClass());
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateOpening);
      if (event == BTIF_AV_CONNECT_REQ_EVT) {
        DEVICE_IOT_CONFIG_ADDR_SET_INT(
            peer_.PeerAddress(), IOT_CONF_KEY_A2DP_ROLE,
            (peer_.LocalUuidServiceClass() == UUID_SERVCLASS_AUDIO_SOURCE)
                ? IOT_CONF_VAL_A2DP_ROLE_SINK
                : IOT_CONF_VAL_A2DP_ROLE_SOURCE);
        DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                           IOT_CONF_KEY_A2DP_CONN_COUNT);
      } else if (event == BTA_AV_PENDING_EVT) {
        DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                           IOT_CONF_KEY_A2DP_CONN_COUNT);
      }
    } break;
    case BTIF_AV_AVRCP_OPEN_EVT:
    case BTA_AV_RC_OPEN_EVT: {
      // IOP_FIX: Jabra 620 only does AVRCP Open without AV Open whenever it
      // connects. So as per the AV WP, an AVRCP connection cannot exist
      // without an AV connection. Therefore, we initiate an AV connection
      // if an RC_OPEN_EVT is received when we are in AV_CLOSED state.
      // We initiate the AV connection after a small 3s timeout to avoid any
      // collisions from the headsets, as some headsets initiate the AVRCP
      // connection first and then immediately initiate the AV connection
      //
      // TODO: We may need to do this only on an AVRCP Play. FixMe
      LOG_WARN("%s: Peer %s : event=%s received without AV",
               __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());

      bool can_connect = true;
      // Check whether connection is allowed
      if (peer_.IsSink()) {
        can_connect = btif_av_source.AllowedToConnect(peer_.PeerAddress());
        if (!can_connect) {
          if (btif_av_src_sink_coexist_enabled())
            BTA_AvCloseRc(((tBTA_AV*)p_data)->rc_open.rc_handle);
          else
            src_disconnect_sink(peer_.PeerAddress());
        }
      } else if (peer_.IsSource()) {
        can_connect = btif_av_sink.AllowedToConnect(peer_.PeerAddress());
        if (!can_connect) {
          if (btif_av_src_sink_coexist_enabled())
            BTA_AvCloseRc(((tBTA_AV*)p_data)->rc_open.rc_handle);
          else
            sink_disconnect_src(peer_.PeerAddress());
        }
      }
      if (!can_connect) {
        LOG_ERROR(
            "%s: Cannot connect to peer %s: too many connected "
            "peers",
            __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
        break;
      }
      /* if peer is source, then start timer for sink connect to src */
      if (btif_av_src_sink_coexist_enabled()) {
        if (peer_.IsSource()) {
          alarm_set_on_mloop(
              peer_.AvOpenOnRcTimer(), BtifAvPeer::kTimeoutAvOpenOnRcMs,
              btif_av_sink_initiate_av_open_timer_timeout, &peer_);
        } else {
          alarm_set_on_mloop(
              peer_.AvOpenOnRcTimer(), BtifAvPeer::kTimeoutAvOpenOnRcMs,
              btif_av_source_initiate_av_open_timer_timeout, &peer_);
        }
      } else {
        if (btif_av_source.Enabled()) {
          alarm_set_on_mloop(
              peer_.AvOpenOnRcTimer(), BtifAvPeer::kTimeoutAvOpenOnRcMs,
              btif_av_source_initiate_av_open_timer_timeout, &peer_);
        } else if (btif_av_sink.Enabled()) {
          alarm_set_on_mloop(
              peer_.AvOpenOnRcTimer(), BtifAvPeer::kTimeoutAvOpenOnRcMs,
              btif_av_sink_initiate_av_open_timer_timeout, &peer_);
        }
      }
      if (event == BTA_AV_RC_OPEN_EVT) {
        btif_rc_handler(event, (tBTA_AV*)p_data);
      }
    } break;

    case BTA_AV_RC_BROWSE_OPEN_EVT:
      btif_rc_handler(event, (tBTA_AV*)p_data);
      break;

    // In case Signalling channel is not down and remote started Streaming
    // Procedure, we have to handle Config and Open event in Idle state.
    // We hit these scenarios while running PTS test case for AVRCP Controller.
    case BTIF_AV_SINK_CONFIG_REQ_EVT: {
      const btif_av_sink_config_req_t* p_config_req =
          static_cast<const btif_av_sink_config_req_t*>(p_data);
      btif_av_report_sink_audio_config_state(p_config_req->peer_address,
                                             p_config_req->sample_rate,
                                             p_config_req->channel_count);
    } break;

    case BTA_AV_OPEN_EVT: {
      tBTA_AV* p_bta_data = (tBTA_AV*)p_data;
      tBTA_AV_STATUS status = p_bta_data->open.status;
      bool can_connect = true;

      LOG_INFO(
          "%s: Peer %s : event=%s flags=%s status=%d(%s) edr=0x%x",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
          status, (status == BTA_AV_SUCCESS) ? "SUCCESS" : "FAILED",
          p_bta_data->open.edr);

      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_CONNECTING,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      if (p_bta_data->open.status == BTA_AV_SUCCESS) {
        peer_.SetEdr(p_bta_data->open.edr);
        if (btif_av_src_sink_coexist_enabled()) {
          LOG_VERBOSE("%s: Peer %s sep=%d, open_sep=%d", __PRETTY_FUNCTION__,
                      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                      peer_.PeerSep(), p_bta_data->open.sep);
          /* if peer is wrong sep type, move it to BtifAvSxxx */
          if (peer_.PeerSep() == AVDT_TSEP_SNK) {
            LOG_VERBOSE("set source invalid_peer_check as false");
            btif_av_source.SetInvalidPeerCheck(false);
          } else {
            LOG_VERBOSE("set sink invalid_peer_check as false");
            btif_av_sink.SetInvalidPeerCheck(false);
          }
          if (peer_.PeerSep() != p_bta_data->open.sep) {
            BtifAvPeer* tmp_peer = nullptr;
            if (peer_.PeerSep() == AVDT_TSEP_SNK) {
              tmp_peer = btif_av_source.popPeer(peer_.PeerAddress());

              if (peer_.PeerAddress() != tmp_peer->PeerAddress())
                LOG_ERROR("error, not same peer");

              btif_av_sink.AddPeer(tmp_peer);
            } else {
              tmp_peer = btif_av_sink.popPeer(peer_.PeerAddress());

              if (peer_.PeerAddress() != tmp_peer->PeerAddress())
                LOG_ERROR("error, not same peer");

              btif_av_source.AddPeer(tmp_peer);
            }
            peer_.SetSep(p_bta_data->open.sep);
          }
          if (btif_rc_is_connected_peer(peer_.PeerAddress())) {
            LOG_VERBOSE("%s, AVRCP connected, update avrc sep", __func__);
            BTA_AvSetPeerSep(peer_.PeerAddress(), peer_.PeerSep());
          }
          btif_rc_check_pending_cmd(p_bta_data->open.bd_addr);
        }
        CHECK(peer_.PeerSep() == p_bta_data->open.sep);

        can_connect = peer_.IsSink()
                          ? btif_av_source.AllowedToConnect(peer_.PeerAddress())
                          : btif_av_sink.AllowedToConnect(peer_.PeerAddress());

        if (!can_connect) {
          LOG_ERROR(
              "%s: Cannot connect to peer %s: too many connected "
              "peers",
              __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

          if (peer_.IsSink()) {
            src_disconnect_sink(peer_.PeerAddress());
          } else if (peer_.IsSource()) {
            sink_disconnect_src(peer_.PeerAddress());
          }

          btif_report_connection_state(
              peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTED,
              bt_status_t::BT_STATUS_NOMEM, BTA_AV_FAIL_RESOURCES);
          peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
        } else {
          if (peer_.IsSink()) {
            // If queued PLAY command, send it now
            btif_rc_check_handle_pending_play(
                p_bta_data->open.bd_addr,
                (p_bta_data->open.status == BTA_AV_SUCCESS));
          } else if (peer_.IsSource() &&
                     (p_bta_data->open.status == BTA_AV_SUCCESS)) {
            // Bring up AVRCP connection as well
            BTA_AvOpenRc(peer_.BtaHandle());
          }
          btif_report_connection_state(
              peer_.PeerAddress(), BTAV_CONNECTION_STATE_CONNECTED,
              bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);
          peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateOpened);
        }
      } else {
        btif_report_connection_state(peer_.PeerAddress(),
                                     BTAV_CONNECTION_STATE_DISCONNECTED,
                                     bt_status_t::BT_STATUS_FAIL, status);
        peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
        DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                           IOT_CONF_KEY_A2DP_CONN_FAIL_COUNT);
      }
      btif_queue_advance();
    } break;

    case BTA_AV_REMOTE_CMD_EVT:
    case BTA_AV_VENDOR_CMD_EVT:
    case BTA_AV_META_MSG_EVT:
    case BTA_AV_RC_FEAT_EVT:
    case BTA_AV_RC_PSM_EVT:
    case BTA_AV_REMOTE_RSP_EVT:
      btif_rc_handler(event, (tBTA_AV*)p_data);
      break;

    case BTIF_AV_AVRCP_CLOSE_EVT:
    case BTA_AV_RC_CLOSE_EVT: {
      LOG_VERBOSE("%s: Peer %s : event=%s : Stopping AV timer",
                  __PRETTY_FUNCTION__,
                  ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                  BtifAvEvent::EventName(event).c_str());
      alarm_cancel(peer_.AvOpenOnRcTimer());

      if (event == BTA_AV_RC_CLOSE_EVT) {
        btif_rc_handler(event, (tBTA_AV*)p_data);
      }
    } break;

    case BTIF_AV_OFFLOAD_START_REQ_EVT:
      LOG_ERROR("%s: Peer %s : event=%s: stream is not Opened",
                __PRETTY_FUNCTION__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                BtifAvEvent::EventName(event).c_str());
      btif_a2dp_on_offload_started(peer_.PeerAddress(), BTA_AV_FAIL);
      break;

    default:
      LOG_WARN("%s: Peer %s : Unhandled event=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      return false;
  }

  return true;
}

void BtifAvStateMachine::StateOpening::OnEnter() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  // Inform the application that we are entering connecting state
  if (btif_av_both_enable()) {
    /* if peer connect to us, don't know which role it is */
    if (!peer_.SelfInitiatedConnection()) return;
  }
  btif_report_connection_state(peer_.PeerAddress(),
                               BTAV_CONNECTION_STATE_CONNECTING,
                               bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);
}

void BtifAvStateMachine::StateOpening::OnExit() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
}

bool BtifAvStateMachine::StateOpening::ProcessEvent(uint32_t event,
                                                    void* p_data) {
  LOG_VERBOSE(
      "%s: Peer %s : event=%s flags=%s active_peer=%s", __PRETTY_FUNCTION__,
      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
      BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
      logbool(peer_.IsActivePeer()).c_str());

  switch (event) {
    case BTIF_AV_STOP_STREAM_REQ_EVT:
    case BTIF_AV_SUSPEND_STREAM_REQ_EVT:
      break;  // Ignore

    case BTIF_AV_ACL_DISCONNECTED:
      // ACL Disconnected needs to be handled only in Opening state, because
      // it is in an intermediate state. In other states we can handle
      // incoming/outgoing connect/disconnect requests.
      LOG_WARN(
          "%s: Peer %s : event=%s: transitioning to Idle due to ACL "
          "Disconnect",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str());
      log_counter_metrics_btif(android::bluetooth::CodePathCounterKeyEnum::
                                   A2DP_CONNECTION_ACL_DISCONNECTED,
                               1);
      btif_report_connection_state(peer_.PeerAddress(),
                                   BTAV_CONNECTION_STATE_DISCONNECTED,
                                   bt_status_t::BT_STATUS_FAIL, BTA_AV_FAIL);
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      if (peer_.SelfInitiatedConnection()) {
        btif_queue_advance();
      }
      break;
    case BTA_AV_REJECT_EVT:
      LOG_WARN("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());
      log_counter_metrics_btif(android::bluetooth::CodePathCounterKeyEnum::
                                   A2DP_CONNECTION_REJECT_EVT,
                               1);
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTED,
          bt_status_t::BT_STATUS_AUTH_REJECTED, BTA_AV_FAIL);
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      if (peer_.SelfInitiatedConnection()) {
        btif_queue_advance();
      }
      break;

    case BTA_AV_OPEN_EVT: {
      tBTA_AV* p_bta_data = (tBTA_AV*)p_data;
      int av_state;
      tBTA_AV_STATUS status = p_bta_data->open.status;

      LOG_INFO(
          "%s: Peer %s : event=%s flags=%s status=%d(%s) edr=0x%x",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
          status, (status == BTA_AV_SUCCESS) ? "SUCCESS" : "FAILED",
          p_bta_data->open.edr);

      if (p_bta_data->open.status == BTA_AV_SUCCESS) {
        av_state = BtifAvStateMachine::kStateOpened;
        peer_.SetEdr(p_bta_data->open.edr);
        if (btif_av_src_sink_coexist_enabled()) {
          LOG_VERBOSE("%s: Peer %s sep=%d, open_sep=%d", __PRETTY_FUNCTION__,
                      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                      peer_.PeerSep(), p_bta_data->open.sep);
          /* if peer is wrong sep type, move it to BtifAvSxxx */
          if (peer_.PeerSep() == AVDT_TSEP_SNK) {
            LOG_VERBOSE("set source invalid_peer_check as false");
            btif_av_source.SetInvalidPeerCheck(false);
          } else {
            LOG_VERBOSE("set sink invalid_peer_check as false");
            btif_av_sink.SetInvalidPeerCheck(false);
          }
          if (peer_.PeerSep() != p_bta_data->open.sep) {
            BtifAvPeer* tmp_peer = nullptr;
            if (peer_.PeerSep() == AVDT_TSEP_SNK) {
              tmp_peer = btif_av_source.popPeer(peer_.PeerAddress());

              if (peer_.PeerAddress() != tmp_peer->PeerAddress())
                LOG_ERROR("error, not same peer");

              btif_av_sink.AddPeer(tmp_peer);
            } else {
              tmp_peer = btif_av_sink.popPeer(peer_.PeerAddress());

              if (peer_.PeerAddress() != tmp_peer->PeerAddress())
                LOG_ERROR("error, not same peer");

              btif_av_source.AddPeer(tmp_peer);
            }
            peer_.SetSep(p_bta_data->open.sep);
          }
          if (btif_rc_is_connected_peer(peer_.PeerAddress())) {
            LOG_VERBOSE("%s, AVRCP connected, update avrc sep", __func__);
            BTA_AvSetPeerSep(peer_.PeerAddress(), peer_.PeerSep());
          }
          btif_rc_check_pending_cmd(p_bta_data->open.bd_addr);
        }
        CHECK(peer_.PeerSep() == p_bta_data->open.sep);
        /** normally it can be checked in IDLE PENDING/CONNECT_REQ, in case:
         * 1 speacker connected to DUT and phone connect DUT, because
         * default
         * connect req is as SINK peer. only at here, we can know which
         * role
         *  it is.@{ */
        if (btif_av_src_sink_coexist_enabled()) {
          bool can_connect = true;
          if (peer_.IsSink()) {
            can_connect = btif_av_source.AllowedToConnect(peer_.PeerAddress());
            if (!can_connect) src_disconnect_sink(peer_.PeerAddress());
          } else if (peer_.IsSource()) {
            can_connect = btif_av_sink.AllowedToConnect(peer_.PeerAddress());
            if (!can_connect) sink_disconnect_src(peer_.PeerAddress());
          }
        }
        /** @} */

        // Report the connection state to the application
        btif_report_connection_state(
            peer_.PeerAddress(), BTAV_CONNECTION_STATE_CONNECTED,
            bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);
        log_counter_metrics_btif(
            android::bluetooth::CodePathCounterKeyEnum::A2DP_CONNECTION_SUCCESS,
            1);
      } else {
        if (btif_rc_is_connected_peer(peer_.PeerAddress())) {
          // Disconnect the AVRCP connection, in case the A2DP connectiton
          // failed for any reason.
          LOG_WARN("%s: Peer %s : Disconnecting AVRCP", __PRETTY_FUNCTION__,
                   ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
          uint8_t peer_handle =
              btif_rc_get_connected_peer_handle(peer_.PeerAddress());
          if (peer_handle != BTRC_HANDLE_NONE) {
            BTA_AvCloseRc(peer_handle);
          }
          DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                             IOT_CONF_KEY_A2DP_CONN_FAIL_COUNT);
        }
        av_state = BtifAvStateMachine::kStateIdle;
        // Report the connection state to the application
        btif_report_connection_state(peer_.PeerAddress(),
                                     BTAV_CONNECTION_STATE_DISCONNECTED,
                                     bt_status_t::BT_STATUS_FAIL, status);
        log_counter_metrics_btif(
            android::bluetooth::CodePathCounterKeyEnum::A2DP_CONNECTION_FAILURE,
            1);
      }

      // Change state to Open/Idle based on the status
      peer_.StateMachine().TransitionTo(av_state);
      if (peer_.IsSink()) {
        // If queued PLAY command, send it now
        btif_rc_check_handle_pending_play(
            p_bta_data->open.bd_addr,
            (p_bta_data->open.status == BTA_AV_SUCCESS));
      } else if (peer_.IsSource() &&
                 (p_bta_data->open.status == BTA_AV_SUCCESS)) {
        // Bring up AVRCP connection as well
        if (btif_av_src_sink_coexist_enabled() &&
            btif_av_sink.AllowedToConnect(peer_.PeerAddress()))
          BTA_AvOpenRc(peer_.BtaHandle());
      }
      if (peer_.SelfInitiatedConnection()) {
        btif_queue_advance();
      }
    } break;

    case BTIF_AV_SINK_CONFIG_REQ_EVT: {
      const btif_av_sink_config_req_t* p_config_req =
          static_cast<const btif_av_sink_config_req_t*>(p_data);
      /* before this point, we don't know it's role, actually peer is source */
      if (btif_av_both_enable()) {
        btif_av_report_sink_audio_config_state(p_config_req->peer_address,
                                               p_config_req->sample_rate,
                                               p_config_req->channel_count);
        break;
      }
      if (peer_.IsSource()) {
        btif_av_report_sink_audio_config_state(p_config_req->peer_address,
                                               p_config_req->sample_rate,
                                               p_config_req->channel_count);
      }
    } break;

    case BTIF_AV_CONNECT_REQ_EVT: {
      // The device has moved already to Opening, hence don't report the
      // connection state.
      LOG_WARN(
          "%s: Peer %s : event=%s : device is already connecting, "
          "ignore Connect request",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str());
      log_counter_metrics_btif(
          android::bluetooth::CodePathCounterKeyEnum::A2DP_ALREADY_CONNECTING,
          1);
      btif_queue_advance();
    } break;

    case BTA_AV_PENDING_EVT: {
      // The device has moved already to Opening, hence don't report the
      // connection state.
      LOG_WARN(
          "%s: Peer %s : event=%s : device is already connecting, "
          "ignore incoming request",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str());
      log_counter_metrics_btif(
          android::bluetooth::CodePathCounterKeyEnum::A2DP_ALREADY_CONNECTING,
          1);
    } break;

    case BTIF_AV_OFFLOAD_START_REQ_EVT:
      LOG_ERROR("%s: Peer %s : event=%s: stream is not Opened",
                __PRETTY_FUNCTION__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                BtifAvEvent::EventName(event).c_str());
      btif_a2dp_on_offload_started(peer_.PeerAddress(), BTA_AV_FAIL);
      log_counter_metrics_btif(android::bluetooth::CodePathCounterKeyEnum::
                                   A2DP_OFFLOAD_START_REQ_FAILURE,
                               1);
      break;

    case BTA_AV_CLOSE_EVT:
      btif_a2dp_on_stopped(nullptr);
      btif_report_connection_state(peer_.PeerAddress(),
                                   BTAV_CONNECTION_STATE_DISCONNECTED,
                                   bt_status_t::BT_STATUS_FAIL, BTA_AV_FAIL);
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      log_counter_metrics_btif(
          android::bluetooth::CodePathCounterKeyEnum::A2DP_CONNECTION_CLOSE, 1);
      DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                         IOT_CONF_KEY_A2DP_CONN_FAIL_COUNT);
      if (peer_.SelfInitiatedConnection()) {
        btif_queue_advance();
      }
      break;

    case BTIF_AV_DISCONNECT_REQ_EVT:
      BTA_AvClose(peer_.BtaHandle());
      btif_report_connection_state(peer_.PeerAddress(),
                                   BTAV_CONNECTION_STATE_DISCONNECTED,
                                   bt_status_t::BT_STATUS_FAIL, BTA_AV_FAIL);
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      DEVICE_IOT_CONFIG_ADDR_INT_ADD_ONE(peer_.PeerAddress(),
                                         IOT_CONF_KEY_A2DP_CONN_FAIL_COUNT);
      log_counter_metrics_btif(android::bluetooth::CodePathCounterKeyEnum::
                                   A2DP_CONNECTION_DISCONNECTED,
                               1);
      if (peer_.SelfInitiatedConnection()) {
        btif_queue_advance();
      }
      break;

      CHECK_RC_EVENT(event, (tBTA_AV*)p_data);

    default:
      log_counter_metrics_btif(android::bluetooth::CodePathCounterKeyEnum::
                                   A2DP_CONNECTION_UNKNOWN_EVENT,
                               1);
      LOG_WARN("%s: Peer %s : Unhandled event=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      return false;
  }
  return true;
}

void BtifAvStateMachine::StateOpened::OnEnter() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  peer_.ClearFlags(BtifAvPeer::kFlagLocalSuspendPending |
                   BtifAvPeer::kFlagPendingStart |
                   BtifAvPeer::kFlagPendingStop);

  // Set the active peer if the first connected device.
  // NOTE: This should be done only if we are A2DP Sink, because the A2DP Sink
  // implementation in Java doesn't support active devices (yet).
  // For A2DP Source, the setting of the Active device is done by the
  // ActiveDeviceManager in Java.
  if (peer_.IsSource() && btif_av_sink.ActivePeer().IsEmpty()) {
    std::promise<void> peer_ready_promise;
    if (!btif_av_sink.SetActivePeer(peer_.PeerAddress(),
                                    std::move(peer_ready_promise))) {
      LOG_ERROR("%s: Error setting %s as active Source peer", __func__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
    }
  }
}

void BtifAvStateMachine::StateOpened::OnExit() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  peer_.ClearFlags(BtifAvPeer::kFlagPendingStart);
}

bool BtifAvStateMachine::StateOpened::ProcessEvent(uint32_t event,
                                                   void* p_data) {
  tBTA_AV* p_av = (tBTA_AV*)p_data;

  LOG_VERBOSE(
      "%s: Peer %s : event=%s flags=%s active_peer=%s", __PRETTY_FUNCTION__,
      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
      BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
      logbool(peer_.IsActivePeer()).c_str());

  if ((event == BTA_AV_REMOTE_CMD_EVT) &&
      peer_.CheckFlags(BtifAvPeer::kFlagRemoteSuspend) &&
      (p_av->remote_cmd.rc_id == AVRC_ID_PLAY)) {
    LOG_VERBOSE("%s: Peer %s : Resetting remote suspend flag on RC PLAY",
                __PRETTY_FUNCTION__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
    peer_.ClearFlags(BtifAvPeer::kFlagRemoteSuspend);
  }

  switch (event) {
    case BTIF_AV_STOP_STREAM_REQ_EVT:
    case BTIF_AV_SUSPEND_STREAM_REQ_EVT:
    case BTIF_AV_ACL_DISCONNECTED:
      break;  // Ignore

    case BTIF_AV_START_STREAM_REQ_EVT: {
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());
      if (p_data) {
        const btif_av_start_stream_req_t* p_start_steam_req =
            static_cast<const btif_av_start_stream_req_t*>(p_data);
        LOG_INFO("Stream use_latency_mode=%s",
                 p_start_steam_req->use_latency_mode ? "true" : "false");
        peer_.SetUseLatencyMode(p_start_steam_req->use_latency_mode);
      }

      BTA_AvStart(peer_.BtaHandle(), peer_.UseLatencyMode());
      peer_.SetFlags(BtifAvPeer::kFlagPendingStart);
    } break;

    case BTA_AV_START_EVT: {
      LOG_INFO(
          "%s: Peer %s : event=%s status=%d suspending=%d "
          "initiator=%d flags=%s",
          __PRETTY_FUNCTION__, ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
          BtifAvEvent::EventName(event).c_str(), p_av->start.status,
          p_av->start.suspending, p_av->start.initiator,
          peer_.FlagsToString().c_str());

      if ((p_av->start.status == BTA_SUCCESS) && p_av->start.suspending)
        return true;

      // If remote tries to start A2DP when DUT is A2DP Source, then Suspend.
      // If A2DP is Sink and call is active, then disconnect the AVDTP
      // channel.
      bool should_suspend = false;
      if (peer_.IsSink()) {
        if (!peer_.CheckFlags(BtifAvPeer::kFlagPendingStart |
                              BtifAvPeer::kFlagRemoteSuspend)) {
          LOG(WARNING) << __PRETTY_FUNCTION__ << ": Peer "
                       << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                       << " : trigger Suspend as remote initiated";
          should_suspend = true;
        } else if (!peer_.IsActivePeer()) {
          LOG(WARNING) << __PRETTY_FUNCTION__ << ": Peer "
                       << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                       << " : trigger Suspend as non-active";
          should_suspend = true;
        }

        // If peer is A2DP Source, do ACK commands to audio HAL and start
        // media task
        if (btif_a2dp_on_started(peer_.PeerAddress(), &p_av->start)) {
          // Only clear pending flag after acknowledgement
          peer_.ClearFlags(BtifAvPeer::kFlagPendingStart);
        }
      }

      // Remain in Open state if status failed
      if (p_av->start.status != BTA_AV_SUCCESS) return false;

      if (peer_.IsSource() && peer_.IsActivePeer()) {
        // Remove flush state, ready for streaming
        btif_a2dp_sink_set_rx_flush(false);
        btif_a2dp_sink_on_start();
      }

      if (should_suspend) {
        btif_av_source_dispatch_sm_event(peer_.PeerAddress(),
                                         BTIF_AV_SUSPEND_STREAM_REQ_EVT);
      }
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateStarted);

    } break;

    case BTIF_AV_DISCONNECT_REQ_EVT:
      BTA_AvClose(peer_.BtaHandle());
      if (peer_.IsSource()) {
        BTA_AvCloseRc(peer_.BtaHandle());
      }

      // Inform the application that we are disconnecting
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTING,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      // Wait in closing state until fully closed
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateClosing);
      break;

    case BTA_AV_CLOSE_EVT:
      // AVDTP link is closed
      // Inform the application that we are disconnecting
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTING,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);
      // Change state to Idle, send acknowledgement if start is pending
      if (peer_.CheckFlags(BtifAvPeer::kFlagPendingStart)) {
        LOG_WARN("%s: Peer %s : failed pending start request",
                 __PRETTY_FUNCTION__,
                 ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
        tBTA_AV_START av_start = {.chnl = p_av->close.chnl,
                                  .hndl = p_av->close.hndl,
                                  .status = BTA_AV_FAIL_STREAM,
                                  .initiator = true,
                                  .suspending = true};
        btif_a2dp_on_started(peer_.PeerAddress(), &av_start);
        // Pending start flag will be cleared when exit current state
      } else if (peer_.IsActivePeer()) {
        btif_a2dp_on_stopped(nullptr);
      }

      // Inform the application that we are disconnected
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTED,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    case BTA_AV_RECONFIG_EVT:
      if (p_av->reconfig.status != BTA_AV_SUCCESS) {
        LOG(WARNING) << __PRETTY_FUNCTION__ << ": Peer "
                     << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                     << " : failed reconfiguration";
        if (peer_.CheckFlags(BtifAvPeer::kFlagPendingStart)) {
          LOG(ERROR) << __PRETTY_FUNCTION__ << ": Peer "
                     << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                     << " : cannot proceed to do AvStart";
          peer_.ClearFlags(BtifAvPeer::kFlagPendingStart);
          btif_a2dp_command_ack(A2DP_CTRL_ACK_FAILURE);
        }
        if (peer_.IsSink()) {
          src_disconnect_sink(peer_.PeerAddress());
        } else if (peer_.IsSource()) {
          sink_disconnect_src(peer_.PeerAddress());
        }
        break;
      }

      if (peer_.IsActivePeer()) {
        LOG(INFO) << __PRETTY_FUNCTION__ << " : Peer "
                  << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                  << " : Reconfig done - calling startSession() to audio HAL";
        std::promise<void> peer_ready_promise;
        std::future<void> peer_ready_future = peer_ready_promise.get_future();
        btif_a2dp_source_start_session(peer_.PeerAddress(),
                                       std::move(peer_ready_promise));
      }
      if (peer_.CheckFlags(BtifAvPeer::kFlagPendingStart)) {
        LOG(INFO) << __PRETTY_FUNCTION__ << " : Peer "
                  << ADDRESS_TO_LOGGABLE_STR(peer_.PeerAddress())
                  << " : Reconfig done - calling BTA_AvStart("
                  << loghex(peer_.BtaHandle()) << ")";
        BTA_AvStart(peer_.BtaHandle(), peer_.UseLatencyMode());
      }
      break;

    case BTIF_AV_CONNECT_REQ_EVT: {
      LOG_WARN("%s: Peer %s : Ignore %s for same device", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      btif_queue_advance();
    } break;

    case BTIF_AV_OFFLOAD_START_REQ_EVT:
      LOG_ERROR("%s: Peer %s : event=%s: stream is not Started",
                __PRETTY_FUNCTION__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                BtifAvEvent::EventName(event).c_str());
      btif_a2dp_on_offload_started(peer_.PeerAddress(), BTA_AV_FAIL);
      break;

    case BTIF_AV_AVRCP_REMOTE_PLAY_EVT:
      if (peer_.CheckFlags(BtifAvPeer::kFlagRemoteSuspend)) {
        LOG_VERBOSE("%s: Peer %s : Resetting remote suspend flag on RC PLAY",
                    __PRETTY_FUNCTION__,
                    ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
        peer_.ClearFlags(BtifAvPeer::kFlagRemoteSuspend);
      }
      break;

      CHECK_RC_EVENT(event, (tBTA_AV*)p_data);

    case BTIF_AV_SET_LATENCY_REQ_EVT: {
      const btif_av_set_latency_req_t* p_set_latency_req =
          static_cast<const btif_av_set_latency_req_t*>(p_data);
      LOG_INFO("Peer %s : event=%s flags=%s is_low_latency=%s",
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str(),
               p_set_latency_req->is_low_latency ? "true" : "false");

      BTA_AvSetLatency(peer_.BtaHandle(), p_set_latency_req->is_low_latency);
    } break;

    case BTIF_AV_SET_CODEC_MODE_EVT: {
      const btif_av_codec_mode_change_t* p_codec_mode_change =
          static_cast<const btif_av_codec_mode_change_t*>(p_data);
      LOG_INFO("Peer %s : event=%s flags=%s enc_mode=%d",
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str(),
               p_codec_mode_change->enc_mode);

      BTA_AvSetCodecMode(peer_.BtaHandle(), p_codec_mode_change->enc_mode);
    } break;

    default:
      LOG_WARN("%s: Peer %s : Unhandled event=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      return false;
  }
  return true;
}

void BtifAvStateMachine::StateStarted::OnEnter() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  // We are again in started state, clear any remote suspend flags
  peer_.ClearFlags(BtifAvPeer::kFlagRemoteSuspend);

  btif_a2dp_sink_set_rx_flush(false);

  // Report that we have entered the Streaming stage. Usually, this should
  // be followed by focus grant. See update_audio_focus_state()
  btif_report_audio_state(peer_.PeerAddress(), BTAV_AUDIO_STATE_STARTED);
}

void BtifAvStateMachine::StateStarted::OnExit() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
}

bool BtifAvStateMachine::StateStarted::ProcessEvent(uint32_t event,
                                                    void* p_data) {
  tBTA_AV* p_av = (tBTA_AV*)p_data;

  LOG_VERBOSE(
      "%s: Peer %s : event=%s flags=%s active_peer=%s", __PRETTY_FUNCTION__,
      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
      BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
      logbool(peer_.IsActivePeer()).c_str());

  switch (event) {
    case BTIF_AV_ACL_DISCONNECTED:
      break;  // Ignore

    case BTIF_AV_START_STREAM_REQ_EVT:
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());
      // We were started remotely, just ACK back the local request
      if (peer_.IsSink()) btif_a2dp_on_started(peer_.PeerAddress(), nullptr);
      break;

    // FIXME -- use suspend = true always to work around issue with BTA AV
    case BTIF_AV_STOP_STREAM_REQ_EVT:
    case BTIF_AV_SUSPEND_STREAM_REQ_EVT:
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());

      // There is a pending LocalSuspend already, ignore.
      if (peer_.CheckFlags(BtifAvPeer::kFlagLocalSuspendPending)) {
        break;
      }

      // Set pending flag to ensure the BTIF task is not trying to restart
      // the stream while suspend is in progress.
      peer_.SetFlags(BtifAvPeer::kFlagLocalSuspendPending);

      // If we were remotely suspended but suspend locally, local suspend
      // always overrides.
      peer_.ClearFlags(BtifAvPeer::kFlagRemoteSuspend);

      if (peer_.IsSink() &&
          (peer_.IsActivePeer() || !btif_av_stream_started_ready())) {
        // Immediately stop transmission of frames while suspend is pending
        if (event == BTIF_AV_STOP_STREAM_REQ_EVT) {
          btif_a2dp_on_stopped(nullptr);
        } else {
          // ensure tx frames are immediately suspended
          btif_a2dp_source_set_tx_flush(true);
        }
      } else if (peer_.IsSource()) {
        btif_a2dp_on_stopped(nullptr);
      }
      BTA_AvStop(peer_.BtaHandle(), true);
      break;

    case BTIF_AV_DISCONNECT_REQ_EVT:
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());

      // Request AVDTP to close
      BTA_AvClose(peer_.BtaHandle());
      if (peer_.IsSource()) {
        BTA_AvCloseRc(peer_.BtaHandle());
      }

      // Inform the application that we are disconnecting
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTING,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      // Wait in closing state until fully closed
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateClosing);
      break;

    case BTA_AV_SUSPEND_EVT: {
      LOG_INFO("%s: Peer %s : event=%s status=%d initiator=%d flags=%s",
               __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(), p_av->suspend.status,
               p_av->suspend.initiator, peer_.FlagsToString().c_str());

      // A2DP suspended, stop A2DP encoder / decoder until resumed
      if (peer_.IsActivePeer() || !btif_av_stream_started_ready()) {
        btif_a2dp_on_suspended(&p_av->suspend);
      }

      // If not successful, remain in current state
      if (p_av->suspend.status != BTA_AV_SUCCESS) {
        peer_.ClearFlags(BtifAvPeer::kFlagLocalSuspendPending);

        if (peer_.IsSink() && peer_.IsActivePeer()) {
          // Suspend failed, reset back tx flush state
          btif_a2dp_source_set_tx_flush(false);
        }
        return false;
      }

      btav_audio_state_t state = BTAV_AUDIO_STATE_REMOTE_SUSPEND;
      if (p_av->suspend.initiator != true) {
        // Remote suspend, notify HAL and await audioflinger to
        // suspend/stop stream.
        //
        // Set remote suspend flag to block media task from restarting
        // stream only if we did not already initiate a local suspend.
        if (!peer_.CheckFlags(BtifAvPeer::kFlagLocalSuspendPending))
          peer_.SetFlags(BtifAvPeer::kFlagRemoteSuspend);
      } else {
        state = BTAV_AUDIO_STATE_STOPPED;
      }

      btif_report_audio_state(peer_.PeerAddress(), state);
      // Suspend completed, clear local pending flags while entering Opened
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateOpened);
    } break;

    case BTA_AV_STOP_EVT:
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());

      peer_.SetFlags(BtifAvPeer::kFlagPendingStop);
      peer_.ClearFlags(BtifAvPeer::kFlagLocalSuspendPending);

      // Don't change the encoder and audio provider state by a non-active
      // peer since they are shared between peers
      if (peer_.IsActivePeer() || !btif_av_stream_started_ready()) {
        btif_a2dp_on_stopped(&p_av->suspend);
      }

      btif_report_audio_state(peer_.PeerAddress(), BTAV_AUDIO_STATE_STOPPED);

      // If stop was successful, change state to Open
      if (p_av->suspend.status == BTA_AV_SUCCESS)
        peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateOpened);

      break;

    case BTA_AV_CLOSE_EVT:
      LOG_INFO("%s: Peer %s : event=%s flags=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str());
      // Inform the application that we are disconnecting
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTING,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      peer_.SetFlags(BtifAvPeer::kFlagPendingStop);

      // AVDTP link is closed
      if (peer_.IsActivePeer()) {
        btif_a2dp_on_stopped(nullptr);
      }

      // Inform the application that we are disconnected
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTED,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    case BTIF_AV_OFFLOAD_START_REQ_EVT:
      if (peer_.CheckFlags(BtifAvPeer::kFlagLocalSuspendPending |
                           BtifAvPeer::kFlagRemoteSuspend |
                           BtifAvPeer::kFlagPendingStop)) {
        LOG_WARN("%s: Peer %s : event=%s flags=%s: stream is Suspending",
                 __PRETTY_FUNCTION__,
                 ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                 BtifAvEvent::EventName(event).c_str(),
                 peer_.FlagsToString().c_str());
        btif_a2dp_on_offload_started(peer_.PeerAddress(), BTA_AV_FAIL);
        break;
      }
      BTA_AvOffloadStart(peer_.BtaHandle());
      break;

    case BTA_AV_OFFLOAD_START_RSP_EVT:
      btif_a2dp_on_offload_started(peer_.PeerAddress(), p_av->status);
      break;

    case BTIF_AV_SET_LATENCY_REQ_EVT: {
      const btif_av_set_latency_req_t* p_set_latency_req =
          static_cast<const btif_av_set_latency_req_t*>(p_data);
      LOG_INFO("Peer %s : event=%s flags=%s is_low_latency=%s",
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str(),
               p_set_latency_req->is_low_latency ? "true" : "false");

      BTA_AvSetLatency(peer_.BtaHandle(), p_set_latency_req->is_low_latency);
    } break;

    case BTIF_AV_SET_CODEC_MODE_EVT: {
      const btif_av_codec_mode_change_t* p_codec_mode_change =
          static_cast<const btif_av_codec_mode_change_t*>(p_data);
      LOG_INFO("Peer %s : event=%s flags=%s enc_mode=%d",
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str(),
               peer_.FlagsToString().c_str(),
               p_codec_mode_change->enc_mode);

      BTA_AvSetCodecMode(peer_.BtaHandle(), p_codec_mode_change->enc_mode);
    } break;

      CHECK_RC_EVENT(event, (tBTA_AV*)p_data);

    default:
      LOG_WARN("%s: Peer %s : Unhandled event=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      return false;
  }

  return true;
}

void BtifAvStateMachine::StateClosing::OnEnter() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));

  if (peer_.IsActivePeer()) {
    if (peer_.IsSink()) {
      // Immediately stop transmission of frames
      btif_a2dp_source_set_tx_flush(true);
      // Wait for Audio Flinger to stop A2DP
    } else if (peer_.IsSource()) {
      btif_a2dp_sink_set_rx_flush(true);
    }
  }
}

void BtifAvStateMachine::StateClosing::OnExit() {
  LOG_VERBOSE("%s: Peer %s", __PRETTY_FUNCTION__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()));
}

bool BtifAvStateMachine::StateClosing::ProcessEvent(uint32_t event,
                                                    void* p_data) {
  LOG_VERBOSE(
      "%s: Peer %s : event=%s flags=%s active_peer=%s", __PRETTY_FUNCTION__,
      ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
      BtifAvEvent::EventName(event).c_str(), peer_.FlagsToString().c_str(),
      logbool(peer_.IsActivePeer()).c_str());

  switch (event) {
    case BTIF_AV_SUSPEND_STREAM_REQ_EVT:
    case BTIF_AV_ACL_DISCONNECTED:
      break;  // Ignore

    case BTA_AV_STOP_EVT:
    case BTIF_AV_STOP_STREAM_REQ_EVT:
      if (peer_.IsActivePeer()) {
        btif_a2dp_on_stopped(nullptr);
      }
      break;

    case BTA_AV_CLOSE_EVT:
      // Inform the application that we are disconnecting
      btif_report_connection_state(
          peer_.PeerAddress(), BTAV_CONNECTION_STATE_DISCONNECTED,
          bt_status_t::BT_STATUS_SUCCESS, BTA_AV_SUCCESS);

      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    // Handle the RC_CLOSE event for the cleanup
    case BTA_AV_RC_CLOSE_EVT:
      btif_rc_handler(event, (tBTA_AV*)p_data);
      break;

    // Handle the RC_BROWSE_CLOSE event for testing
    case BTA_AV_RC_BROWSE_CLOSE_EVT:
      btif_rc_handler(event, (tBTA_AV*)p_data);
      break;

    case BTIF_AV_OFFLOAD_START_REQ_EVT:
      LOG_ERROR("%s: Peer %s : event=%s: stream is not Opened",
                __PRETTY_FUNCTION__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
                BtifAvEvent::EventName(event).c_str());
      btif_a2dp_on_offload_started(peer_.PeerAddress(), BTA_AV_FAIL);
      break;

    case BTIF_AV_CONNECT_REQ_EVT:
      LOG_WARN("%s: Peer %s : Ignore %s in StateClosing", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      btif_queue_advance();
      peer_.StateMachine().TransitionTo(BtifAvStateMachine::kStateIdle);
      break;

    default:
      LOG_WARN("%s: Peer %s : Unhandled event=%s", __PRETTY_FUNCTION__,
               ADDRESS_TO_LOGGABLE_CSTR(peer_.PeerAddress()),
               BtifAvEvent::EventName(event).c_str());
      return false;
  }
  return true;
}

/**
 * Timer to trigger AV Open on the Source if the remote Sink device
 * establishes AVRCP connection without AV connection. The timer is needed to
 * interoperate with headsets that do establish AV after AVRCP connection.
 */
static void btif_av_source_initiate_av_open_timer_timeout(void* data) {
  BtifAvPeer* peer = (BtifAvPeer*)data;

  LOG_VERBOSE("%s: Peer %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));

  // Check if AVRCP is connected to the peer
  if (!btif_rc_is_connected_peer(peer->PeerAddress())) {
    LOG_ERROR("%s: AVRCP peer %s is not connected", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));
    return;
  }

  // Connect to the AVRCP peer
  if (btif_av_source.Enabled() &&
      btif_av_source.FindPeer(peer->PeerAddress()) == peer) {
    LOG_VERBOSE("%s: Connecting to AVRCP peer %s", __func__,
                ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));
    btif_av_source_dispatch_sm_event(peer->PeerAddress(),
                                     BTIF_AV_CONNECT_REQ_EVT);
  }
}

/**
 * Timer to trigger AV Open on the Sink if the remote Source device
 * establishes AVRCP connection without AV connection.
 */
static void btif_av_sink_initiate_av_open_timer_timeout(void* data) {
  BtifAvPeer* peer = (BtifAvPeer*)data;

  LOG_VERBOSE("%s: Peer %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));

  // Check if AVRCP is connected to the peer
  if (!btif_rc_is_connected_peer(peer->PeerAddress())) {
    LOG_ERROR("%s: AVRCP peer %s is not connected", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));
    return;
  }

  // Connect to the AVRCP peer
  if (btif_av_sink.Enabled() &&
      btif_av_sink.FindPeer(peer->PeerAddress()) == peer) {
    LOG_VERBOSE("%s: Connecting to AVRCP peer %s", __func__,
                ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()));
    btif_av_sink_dispatch_sm_event(peer->PeerAddress(),
                                   BTIF_AV_CONNECT_REQ_EVT);
  }
}

/**
 * Report the A2DP connection state
 *
 * @param peer_address the peer address
 * @param state the connection state
 */
static void btif_report_connection_state(const RawAddress& peer_address,
                                         btav_connection_state_t state,
                                         bt_status_t status,
                                         uint8_t error_code) {
  LOG_INFO("%s: peer_address=%s state=%d", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer_address), state);
  if (btif_av_src_sink_coexist_enabled() && btif_av_both_enable()) {
    BtifAvPeer* peer = btif_av_find_peer(peer_address);
    if (peer == nullptr) {
      LOG_ERROR("%s: peer is null", __func__);
      return;
    }

    if (peer->IsSink()) {
      do_in_jni_thread(
          FROM_HERE,
          base::BindOnce(btif_av_source.Callbacks()->connection_state_cb,
                         peer_address, state, btav_error_t{}));
    } else if (peer->IsSource()) {
      do_in_jni_thread(
          FROM_HERE,
          base::BindOnce(btif_av_sink.Callbacks()->connection_state_cb,
                         peer_address, state, btav_error_t{}));
    }
    return;
  }

  if (btif_av_source.Enabled()) {
    do_in_jni_thread(
        FROM_HERE,
        base::BindOnce(
            btif_av_source.Callbacks()->connection_state_cb, peer_address,
            state, btav_error_t{.status = status, .error_code = error_code}));
  } else if (btif_av_sink.Enabled()) {
    do_in_jni_thread(
        FROM_HERE,
        base::BindOnce(
            btif_av_sink.Callbacks()->connection_state_cb, peer_address, state,
            btav_error_t{.status = status, .error_code = error_code}));
  }
}

/**
 * Report the audio state of the A2DP connection.
 * The state is updated when either the remote ends starts streaming
 * (Started state) or whenever it transitions out of Started state
 * (to Opened or Streaming state).
 *
 * @param peer_address the peer address
 * @param state the audio state
 */
static void btif_report_audio_state(const RawAddress& peer_address,
                                    btav_audio_state_t state) {
  LOG_INFO("%s: peer_address=%s state=%d", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer_address), state);

  if (btif_av_both_enable()) {
    BtifAvPeer* peer = btif_av_find_peer(peer_address);
    if (peer->IsSink()) {
      do_in_jni_thread(
          FROM_HERE, base::BindOnce(btif_av_source.Callbacks()->audio_state_cb,
                                    peer_address, state));
    } else if (peer->IsSource()) {
      do_in_jni_thread(FROM_HERE,
                       base::BindOnce(btif_av_sink.Callbacks()->audio_state_cb,
                                      peer_address, state));
    }
    return;
  }
  if (btif_av_source.Enabled()) {
    do_in_jni_thread(FROM_HERE,
                     base::BindOnce(btif_av_source.Callbacks()->audio_state_cb,
                                    peer_address, state));
  } else if (btif_av_sink.Enabled()) {
    do_in_jni_thread(FROM_HERE,
                     base::BindOnce(btif_av_sink.Callbacks()->audio_state_cb,
                                    peer_address, state));
  }

  using android::bluetooth::a2dp::AudioCodingModeEnum;
  using android::bluetooth::a2dp::PlaybackStateEnum;
  PlaybackStateEnum playback_state = PlaybackStateEnum::PLAYBACK_STATE_UNKNOWN;
  switch (state) {
    case BTAV_AUDIO_STATE_STARTED:
      playback_state = PlaybackStateEnum::PLAYBACK_STATE_PLAYING;
      break;
    case BTAV_AUDIO_STATE_STOPPED:
      playback_state = PlaybackStateEnum::PLAYBACK_STATE_NOT_PLAYING;
      break;
    default:
      break;
  }
  AudioCodingModeEnum audio_coding_mode =
      btif_av_is_a2dp_offload_running()
          ? AudioCodingModeEnum::AUDIO_CODING_MODE_HARDWARE
          : AudioCodingModeEnum::AUDIO_CODING_MODE_SOFTWARE;

  log_a2dp_playback_event(peer_address, playback_state, audio_coding_mode);
}

void btif_av_report_source_codec_state(
    const RawAddress& peer_address,
    const btav_a2dp_codec_config_t& codec_config,
    const std::vector<btav_a2dp_codec_config_t>& codecs_local_capabilities,
    const std::vector<btav_a2dp_codec_config_t>&
        codecs_selectable_capabilities) {
  LOG_VERBOSE("%s: peer_address=%s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  if (btif_av_source.Enabled()) {
    do_in_jni_thread(
        FROM_HERE,
        base::BindOnce(btif_av_source.Callbacks()->audio_config_cb,
                       peer_address, codec_config, codecs_local_capabilities,
                       codecs_selectable_capabilities));
  }
}

/**
 * Report the audio config state of the A2DP Sink connection.
 *
 * @param peer_address the peer address
 * @param sample_rate the sample rate (in samples per second)
 * @param channel_count the channel count (1 for Mono, 2 for Stereo)
 */
static void btif_av_report_sink_audio_config_state(
    const RawAddress& peer_address, int sample_rate, int channel_count) {
  LOG_INFO("%s: Peer %s : sample_rate=%d channel_count=%d", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer_address), sample_rate, channel_count);
  if (btif_av_sink.Enabled()) {
    do_in_jni_thread(FROM_HERE,
                     base::BindOnce(btif_av_sink.Callbacks()->audio_config_cb,
                                    peer_address, sample_rate, channel_count));
  }
}

/**
 * Call out to JNI / JAVA layers to retrieve whether the mandatory codec is
 * more preferred than others.
 *
 * @param peer_address the peer address
 */
static void btif_av_query_mandatory_codec_priority(
    const RawAddress& peer_address) {
  auto query_priority = [](const RawAddress& peer_address) {
    if (!btif_av_source.Enabled()) {
      LOG_WARN("BTIF AV Source is not enabled");
      return;
    }
    btav_source_callbacks_t* callbacks = btif_av_source.Callbacks();
    bool preferred = callbacks != nullptr &&
                     callbacks->mandatory_codec_preferred_cb(peer_address);
    if (preferred) {
      auto apply_priority = [](const RawAddress& peer_address, bool preferred) {
        BtifAvPeer* peer = btif_av_find_peer(peer_address);
        if (peer == nullptr) {
          LOG_WARN("btif_av_query_mandatory_codec_priority: peer is null");
          return;
        }
        peer->SetMandatoryCodecPreferred(preferred);
      };
      do_in_main_thread(
          FROM_HERE, base::BindOnce(apply_priority, peer_address, preferred));
    }
  };
  if (btif_av_source.Enabled()) {
    do_in_jni_thread(FROM_HERE, base::BindOnce(query_priority, peer_address));
  }
}

static BtifAvPeer* btif_av_handle_both_peer(uint8_t peer_sep,
                                            const RawAddress& peer_address,
                                            tBTA_AV_HNDL bta_handle) {
  BtifAvPeer* peer = nullptr;

  if (peer_address != RawAddress::kEmpty) {
    if (btif_av_both_enable()) {
      peer = btif_av_find_peer(peer_address);
      /* if no this peer, default it's sink device */
      if (peer == nullptr) {
        if (peer_sep == AVDT_TSEP_SRC) {
          LOG_VERBOSE("%s: peer_sep(%d), create a new source peer", __func__,
                      peer_sep);
          peer = btif_av_sink.FindOrCreatePeer(peer_address, bta_handle);
        } else if (peer_sep == AVDT_TSEP_SNK) {
          LOG_VERBOSE("%s: peer_sep(%d), create a new sink peer", __func__,
                      peer_sep);
          peer = btif_av_source.FindOrCreatePeer(peer_address, bta_handle);
        } else {
          btif_av_source.SetInvalidPeerCheck(true);
          if (!btif_av_source.Peers().empty()) {
            LOG_VERBOSE(
                "%s: peer_sep invalid, and already has sink peer,"
                " so try create a new sink peer",
                __func__);
            peer = btif_av_source.FindOrCreatePeer(peer_address, bta_handle);
          } else if (!btif_av_sink.Peers().empty()) {
            LOG_VERBOSE(
                "%s: peer_sep invalid, and already has source peer,"
                " so try create a new source peer",
                __func__);
            peer = btif_av_sink.FindOrCreatePeer(peer_address, bta_handle);
          } else {
            LOG_VERBOSE(
                "%s: peer_sep invalid, and no active peer,"
                " so try create a new sink peer",
                __func__);
            peer = btif_av_source.FindOrCreatePeer(peer_address, bta_handle);
          }
        }
      }
    } else {
      if (peer_sep == AVDT_TSEP_SNK) {
        LOG_VERBOSE("%s: peer_sep(%d), only init src create a new source peer",
                    __func__, peer_sep);
        peer = btif_av_source.FindOrCreatePeer(peer_address, bta_handle);
      } else if (peer_sep == AVDT_TSEP_SRC) {
        LOG_VERBOSE("%s: peer_sep(%d), only init sink create a new source peer",
                    __func__, peer_sep);
        peer = btif_av_sink.FindOrCreatePeer(peer_address, bta_handle);
      }
    }
    if (peer == NULL && bta_handle != 0) {
      if (peer_sep == AVDT_TSEP_SNK) {
        peer = btif_av_source.FindPeerByHandle(bta_handle);
      } else if (peer_sep == AVDT_TSEP_SRC) {
        peer = btif_av_sink.FindPeerByHandle(bta_handle);
      }
      LOG_VERBOSE("%s:peer is check 3", __func__);
    }
  } else if (bta_handle != 0) {
    if (peer_sep == AVDT_TSEP_INVALID) {
      peer = btif_av_source.FindPeerByHandle(bta_handle);
      /* if no this peer, default it's sink device */
      if (peer == nullptr) {
        peer = btif_av_sink.FindPeerByHandle(bta_handle);
      }
    } else if (peer_sep == AVDT_TSEP_SNK) {
      peer = btif_av_source.FindPeerByHandle(bta_handle);
    } else if (peer_sep == AVDT_TSEP_SRC) {
      peer = btif_av_sink.FindPeerByHandle(bta_handle);
    }
  }
  return peer;
}

/**
 * Process BTIF or BTA AV or BTA AVRCP events. The processing is done on the
 * JNI thread.
 *
 * @param peer_sep the corresponding peer's SEP: AVDT_TSEP_SRC if the peer
 * is A2DP Source, or AVDT_TSEP_SNK if the peer is A2DP Sink.
 * @param peer_address the peer address if known, otherwise RawAddress::kEmpty
 * @param bta_handle the BTA handle for the peer if known, otherwise
 * kBtaHandleUnknown
 * @param btif_av_event the corresponding event
 */
static void btif_av_handle_event(uint8_t peer_sep,
                                 const RawAddress& peer_address,
                                 tBTA_AV_HNDL bta_handle,
                                 const BtifAvEvent& btif_av_event) {
  LOG_DEBUG("Handle event peer_address=%s bta_handle=0x%x",
            ADDRESS_TO_LOGGABLE_CSTR(peer_address), bta_handle);

  BtifAvPeer* peer = nullptr;

  // Find the peer
  if (btif_av_src_sink_coexist_enabled()) {
    peer = btif_av_handle_both_peer(peer_sep, peer_address, bta_handle);
  } else {
    if (peer_address != RawAddress::kEmpty) {
      if (peer_sep == AVDT_TSEP_SNK) {
        peer = btif_av_source.FindOrCreatePeer(peer_address, bta_handle);
      } else if (peer_sep == AVDT_TSEP_SRC) {
        peer = btif_av_sink.FindOrCreatePeer(peer_address, bta_handle);
      }
    } else if (bta_handle != kBtaHandleUnknown) {
      if (peer_sep == AVDT_TSEP_SNK) {
        peer = btif_av_source.FindPeerByHandle(bta_handle);
      } else if (peer_sep == AVDT_TSEP_SRC) {
        peer = btif_av_sink.FindPeerByHandle(bta_handle);
      }
    }
  }
  if (peer == nullptr) {
    LOG_ERROR(
        "jni_thread: Cannot find or create %s peer for peer_address=%s "
        " bta_handle=0x%x : event dropped: %s",
        peer_stream_endpoint_text(peer_sep).c_str(),
        ADDRESS_TO_LOGGABLE_CSTR(peer_address), bta_handle,
        btif_av_event.ToString().c_str());
    return;
  }

  peer->StateMachine().ProcessEvent(btif_av_event.Event(),
                                    btif_av_event.Data());
}

/**
 * Process BTA AV or BTA AVRCP events. The processing is done on the JNI
 * thread.
 *
 * @param peer_sep the corresponding peer's SEP: AVDT_TSEP_SRC if the peer
 * is A2DP Source, or AVDT_TSEP_SNK if the peer is A2DP Sink.
 * @param btif_av_event the corresponding event
 */
static void btif_av_handle_bta_av_event(uint8_t peer_sep,
                                        const BtifAvEvent& btif_av_event) {
  RawAddress peer_address = RawAddress::kEmpty;
  tBTA_AV_HNDL bta_handle = kBtaHandleUnknown;
  tBTA_AV_EVT event = btif_av_event.Event();
  tBTA_AV* p_data = (tBTA_AV*)btif_av_event.Data();
  std::string msg;

  LOG_DEBUG(
      "jni_thread: Handle BTA AV or AVRCP event %s: peer_sep=%hhu event=%s",
      peer_stream_endpoint_text(peer_sep).c_str(), peer_sep,
      btif_av_event.ToString().c_str());

  switch (event) {
    case BTA_AV_ENABLE_EVT: {
      const tBTA_AV_ENABLE& enable = p_data->enable;
      LOG_DEBUG("Enable features=0x%x", enable.features);
      return;  // Nothing to do
    }
    case BTA_AV_REGISTER_EVT: {
      const tBTA_AV_REGISTER& reg = p_data->reg;
      bta_handle = reg.hndl;
      uint8_t peer_id = reg.app_id;  // The PeerId is used as AppId
      LOG_DEBUG("Register bta_handle=0x%x app_id=%d", bta_handle, reg.app_id);
      if (btif_av_src_sink_coexist_enabled()) {
        if (peer_sep == AVDT_TSEP_INVALID) {
          if (reg.peer_sep == AVDT_TSEP_SNK)
            peer_sep = AVDT_TSEP_SNK;
          else
            peer_sep = AVDT_TSEP_SRC;
        }
      }
      if (peer_sep == AVDT_TSEP_SNK) {
        btif_av_source.BtaHandleRegistered(peer_id, bta_handle);
      } else if (peer_sep == AVDT_TSEP_SRC) {
        btif_av_sink.BtaHandleRegistered(peer_id, bta_handle);
      }
      return;  // Nothing else to do
    }
    case BTA_AV_OPEN_EVT: {
      const tBTA_AV_OPEN& open = p_data->open;
      peer_address = open.bd_addr;
      bta_handle = open.hndl;
      msg = "Stream opened";
      break;
    }
    case BTA_AV_CLOSE_EVT: {
      const tBTA_AV_CLOSE& close = p_data->close;
      bta_handle = close.hndl;
      msg = "Stream closed";
      break;
    }
    case BTA_AV_START_EVT: {
      const tBTA_AV_START& start = p_data->start;
      bta_handle = start.hndl;
      msg = "Stream started";
      break;
    }
    case BTA_AV_SUSPEND_EVT:
    case BTA_AV_STOP_EVT: {
      const tBTA_AV_SUSPEND& suspend = p_data->suspend;
      bta_handle = suspend.hndl;
      msg = "Stream stopped";
      break;
    }
    case BTA_AV_PROTECT_REQ_EVT: {
      const tBTA_AV_PROTECT_REQ& protect_req = p_data->protect_req;
      bta_handle = protect_req.hndl;
      break;
    }
    case BTA_AV_PROTECT_RSP_EVT: {
      const tBTA_AV_PROTECT_RSP& protect_rsp = p_data->protect_rsp;
      bta_handle = protect_rsp.hndl;
      break;
    }
    case BTA_AV_RC_OPEN_EVT: {
      const tBTA_AV_RC_OPEN& rc_open = p_data->rc_open;
      peer_address = rc_open.peer_addr;
      break;
    }
    case BTA_AV_RC_CLOSE_EVT: {
      const tBTA_AV_RC_CLOSE& rc_close = p_data->rc_close;
      peer_address = rc_close.peer_addr;
      break;
    }
    case BTA_AV_RC_BROWSE_OPEN_EVT: {
      const tBTA_AV_RC_BROWSE_OPEN& rc_browse_open = p_data->rc_browse_open;
      peer_address = rc_browse_open.peer_addr;
      break;
    }
    case BTA_AV_RC_BROWSE_CLOSE_EVT: {
      const tBTA_AV_RC_BROWSE_CLOSE& rc_browse_close = p_data->rc_browse_close;
      peer_address = rc_browse_close.peer_addr;
      break;
    }
    case BTA_AV_REMOTE_CMD_EVT:
    case BTA_AV_REMOTE_RSP_EVT:
    case BTA_AV_VENDOR_CMD_EVT:
    case BTA_AV_VENDOR_RSP_EVT:
    case BTA_AV_META_MSG_EVT: {
      if (btif_av_src_sink_coexist_enabled()) {
        if (peer_sep == AVDT_TSEP_INVALID) {
          const tBTA_AV_REMOTE_CMD& rc_rmt_cmd = p_data->remote_cmd;
          btif_rc_get_addr_by_handle(rc_rmt_cmd.rc_handle, peer_address);
          if (peer_address == RawAddress::kEmpty) {
            peer_address = btif_av_source.ActivePeer();
            if (peer_address == RawAddress::kEmpty) {
              peer_address = btif_av_sink.ActivePeer();
            }
          }
        } else if (peer_sep == AVDT_TSEP_SNK) {
          peer_address = btif_av_source.ActivePeer();
        } else if (peer_sep == AVDT_TSEP_SRC) {
          peer_address = btif_av_sink.ActivePeer();
        }
        break;
      } else
        FALLTHROUGH_INTENDED;
    }
    case BTA_AV_OFFLOAD_START_RSP_EVT: {
      // TODO: Might be wrong - this code will be removed once those
      // events are received from the AVRCP module.
      if (peer_sep == AVDT_TSEP_SNK) {
        peer_address = btif_av_source.ActivePeer();
        msg = "Stream sink offloaded";
      } else if (peer_sep == AVDT_TSEP_SRC) {
        peer_address = btif_av_sink.ActivePeer();
        msg = "Stream source offloaded";
      }
      break;
    }
    case BTA_AV_RECONFIG_EVT: {
      const tBTA_AV_RECONFIG& reconfig = p_data->reconfig;
      bta_handle = reconfig.hndl;
      break;
    }
    case BTA_AV_PENDING_EVT: {
      const tBTA_AV_PEND& pend = p_data->pend;
      peer_address = pend.bd_addr;
      break;
    }
    case BTA_AV_REJECT_EVT: {
      const tBTA_AV_REJECT& reject = p_data->reject;
      peer_address = reject.bd_addr;
      bta_handle = reject.hndl;
      break;
    }
    case BTA_AV_RC_FEAT_EVT: {
      const tBTA_AV_RC_FEAT& rc_feat = p_data->rc_feat;
      peer_address = rc_feat.peer_addr;
      break;
    }
    case BTA_AV_RC_PSM_EVT: {
      const tBTA_AV_RC_PSM& rc_psm = p_data->rc_cover_art_psm;
      peer_address = rc_psm.peer_addr;
      break;
    }
  }

  if (!msg.empty()) {
    BTM_LogHistory(kBtmLogHistoryTag, peer_address, msg,
                   btif_av_event.ToString());
  }
  btif_av_handle_event(peer_sep, peer_address, bta_handle, btif_av_event);
}

bool btif_av_both_enable(void) {
  return (btif_av_sink.Enabled() && btif_av_source.Enabled());
}

bool btif_av_src_sink_coexist_enabled(void) {
  return GET_SYSPROP(A2dp, src_sink_coexist, false);
}

static void bta_av_source_callback(tBTA_AV_EVT event, tBTA_AV* p_data) {
  BtifAvEvent btif_av_event(event, p_data, sizeof(tBTA_AV));
  LOG_VERBOSE("%s: event=%s", __func__, btif_av_event.ToString().c_str());

  do_in_main_thread(
      FROM_HERE, base::BindOnce(&btif_av_handle_bta_av_event,
                                AVDT_TSEP_SNK /* peer_sep */, btif_av_event));
}

static void bta_av_sink_callback(tBTA_AV_EVT event, tBTA_AV* p_data) {
  BtifAvEvent btif_av_event(event, p_data, sizeof(tBTA_AV));
  do_in_main_thread(
      FROM_HERE, base::BindOnce(&btif_av_handle_bta_av_event,
                                AVDT_TSEP_SRC /* peer_sep */, btif_av_event));
}

static void bta_av_event_callback(tBTA_AV_EVT event, tBTA_AV* p_data) {
  if (btif_av_both_enable()) {
    BtifAvEvent btif_av_event(event, p_data, sizeof(tBTA_AV));
    do_in_main_thread(
        FROM_HERE,
        base::BindOnce(&btif_av_handle_bta_av_event,
                       AVDT_TSEP_INVALID /* peer_sep */, btif_av_event));
    return;
  }

  if (btif_av_is_sink_enabled()) {
    return bta_av_sink_callback(event, p_data);
  }

  return bta_av_source_callback(event, p_data);
}

// TODO: All processing should be done on the JNI thread
static void bta_av_sink_media_callback(const RawAddress& peer_address,
                                       tBTA_AV_EVT event,
                                       tBTA_AV_MEDIA* p_data) {
  LOG_VERBOSE("%s: event=%d", __func__, event);

  switch (event) {
    case BTA_AV_SINK_MEDIA_DATA_EVT: {
      BtifAvPeer* peer = btif_av_sink_find_peer(peer_address);
      if (peer != nullptr && peer->IsActivePeer()) {
        int state = peer->StateMachine().StateId();
        if ((state == BtifAvStateMachine::kStateStarted) ||
            (state == BtifAvStateMachine::kStateOpened)) {
          uint8_t queue_len = btif_a2dp_sink_enqueue_buf((BT_HDR*)p_data);
          LOG_VERBOSE("%s: Packets in Sink queue %d", __func__, queue_len);
        }
      }
      break;
    }
    case BTA_AV_SINK_MEDIA_CFG_EVT: {
      btif_av_sink_config_req_t config_req;

      LOG_VERBOSE("%s: address=%s", __func__,
                  ADDRESS_TO_LOGGABLE_CSTR(p_data->avk_config.bd_addr));

      // Update the codec info of the A2DP Sink decoder
      btif_a2dp_sink_update_decoder((uint8_t*)(p_data->avk_config.codec_info));

      config_req.sample_rate =
          A2DP_GetTrackSampleRate(p_data->avk_config.codec_info);
      if (config_req.sample_rate == -1) {
        LOG_ERROR("%s: Cannot get the track frequency", __func__);
        break;
      }
      config_req.channel_count =
          A2DP_GetTrackChannelCount(p_data->avk_config.codec_info);
      if (config_req.channel_count == -1) {
        LOG_ERROR("%s: Cannot get the channel count", __func__);
        break;
      }
      config_req.peer_address = p_data->avk_config.bd_addr;
      BtifAvEvent btif_av_event(BTIF_AV_SINK_CONFIG_REQ_EVT, &config_req,
                                sizeof(config_req));
      do_in_main_thread(
          FROM_HERE, base::BindOnce(&btif_av_handle_event,
                                    AVDT_TSEP_SRC,  // peer_sep
                                    config_req.peer_address, kBtaHandleUnknown,
                                    btif_av_event));
      break;
    }
    default:
      break;
  }
}

// Initializes the AV interface for source mode
static bt_status_t init_src(
    btav_source_callbacks_t* callbacks, int max_connected_audio_devices,
    const std::vector<btav_a2dp_codec_config_t>& codec_priorities,
    const std::vector<btav_a2dp_codec_config_t>& offloading_preference,
    std::vector<btav_a2dp_codec_info_t>* supported_codecs) {
  LOG_VERBOSE("%s", __func__);
  return btif_av_source.Init(callbacks, max_connected_audio_devices,
                             codec_priorities, offloading_preference,
                             supported_codecs);
}

// Initializes the AV interface for sink mode
static bt_status_t init_sink(btav_sink_callbacks_t* callbacks,
                             int max_connected_audio_devices) {
  LOG_VERBOSE("%s", __func__);
  return btif_av_sink.Init(callbacks, max_connected_audio_devices);
}

// Updates the final focus state reported by components calling this module
static void update_audio_focus_state(int state) {
  LOG_VERBOSE("%s: state=%d", __func__, state);
  btif_a2dp_sink_set_focus_state_req((btif_a2dp_sink_focus_state_t)state);
}

// Updates the track gain (used for ducking).
static void update_audio_track_gain(float gain) {
  LOG_VERBOSE("%s: gain=%f", __func__, gain);
  btif_a2dp_sink_set_audio_track_gain(gain);
}

// Establishes the AV signalling channel with the remote headset
static bt_status_t connect_int(RawAddress* peer_address, uint16_t uuid) {
  LOG_VERBOSE("%s: peer_address=%s uuid=0x%x", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(*peer_address), uuid);
  if (btif_av_both_enable()) {
    const RawAddress tmp = *peer_address;
    if (uuid == UUID_SERVCLASS_AUDIO_SOURCE) {
      btif_av_source_dispatch_sm_event(tmp, BTIF_AV_CONNECT_REQ_EVT);
    } else if (uuid == UUID_SERVCLASS_AUDIO_SINK) {
      btif_av_sink_dispatch_sm_event(tmp, BTIF_AV_CONNECT_REQ_EVT);
    }
    return BT_STATUS_SUCCESS;
  }

  auto connection_task = [](RawAddress* peer_address, uint16_t uuid) {
    BtifAvPeer* peer = nullptr;
    if (uuid == UUID_SERVCLASS_AUDIO_SOURCE) {
      peer = btif_av_source.FindOrCreatePeer(*peer_address, kBtaHandleUnknown);
    } else if (uuid == UUID_SERVCLASS_AUDIO_SINK) {
      peer = btif_av_sink.FindOrCreatePeer(*peer_address, kBtaHandleUnknown);
    }
    if (peer == nullptr) {
      btif_queue_advance();
      return;
    }
    peer->StateMachine().ProcessEvent(BTIF_AV_CONNECT_REQ_EVT, nullptr);
  };
  bt_status_t status = do_in_main_thread(
      FROM_HERE, base::BindOnce(connection_task, peer_address, uuid));
  if (status != BT_STATUS_SUCCESS) {
    LOG(ERROR) << __func__ << ": can't post connection task to main_thread";
  }
  return status;
}

static void set_source_silence_peer_int(const RawAddress& peer_address,
                                        bool silence) {
  LOG_VERBOSE("%s: peer_address=%s, silence=%s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address),
              silence ? "true" : "false");
  if (!btif_av_source.SetSilencePeer(peer_address, silence)) {
    LOG_ERROR("%s: Error setting silence state to %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  }
}

// Set the active peer
static void set_active_peer_int(uint8_t peer_sep,
                                const RawAddress& peer_address,
                                std::promise<void> peer_ready_promise) {
  LOG_VERBOSE("%s: peer_sep=%s (%d) peer_address=%s", __func__,
              (peer_sep == AVDT_TSEP_SRC) ? "Source" : "Sink", peer_sep,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  BtifAvPeer* peer = nullptr;
  if (peer_sep == AVDT_TSEP_SNK) {
    if (!btif_av_src_sink_coexist_enabled() || (btif_av_src_sink_coexist_enabled() &&
      btif_av_both_enable() && (btif_av_sink.FindPeer(peer_address) == nullptr))) {
      btif_av_source.SetActivePeer(peer_address,
                                      std::move(peer_ready_promise));
      LOG_ERROR("%s: Error setting %s as active Sink peer", __func__,
                ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    }
    return;
  }
  if (peer_sep == AVDT_TSEP_SRC) {
    if (!btif_av_src_sink_coexist_enabled() || (btif_av_src_sink_coexist_enabled() &&
      btif_av_both_enable() && (btif_av_source.FindPeer(peer_address) == nullptr))) {
      if (!btif_av_sink.SetActivePeer(peer_address,
                                      std::move(peer_ready_promise))) {
        LOG_ERROR("%s: Error setting %s as active Source peer", __func__,
                  ADDRESS_TO_LOGGABLE_CSTR(peer_address));
      }
    }
    return;
  }
  // If reached here, we could not set the active peer
  LOG_ERROR("%s: Cannot set active %s peer to %s: peer not %s", __func__,
            (peer_sep == AVDT_TSEP_SRC) ? "Source" : "Sink",
            ADDRESS_TO_LOGGABLE_CSTR(peer_address),
            (peer == nullptr) ? "found" : "connected");
  peer_ready_promise.set_value();
}

static bt_status_t src_connect_sink(const RawAddress& peer_address) {
  if (!btif_av_source.Enabled()) {
    LOG_WARN("BTIF AV Source is not enabled");
    return BT_STATUS_NOT_READY;
  }

  RawAddress peer_address_copy(peer_address);
  LOG_DEBUG("Connecting to AV sink peer:%s",
            ADDRESS_TO_LOGGABLE_CSTR(peer_address_copy));

  return btif_queue_connect(UUID_SERVCLASS_AUDIO_SOURCE, &peer_address_copy,
                            connect_int);
}

static bt_status_t sink_connect_src(const RawAddress& peer_address) {
  LOG_INFO("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!btif_av_sink.Enabled()) {
    LOG_WARN("%s: BTIF AV Sink is not enabled", __func__);
    return BT_STATUS_NOT_READY;
  }

  RawAddress peer_address_copy(peer_address);
  return btif_queue_connect(UUID_SERVCLASS_AUDIO_SINK, &peer_address_copy,
                            connect_int);
}

static bt_status_t src_disconnect_sink(const RawAddress& peer_address) {
  LOG_INFO("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!btif_av_source.Enabled()) {
    LOG_WARN("%s: BTIF AV Source is not enabled", __func__);
    return BT_STATUS_NOT_READY;
  }

  BtifAvEvent btif_av_event(BTIF_AV_DISCONNECT_REQ_EVT, &peer_address,
                            sizeof(peer_address));
  return do_in_main_thread(
      FROM_HERE,
      base::BindOnce(&btif_av_handle_event,
                     AVDT_TSEP_SNK,  // peer_sep
                     peer_address, kBtaHandleUnknown, btif_av_event));
}

static bt_status_t sink_disconnect_src(const RawAddress& peer_address) {
  LOG_INFO("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!btif_av_sink.Enabled()) {
    LOG_WARN("%s: BTIF AV Sink is not enabled", __func__);
    return BT_STATUS_NOT_READY;
  }

  BtifAvEvent btif_av_event(BTIF_AV_DISCONNECT_REQ_EVT, &peer_address,
                            sizeof(peer_address));
  return do_in_main_thread(
      FROM_HERE,
      base::BindOnce(&btif_av_handle_event,
                     AVDT_TSEP_SRC,  // peer_sep
                     peer_address, kBtaHandleUnknown, btif_av_event));
}

static bt_status_t sink_set_active_device(const RawAddress& peer_address) {
  LOG_VERBOSE("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!btif_av_sink.Enabled()) {
    LOG(WARNING) << __func__ << ": BTIF AV Source is not enabled";
    return BT_STATUS_NOT_READY;
  }

  std::promise<void> peer_ready_promise;
  std::future<void> peer_ready_future = peer_ready_promise.get_future();
  bt_status_t status = do_in_main_thread(
      FROM_HERE, base::BindOnce(&set_active_peer_int,
                                AVDT_TSEP_SRC,  // peer_sep
                                peer_address, std::move(peer_ready_promise)));
  if (status == BT_STATUS_SUCCESS) {
    peer_ready_future.wait();
  } else {
    LOG(WARNING) << __func__ << ": BTIF AV Sink fails to change peer";
  }
  return status;
}

static bt_status_t src_set_silence_sink(const RawAddress& peer_address,
                                        bool silence) {
  LOG_VERBOSE("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  if (!btif_av_source.Enabled()) {
    LOG_WARN("%s: BTIF AV Source is not enabled", __func__);
    return BT_STATUS_NOT_READY;
  }

  return do_in_main_thread(
      FROM_HERE,
      base::BindOnce(&set_source_silence_peer_int, peer_address, silence));
}

static bt_status_t src_set_active_sink(const RawAddress& peer_address) {
  LOG_VERBOSE("%s: Peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!btif_av_source.Enabled()) {
    LOG(WARNING) << __func__ << ": BTIF AV Source is not enabled";
    return BT_STATUS_NOT_READY;
  }

  std::promise<void> peer_ready_promise;
  std::future<void> peer_ready_future = peer_ready_promise.get_future();
  bt_status_t status = do_in_main_thread(
      FROM_HERE, base::BindOnce(&set_active_peer_int,
                                AVDT_TSEP_SNK,  // peer_sep
                                peer_address, std::move(peer_ready_promise)));
  if (status == BT_STATUS_SUCCESS) {
    peer_ready_future.wait();
  } else {
    LOG(WARNING) << __func__ << ": BTIF AV Source fails to change peer";
  }
  return status;
}

static bt_status_t codec_config_src(
    const RawAddress& peer_address,
    std::vector<btav_a2dp_codec_config_t> codec_preferences) {
  LOG_VERBOSE("%s", __func__);

  if (!btif_av_source.Enabled()) {
    LOG(WARNING) << __func__ << ": BTIF AV Source is not enabled";
    return BT_STATUS_NOT_READY;
  }

  if (peer_address.IsEmpty()) {
    LOG(WARNING) << __func__ << ": BTIF AV Source needs peer to config";
    return BT_STATUS_PARM_INVALID;
  }

  std::promise<void> peer_ready_promise;
  std::future<void> peer_ready_future = peer_ready_promise.get_future();
  bt_status_t status = do_in_main_thread(
      FROM_HERE,
      base::BindOnce(&BtifAvSource::UpdateCodecConfig,
                     base::Unretained(&btif_av_source), peer_address,
                     codec_preferences, std::move(peer_ready_promise)));
  if (status == BT_STATUS_SUCCESS) {
    peer_ready_future.wait();
  } else {
    LOG(WARNING) << __func__ << ": BTIF AV Source fails to config codec";
  }
  return status;
}

static void set_stream_mode(bool isGamingEnabled, bool isLowLatency) {
  LOG(INFO) << __func__ << "isGamingEnabled: " << isGamingEnabled
                        << "isLowLatency: " << isLowLatency;

  if (isGamingEnabled || isLowLatency) {
    btif_av_update_codec_mode(true);
  }
}

static void cleanup_src(void) {
  LOG_VERBOSE("%s", __func__);
  do_in_main_thread(FROM_HERE,
                    base::BindOnce(&BtifAvSource::Cleanup,
                                   base::Unretained(&btif_av_source)));
}

static void cleanup_sink(void) {
  LOG_VERBOSE("%s", __func__);
  do_in_main_thread(FROM_HERE, base::BindOnce(&BtifAvSink::Cleanup,
                                              base::Unretained(&btif_av_sink)));
}

static const btav_source_interface_t bt_av_src_interface = {
    sizeof(btav_source_interface_t),
    init_src,
    src_connect_sink,
    src_disconnect_sink,
    src_set_silence_sink,
    src_set_active_sink,
    codec_config_src,
    set_stream_mode,
    cleanup_src,
};

static const btav_sink_interface_t bt_av_sink_interface = {
    sizeof(btav_sink_interface_t),
    init_sink,
    sink_connect_src,
    sink_disconnect_src,
    cleanup_sink,
    update_audio_focus_state,
    update_audio_track_gain,
    sink_set_active_device};

RawAddress btif_av_source_active_peer(void) {
  return btif_av_source.ActivePeer();
}
RawAddress btif_av_sink_active_peer(void) { return btif_av_sink.ActivePeer(); }

bool btif_av_is_sink_enabled(void) { return btif_av_sink.Enabled(); }

bool btif_av_is_source_enabled(void) { return btif_av_source.Enabled(); }

void btif_av_stream_start(void) {
  LOG_INFO("%s", __func__);
  btif_av_source_dispatch_sm_event(btif_av_source_active_peer(),
                                   BTIF_AV_START_STREAM_REQ_EVT);
}

void btif_av_stream_start_with_latency(bool use_latency_mode) {
  LOG_INFO("%s", __func__);

  btif_av_start_stream_req_t start_stream_req;
  start_stream_req.use_latency_mode = use_latency_mode;
  BtifAvEvent btif_av_event(BTIF_AV_START_STREAM_REQ_EVT, &start_stream_req,
                            sizeof(start_stream_req));
  LOG_INFO("peer_address=%s event=%s use_latency_mode=%s",
           ADDRESS_TO_LOGGABLE_CSTR(btif_av_source_active_peer()),
           btif_av_event.ToString().c_str(),
           use_latency_mode ? "true" : "false");

  do_in_main_thread(
      FROM_HERE, base::BindOnce(&btif_av_handle_event,
                                AVDT_TSEP_SNK,  // peer_sep
                                btif_av_source_active_peer(), kBtaHandleUnknown,
                                btif_av_event));
}

void src_do_suspend_in_main_thread(btif_av_sm_event_t event) {
  if (event != BTIF_AV_SUSPEND_STREAM_REQ_EVT &&
      event != BTIF_AV_STOP_STREAM_REQ_EVT)
    return;
  auto src_do_stream_suspend = [](btif_av_sm_event_t event) {
    bool is_idle = true;
    for (auto it : btif_av_source.Peers()) {
      const BtifAvPeer* peer = it.second;
      if (peer->StateMachine().StateId() == BtifAvStateMachine::kStateStarted) {
        btif_av_source_dispatch_sm_event(peer->PeerAddress(), event);
        is_idle = false;
      }
    }
    if (is_idle) {
      btif_a2dp_on_stopped(nullptr);
    }
  };
  // switch to main thread to prevent a race condition of accessing peers
  do_in_main_thread(FROM_HERE, base::BindOnce(src_do_stream_suspend, event));
}

void btif_av_stream_stop(const RawAddress& peer_address) {
  LOG_INFO("%s peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));

  if (!peer_address.IsEmpty()) {
    btif_av_source_dispatch_sm_event(peer_address, BTIF_AV_STOP_STREAM_REQ_EVT);
    return;
  }

  // The active peer might have changed and we might be in the process
  // of reconfiguring the stream. We need to stop the appropriate peer(s).
  src_do_suspend_in_main_thread(BTIF_AV_STOP_STREAM_REQ_EVT);
}

void btif_av_stream_suspend(void) {
  LOG_INFO("%s", __func__);
  // The active peer might have changed and we might be in the process
  // of reconfiguring the stream. We need to suspend the appropriate peer(s).
  src_do_suspend_in_main_thread(BTIF_AV_SUSPEND_STREAM_REQ_EVT);
}

void btif_av_stream_start_offload(void) {
  LOG_INFO("%s", __func__);
  btif_av_source_dispatch_sm_event(btif_av_source_active_peer(),
                                   BTIF_AV_OFFLOAD_START_REQ_EVT);
}

void btif_av_src_disconnect_sink(const RawAddress& peer_address) {
  LOG_INFO("%s: peer %s", __func__, ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  src_disconnect_sink(peer_address);
}

bool btif_av_stream_ready(void) {
  // Make sure the main adapter is enabled
  if (btif_is_enabled() == 0) {
    LOG_VERBOSE("%s: Main adapter is not enabled", __func__);
    return false;
  }

  BtifAvPeer* peer = btif_av_find_active_peer();
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  int state = peer->StateMachine().StateId();
  LOG_INFO("%s: Peer %s : state=%d, flags=%s", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), state,
           peer->FlagsToString().c_str());
  // check if we are remotely suspended or stop is pending
  if (peer->CheckFlags(BtifAvPeer::kFlagRemoteSuspend |
                       BtifAvPeer::kFlagPendingStop)) {
    return false;
  }

  return (state == BtifAvStateMachine::kStateOpened);
}

bool btif_av_stream_started_ready(void) {
  BtifAvPeer* peer = btif_av_find_active_peer();
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  int state = peer->StateMachine().StateId();
  bool ready = false;
  if (peer->CheckFlags(BtifAvPeer::kFlagLocalSuspendPending |
                       BtifAvPeer::kFlagRemoteSuspend |
                       BtifAvPeer::kFlagPendingStop)) {
    // Disallow media task to start if we have pending actions
    ready = false;
  } else {
    ready = (state == BtifAvStateMachine::kStateStarted);
  }
  LOG_INFO("%s: Peer %s : state=%d flags=%s ready=%d", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()), state,
           peer->FlagsToString().c_str(), ready);

  return ready;
}

static void btif_av_source_dispatch_sm_event(const RawAddress& peer_address,
                                             btif_av_sm_event_t event) {
  BtifAvEvent btif_av_event(event, nullptr, 0);
  LOG_VERBOSE("%s: peer_address=%s event=%s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address),
              btif_av_event.ToString().c_str());

  do_in_main_thread(FROM_HERE, base::BindOnce(&btif_av_handle_event,
                                              AVDT_TSEP_SNK,  // peer_sep
                                              peer_address, kBtaHandleUnknown,
                                              btif_av_event));
}

static void btif_av_sink_dispatch_sm_event(const RawAddress& peer_address,
                                           btif_av_sm_event_t event) {
  BtifAvEvent btif_av_event(event, nullptr, 0);
  LOG_VERBOSE("%s: peer_address=%s event=%s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address),
              btif_av_event.ToString().c_str());

  do_in_main_thread(FROM_HERE, base::BindOnce(&btif_av_handle_event,
                                              AVDT_TSEP_SRC,  // peer_sep
                                              peer_address, kBtaHandleUnknown,
                                              btif_av_event));
}

bt_status_t btif_av_source_execute_service(bool enable) {
  LOG_VERBOSE("%s: Source service: %s", __func__,
              (enable) ? "enable" : "disable");

  if (enable) {
    // Added BTA_AV_FEAT_NO_SCO_SSPD - this ensures that the BTA does not
    // auto-suspend av streaming on AG events(SCO or Call). The suspend shall
    // be initiated by the app/audioflinger layers.
    // Support for browsing for SDP record should work only if we enable
    // BROWSE while registering.
    tBTA_AV_FEAT features = BTA_AV_FEAT_RCTG | BTA_AV_FEAT_METADATA |
                            BTA_AV_FEAT_VENDOR | BTA_AV_FEAT_NO_SCO_SSPD;

    if (delay_reporting_enabled()) {
      features |= BTA_AV_FEAT_DELAY_RPT;
    }

    if (avrcp_absolute_volume_is_enabled()) {
      features |= BTA_AV_FEAT_RCCT | BTA_AV_FEAT_ADV_CTRL | BTA_AV_FEAT_BROWSE;
    }

    if (btif_av_src_sink_coexist_enabled()) {
      features |= BTA_AV_FEAT_SRC;
      BTA_AvEnable(features, bta_av_event_callback);
    } else {
      BTA_AvEnable(features, bta_av_source_callback);
    }
    btif_av_source.RegisterAllBtaHandles();
    return BT_STATUS_SUCCESS;
  }

  // Disable the service
  btif_av_source.DeregisterAllBtaHandles();
  BTA_AvDisable();
  return BT_STATUS_SUCCESS;
}

bt_status_t btif_av_sink_execute_service(bool enable) {
  LOG_VERBOSE("%s: Sink service: %s", __func__,
              (enable) ? "enable" : "disable");

  if (enable) {
    // Added BTA_AV_FEAT_NO_SCO_SSPD - this ensures that the BTA does not
    // auto-suspend AV streaming on AG events (SCO or Call). The suspend shall
    // be initiated by the app/audioflinger layers.
    tBTA_AV_FEAT features = BTA_AV_FEAT_NO_SCO_SSPD | BTA_AV_FEAT_RCCT |
                            BTA_AV_FEAT_METADATA | BTA_AV_FEAT_VENDOR |
                            BTA_AV_FEAT_ADV_CTRL | BTA_AV_FEAT_RCTG |
                            BTA_AV_FEAT_BROWSE | BTA_AV_FEAT_COVER_ARTWORK;

    if (delay_reporting_enabled()) {
      features |= BTA_AV_FEAT_DELAY_RPT;
    }

    if (btif_av_src_sink_coexist_enabled())
      BTA_AvEnable(features, bta_av_event_callback);
    else
      BTA_AvEnable(features, bta_av_sink_callback);
    btif_av_sink.RegisterAllBtaHandles();
    return BT_STATUS_SUCCESS;
  }

  // Disable the service
  btif_av_sink.DeregisterAllBtaHandles();
  BTA_AvDisable();
  return BT_STATUS_SUCCESS;
}

// Get the AV callback interface for A2DP source profile
const btav_source_interface_t* btif_av_get_src_interface(void) {
  LOG_VERBOSE("%s", __func__);
  return &bt_av_src_interface;
}

// Get the AV callback interface for A2DP sink profile
const btav_sink_interface_t* btif_av_get_sink_interface(void) {
  LOG_VERBOSE("%s", __func__);
  return &bt_av_sink_interface;
}

bool btif_av_is_connected(void) {
  BtifAvPeer* peer = btif_av_find_active_peer();
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  bool connected = peer->IsConnected();
  LOG_VERBOSE("%s: Peer %s is %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
              (connected) ? "connected" : "not connected");
  return connected;
}

uint8_t btif_av_get_peer_sep(void) {
  BtifAvPeer* peer = btif_av_find_active_peer();
  if (peer == nullptr) {
    LOG_INFO("No active sink or source peer found");
    return AVDT_TSEP_INVALID;
  }

  uint8_t peer_sep = peer->PeerSep();
  LOG_VERBOSE("Peer %s SEP is %s (%d)",
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
              (peer_sep == AVDT_TSEP_SRC) ? "Source" : "Sink", peer_sep);
  return peer_sep;
}

void btif_av_clear_remote_suspend_flag(void) {
  auto clear_remote_suspend_flag = []() {
    BtifAvPeer* peer = btif_av_find_active_peer();
    if (peer == nullptr) {
      LOG_WARN("%s: No active peer found", __func__);
      return;
    }
    LOG_VERBOSE("%s: Peer %s : flags=%s are cleared", __func__,
                ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
                peer->FlagsToString().c_str());
    peer->ClearFlags(BtifAvPeer::kFlagRemoteSuspend);
  };
  // switch to main thread to prevent a race condition of accessing peers
  do_in_main_thread(FROM_HERE, base::BindOnce(clear_remote_suspend_flag));
}

bool btif_av_is_peer_edr(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No peer found for peer_address=%s", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return false;
  }
  if (!peer->IsConnected()) {
    LOG_WARN("%s: Peer %s is not connected", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return false;
  }

  bool is_edr = peer->IsEdr();
  LOG_VERBOSE("%s: Peer %s : is_edr=%d", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address), is_edr);
  return is_edr;
}

bool btif_av_peer_supports_3mbps(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No peer found for peer_address=%s", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return false;
  }

  bool is3mbps = peer->Is3Mbps();
  bool is_connected = peer->IsConnected();
  LOG_VERBOSE("%s: Peer %s : connected=%d, edr_3mbps=%d", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer_address), is_connected, is3mbps);
  return (is_connected && is3mbps);
}

bool btif_av_peer_prefers_mandatory_codec(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No peer found for peer_address=%s", __func__,
             ADDRESS_TO_LOGGABLE_CSTR(peer_address));
    return false;
  }
  return peer->IsMandatoryCodecPreferred();
}

void btif_av_acl_disconnected(const RawAddress& peer_address) {
  // Inform the application that ACL is disconnected and move to idle state
  LOG_INFO("%s: Peer %s : ACL Disconnected", __func__,
           ADDRESS_TO_LOGGABLE_CSTR(peer_address));
  if (btif_av_both_enable()) {
    BtifAvPeer* peer = btif_av_find_peer(peer_address);
    if (peer != nullptr) {
      if (peer->IsSource()) {
        btif_av_sink_dispatch_sm_event(peer_address, BTIF_AV_ACL_DISCONNECTED);
      } else {
        btif_av_source_dispatch_sm_event(peer_address,
                                         BTIF_AV_ACL_DISCONNECTED);
      }
    }
    return;
  }

  if (btif_av_source.Enabled()) {
    btif_av_source_dispatch_sm_event(peer_address, BTIF_AV_ACL_DISCONNECTED);
  } else if (btif_av_sink.Enabled()) {
    btif_av_sink_dispatch_sm_event(peer_address, BTIF_AV_ACL_DISCONNECTED);
  }
}

static void btif_debug_av_peer_dump(int fd, const BtifAvPeer& peer) {
  std::string state_str;
  int state = peer.StateMachine().StateId();
  switch (state) {
    case BtifAvStateMachine::kStateIdle:
      state_str = "Idle";
      break;
    case BtifAvStateMachine::kStateOpening:
      state_str = "Opening";
      break;
    case BtifAvStateMachine::kStateOpened:
      state_str = "Opened";
      break;
    case BtifAvStateMachine::kStateStarted:
      state_str = "Started";
      break;
    case BtifAvStateMachine::kStateClosing:
      state_str = "Closing";
      break;
    default:
      state_str = "Unknown(" + std::to_string(state) + ")";
      break;
  }

  dprintf(fd, "  Peer: %s\n", ADDRESS_TO_LOGGABLE_CSTR(peer.PeerAddress()));
  dprintf(fd, "    Connected: %s\n", peer.IsConnected() ? "true" : "false");
  dprintf(fd, "    Streaming: %s\n", peer.IsStreaming() ? "true" : "false");
  dprintf(fd, "    SEP: %d(%s)\n", peer.PeerSep(),
          (peer.IsSource()) ? "Source" : "Sink");
  dprintf(fd, "    State Machine: %s\n", state_str.c_str());
  dprintf(fd, "    Flags: %s\n", peer.FlagsToString().c_str());
  dprintf(fd, "    OpenOnRcTimer: %s\n",
          alarm_is_scheduled(peer.AvOpenOnRcTimer()) ? "Scheduled"
                                                     : "Not scheduled");
  dprintf(fd, "    BTA Handle: 0x%x\n", peer.BtaHandle());
  dprintf(fd, "    Peer ID: %d\n", peer.PeerId());
  dprintf(fd, "    EDR: %s\n", peer.IsEdr() ? "true" : "false");
  dprintf(fd, "    Support 3Mbps: %s\n", peer.Is3Mbps() ? "true" : "false");
  dprintf(fd, "    Self Initiated Connection: %s\n",
          peer.SelfInitiatedConnection() ? "true" : "false");
  dprintf(fd, "    Delay Reporting: %u (in 1/10 milliseconds) \n",
          peer.GetDelayReport());
  dprintf(fd, "    Codec Preferred: %s\n",
          peer.IsMandatoryCodecPreferred() ? "Mandatory" : "Optional");
}

static void btif_debug_av_source_dump(int fd) {
  bool enabled = btif_av_source.Enabled();

  dprintf(fd, "\nA2DP Source State: %s\n", (enabled) ? "Enabled" : "Disabled");
  if (!enabled) return;
  dprintf(fd, "  Active peer: %s\n",
          ADDRESS_TO_LOGGABLE_CSTR(btif_av_source.ActivePeer()));
  for (auto it : btif_av_source.Peers()) {
    const BtifAvPeer* peer = it.second;
    if (peer != nullptr) {
      btif_debug_av_peer_dump(fd, *peer);
    }
  }
}

static void btif_debug_av_sink_dump(int fd) {
  bool enabled = btif_av_sink.Enabled();

  dprintf(fd, "\nA2DP Sink State: %s\n", (enabled) ? "Enabled" : "Disabled");
  if (!enabled) return;
  dprintf(fd, "  Active peer: %s\n",
          ADDRESS_TO_LOGGABLE_CSTR(btif_av_sink.ActivePeer()));
  dprintf(fd, "  Peers:\n");
  for (auto it : btif_av_sink.Peers()) {
    const BtifAvPeer* peer = it.second;
    btif_debug_av_peer_dump(fd, *peer);
  }
}

void btif_debug_av_dump(int fd) {
  btif_debug_av_source_dump(fd);
  btif_debug_av_sink_dump(fd);
}

void btif_av_set_audio_delay(const RawAddress& peer_address, uint16_t delay) {
  btif_a2dp_control_set_audio_delay(delay);
  BtifAvPeer* peer = btif_av_find_peer(peer_address);
  if (peer != nullptr && peer->IsSink()) {
    peer->SetDelayReport(delay);
    if (peer->IsActivePeer()) {
      bluetooth::audio::a2dp::set_remote_delay(peer->GetDelayReport());
    }
  }
}

uint16_t btif_av_get_audio_delay() {
  BtifAvPeer* peer = btif_av_find_active_peer();
  if (peer != nullptr && peer->IsSink()) {
    return peer->GetDelayReport();
  }
  return 0;
}

void btif_av_reset_audio_delay(void) { btif_a2dp_control_reset_audio_delay(); }

bool btif_av_is_a2dp_offload_enabled() {
  return btif_av_source.A2dpOffloadEnabled();
}

bool btif_av_is_a2dp_offload_running() {
  if (!btif_av_is_a2dp_offload_enabled()) {
    return false;
  }
  if (!bluetooth::audio::a2dp::is_hal_enabled()) {
    return false;
  }
  return bluetooth::audio::a2dp::is_hal_offloading();
}

bool btif_av_is_dual_mode_enabled() {
  return bluetooth::os::GetSystemPropertyBool(
          bluetooth::os::kIsDualModeAudioEnabledProperty, false);
}

void btif_av_metadata_update(uint16_t context) {
  LOG_INFO("context: %d", context);
  bool is_src = true;

  if (btif_av_source.Enabled()) {
    do_in_jni_thread(
        FROM_HERE,
        base::BindOnce(btif_av_source.Callbacks()->update_metadata_cb,
                                                  context));
  }
}

bool btif_av_is_peer_silenced(const RawAddress& peer_address) {
  return btif_av_source.IsPeerSilenced(peer_address);
}

void btif_av_set_dynamic_audio_buffer_size(uint8_t dynamic_audio_buffer_size) {
  btif_a2dp_source_set_dynamic_audio_buffer_size(dynamic_audio_buffer_size);
}

void btif_av_set_low_latency(bool is_low_latency) {
  LOG_INFO("is_low_latency: %s", is_low_latency ? "true" : "false");

  btif_av_set_latency_req_t set_latency_req;
  set_latency_req.is_low_latency = is_low_latency;
  BtifAvEvent btif_av_event(BTIF_AV_SET_LATENCY_REQ_EVT, &set_latency_req,
                            sizeof(set_latency_req));
  LOG_INFO("peer_address=%s event=%s",
           ADDRESS_TO_LOGGABLE_CSTR(btif_av_source_active_peer()),
           btif_av_event.ToString().c_str());
  do_in_main_thread(
      FROM_HERE, base::BindOnce(&btif_av_handle_event,
                                AVDT_TSEP_SNK,  // peer_sep
                                btif_av_source_active_peer(), kBtaHandleUnknown,
                                btif_av_event));
}

static void btif_av_sink_delete_active_peer(void) {
  btif_av_sink.DeleteActivePeer();
}

static void btif_av_source_delete_active_peer(void) {
  btif_av_source.DeleteActivePeer();
}

bool btif_av_is_connected_addr(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  bool connected = peer->IsConnected();
  LOG_VERBOSE("%s: Peer %s is %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
              (connected) ? "connected" : "not connected");
  return connected;
}

bool btif_av_peer_is_connected_sink(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_source_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  bool connected = peer->IsConnected();
  LOG_VERBOSE("%s: Peer %s is %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
              (connected) ? "connected" : "not connected");
  return connected;
}

bool btif_av_peer_is_connected_source(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_sink_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  bool connected = peer->IsConnected();
  LOG_VERBOSE("%s: Peer %s is %s", __func__,
              ADDRESS_TO_LOGGABLE_CSTR(peer->PeerAddress()),
              (connected) ? "connected" : "not connected");
  return connected;
}

bool btif_av_peer_is_sink(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_source_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  return true;
}

bool btif_av_peer_is_source(const RawAddress& peer_address) {
  BtifAvPeer* peer = btif_av_sink_find_peer(peer_address);
  if (peer == nullptr) {
    LOG_WARN("%s: No active peer found", __func__);
    return false;
  }

  return true;
}

void btif_av_update_codec_mode(bool is_gaming_latency) {
  btif_av_codec_mode_change_t codec_mode_change;
  A2dpCodecConfig* current_codec = bta_av_get_a2dp_current_codec();
  if (current_codec != nullptr) {
    btav_a2dp_codec_config_t codec_config;
    codec_config = current_codec->getCodecConfig();
    if (codec_config.codec_type == BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_ADAPTIVE) {
      if (is_gaming_latency) {
        LOG_INFO(" Is game/low latency, going for Low Latency Mode");
        codec_mode_change.enc_mode = APTX_LL;
      } else {
        LOG_INFO(" Isn't game/low latency, going for High Quality Mode");
        codec_mode_change.enc_mode = APTX_HQ;
      }
      BtifAvEvent btif_av_event(BTIF_AV_SET_CODEC_MODE_EVT, &codec_mode_change,
                                sizeof(codec_mode_change));
      LOG_INFO("peer_address=%s event=%s",
                ADDRESS_TO_LOGGABLE_CSTR(btif_av_source_active_peer()),
                btif_av_event.ToString().c_str());
      do_in_main_thread(FROM_HERE, base::Bind(&btif_av_handle_event,
                                              AVDT_TSEP_SNK,  // peer_sep
                                              btif_av_source_active_peer(),
                                              kBtaHandleUnknown, btif_av_event));
    }
  }
}

void btif_av_update_source_metadata(bool is_Gaming_Enabled) {
  LOG_INFO("btif_av_update_source_metadata");

  btif_av_update_codec_mode(is_Gaming_Enabled);
}

void btif_av_set_low_latency_spatial_audio(bool is_low_latency) {
  LOG_INFO("is_low_latency: %s", is_low_latency ? "true" : "false");

  btif_av_update_codec_mode(is_low_latency);
}
