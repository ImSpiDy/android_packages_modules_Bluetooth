/*
 * Copyright (C) 2020, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */
/*
 * Copyright 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.

    * Redistribution and use in source and binary forms, with or without
      modification, are permitted (subject to the limitations in the
      disclaimer below) provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#include "a2dp_encoding.h"
#include "client_interface.h"

#include "a2dp_sbc_constants.h"
#include "a2dp_vendor_ldac_constants.h"

#include "a2dp_aac.h"
#include "bta/av/bta_av_int.h"
#include "btif/include/btif_a2dp_source.h"
#include "btif/include/btif_av.h"
#include "btif/include/btif_av_co.h"
#include "btif/include/btif_hf.h"
#include "osi/include/properties.h"
#include "a2dp_sbc.h"
#include <a2dp_vendor.h>
#include "types/raw_address.h"
#include "a2dp_vendor_aptx_adaptive.h"
#include "stack/include/btm_client_interface.h"
#include "stack/include/btm_vendor_types.h"

extern void btif_a2dp_source_encoder_init(void);

#define AAC_SAMPLE_SIZE  1024
#define AAC_LATM_HEADER  12
#define A2DP_HEADER_SIZE 1
namespace {

using vendor::qti::hardware::bluetooth_audio::V2_0::AacObjectType;
using vendor::qti::hardware::bluetooth_audio::V2_0::AacVariableBitRate;
using CodecType =
      vendor::qti::hardware::bluetooth_audio::V2_0::CodecType;
using CodecType_2_1 =
      vendor::qti::hardware::bluetooth_audio::V2_1::CodecType;
using vendor::qti::hardware::bluetooth_audio::V2_0::LdacChannelMode;
using vendor::qti::hardware::bluetooth_audio::V2_0::LdacQualityIndex;
using vendor::qti::hardware::bluetooth_audio::V2_0::SbcAllocMethod;
using vendor::qti::hardware::bluetooth_audio::V2_0::SbcBlockLength;
using vendor::qti::hardware::bluetooth_audio::V2_0::SbcChannelMode;
using vendor::qti::hardware::bluetooth_audio::V2_0::SbcNumSubbands;
using vendor::qti::hardware::bluetooth_audio::V2_0::AptxAdaptiveChannelMode;
using vendor::qti::hardware::bluetooth_audio::V2_0::AptxMode;
using vendor::qti::hardware::bluetooth_audio::V2_0::SessionParams;
using vendor::qti::hardware::bluetooth_audio::V2_0::SessionParamType;
using vendor::qti::hardware::bluetooth_audio::V2_0::SinkLatency;
using vendor::qti::hardware::bluetooth_audio::V2_0::InputMode;
using AudioConfiguration = vendor::qti::hardware::bluetooth_audio::V2_0::AudioConfiguration;
using AudioConfiguration_2_1 = vendor::qti::hardware::bluetooth_audio::V2_1::AudioConfiguration;
using CodecConfiguration =
      vendor::qti::hardware::bluetooth_audio::V2_0::CodecConfiguration;
using CodecConfiguration_2_1 =
      vendor::qti::hardware::bluetooth_audio::V2_1::CodecConfiguration;
using vendor::qti::hardware::bluetooth_audio::V2_1::ExtSampleRate;
using vendor::qti::hardware::bluetooth_audio::V2_0::BitsPerSample;
using bluetooth::audio::qti_hidl::BluetoothAudioCtrlAck;
using vendor::qti::hardware::bluetooth_audio::V2_0::ChannelMode;
using vendor::qti::hardware::bluetooth_audio::V2_0::PcmParameters;
using vendor::qti::hardware::bluetooth_audio::V2_0::SampleRate;
using vendor::qti::hardware::bluetooth_audio::V2_0::SessionType;

std::mutex internal_mutex_;
std::condition_variable ack_wait_cv;
tA2DP_CTRL_ACK ack_status;



BluetoothAudioCtrlAck a2dp_ack_to_bt_audio_ctrl_ack(tA2DP_CTRL_ACK ack);

// Provide call-in APIs for the Bluetooth Audio HAL
class A2dpTransport : public ::bluetooth::audio::qti_hidl::IBluetoothTransportInstance {
 public:
  A2dpTransport(SessionType sessionType, AudioConfiguration audioConfig)
      : IBluetoothTransportInstance(sessionType, std::move(audioConfig)),
        a2dp_pending_cmd_(A2DP_CTRL_CMD_NONE),
        remote_delay_report_(0),
        total_bytes_read_(0),
        data_position_({}){};

  BluetoothAudioCtrlAck StartRequest() override {
    // Check if a previous request is not finished
    if (a2dp_pending_cmd_ == A2DP_CTRL_CMD_START) {
      LOG(INFO) << __func__ << ": A2DP_CTRL_CMD_START in progress";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
    } else if (a2dp_pending_cmd_ != A2DP_CTRL_CMD_NONE) {
      LOG(WARNING) << __func__ << ": busy in pending_cmd=" << a2dp_pending_cmd_;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
    }

    // Don't send START request to stack while we are in a call
    if (!bluetooth::headset::IsCallIdle()) {
      LOG(ERROR) << __func__ << ": call state is busy";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_INCALL_FAILURE);
    }

    if (btif_av_stream_started_ready(A2dpType::kSource)) {
      // Already started, ACK back immediately.
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
    }
    if (btif_av_stream_ready(A2dpType::kSource)) {
      /*
       * Post start event and wait for audio path to open.
       * If we are the source, the ACK will be sent after the start
       * procedure is completed, othewise send it now.
       */
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_START;
      btif_av_stream_start(A2dpType::kSource);
      if (btif_av_get_peer_sep(A2dpType::kSource) != AVDT_TSEP_SRC) {
        LOG(INFO) << __func__ << ": accepted";
        return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
      }
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
    }
    LOG(ERROR) << __func__ << ": AV stream is not ready to start";
    return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
  }

  BluetoothAudioCtrlAck SuspendRequest() override {
    // Previous request is not finished
    if (a2dp_pending_cmd_ == A2DP_CTRL_CMD_SUSPEND) {
      LOG(INFO) << __func__ << ": A2DP_CTRL_CMD_SUSPEND in progress";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
    } else if (a2dp_pending_cmd_ != A2DP_CTRL_CMD_NONE) {
      LOG(WARNING) << __func__ << ": busy in pending_cmd=" << a2dp_pending_cmd_;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
    }
    // Local suspend
    if (btif_av_stream_started_ready(A2dpType::kSource)) {
      LOG(INFO) << __func__ << ": accepted";
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_SUSPEND;
      btif_av_stream_suspend();
      return BluetoothAudioCtrlAck::PENDING;
    }
    /* If we are not in started state, just ack back ok and let
     * audioflinger close the channel. This can happen if we are
     * remotely suspended, clear REMOTE SUSPEND flag.
     */
    btif_av_clear_remote_suspend_flag(A2dpType::kSource);
    return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
  }

  void StopRequest() override {
    if (btif_av_get_peer_sep(A2dpType::kSource) == AVDT_TSEP_SNK &&
        !btif_av_stream_started_ready(A2dpType::kSource)) {
      btif_av_clear_remote_suspend_flag(A2dpType::kSource);
      return;
    }
    LOG(INFO) << __func__ << ": handling";
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_STOP;
    btif_av_stream_stop(RawAddress::kEmpty);
  }

  bool GetPresentationPosition(uint64_t* remote_delay_report_ns,
                               uint64_t* total_bytes_read,
                               timespec* data_position) override {
    *remote_delay_report_ns = remote_delay_report_ * 100000u;
    *total_bytes_read = total_bytes_read_;
    *data_position = data_position_;
    VLOG(2) << __func__ << ": delay=" << remote_delay_report_
            << "/10ms, data=" << total_bytes_read_
            << " byte(s), timestamp=" << data_position_.tv_sec << "."
            << data_position_.tv_nsec << "s";
    return true;
  }

  tA2DP_CTRL_CMD GetPendingCmd() const { return a2dp_pending_cmd_; }

  void ResetPendingCmd() {
    LOG(WARNING) << "ResetPendingCmd  " << a2dp_pending_cmd_;
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
  }

  void ResetPresentationPosition() override {
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
  }

  void LogBytesRead(size_t bytes_read) override {
    if (bytes_read != 0) {
      total_bytes_read_ += bytes_read;
      clock_gettime(CLOCK_MONOTONIC, &data_position_);
    }
  }

  // delay reports from AVDTP is based on 1/10 ms (100us)
  void SetRemoteDelay(uint16_t delay_report) {
    remote_delay_report_ = delay_report;
  }

  void Init(SessionType sessionType, AudioConfiguration audioConfig) {
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
    UpdateSessionType(sessionType);
    UpdateAudioConfiguration(audioConfig);
  }

  void Cleanup() {
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
    SessionType sessionType = SessionType::UNKNOWN;
    UpdateSessionType(sessionType);
  }

 private:
  tA2DP_CTRL_CMD a2dp_pending_cmd_;
  uint16_t remote_delay_report_;
  uint64_t total_bytes_read_;
  timespec data_position_;

};

class A2dpTransport_2_1 : public ::bluetooth::audio::qti_hidl::IBluetoothTransportInstance_2_1 {
 public:
  A2dpTransport_2_1(SessionType sessionType, AudioConfiguration_2_1 audioConfig)
      : IBluetoothTransportInstance_2_1(sessionType, std::move(audioConfig)),
        a2dp_pending_cmd_(A2DP_CTRL_CMD_NONE),
        remote_delay_report_(0),
        total_bytes_read_(0),
        data_position_({}){};

  BluetoothAudioCtrlAck StartRequest() override {
    // Check if a previous request is not finished
    if (a2dp_pending_cmd_ == A2DP_CTRL_CMD_START) {
      LOG(INFO) << __func__ << ": A2DP_CTRL_CMD_START in progress";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
    } else if (a2dp_pending_cmd_ != A2DP_CTRL_CMD_NONE) {
      LOG(WARNING) << __func__ << ": busy in pending_cmd=" << a2dp_pending_cmd_;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
    }

    // Don't send START request to stack while we are in a call
    if (!bluetooth::headset::IsCallIdle()) {
      LOG(ERROR) << __func__ << ": call state is busy";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_INCALL_FAILURE);
    }

    if (btif_av_stream_started_ready(A2dpType::kSource)) {
      // Already started, ACK back immediately.
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
    }
    if (btif_av_stream_ready(A2dpType::kSource)) {
      /*
       * Post start event and wait for audio path to open.
       * If we are the source, the ACK will be sent after the start
       * procedure is completed, othewise send it now.
       */
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_START;
      btif_av_stream_start(A2dpType::kSource);
      if (btif_av_get_peer_sep(A2dpType::kSource) != AVDT_TSEP_SRC) {
        LOG(INFO) << __func__ << ": accepted";
        return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
      }
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
    }
    LOG(ERROR) << __func__ << ": AV stream is not ready to start";
    return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
  }

  BluetoothAudioCtrlAck SuspendRequest() override {
    // Previous request is not finished
    if (a2dp_pending_cmd_ == A2DP_CTRL_CMD_SUSPEND) {
      LOG(INFO) << __func__ << ": A2DP_CTRL_CMD_SUSPEND in progress";
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_PENDING);
    } else if (a2dp_pending_cmd_ != A2DP_CTRL_CMD_NONE) {
      LOG(WARNING) << __func__ << ": busy in pending_cmd=" << a2dp_pending_cmd_;
      return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_FAILURE);
    }
    // Local suspend
    if (btif_av_stream_started_ready(A2dpType::kSource)) {
      LOG(INFO) << __func__ << ": accepted";
      a2dp_pending_cmd_ = A2DP_CTRL_CMD_SUSPEND;
      btif_av_stream_suspend();
      return BluetoothAudioCtrlAck::PENDING;
    }
    /* If we are not in started state, just ack back ok and let
     * audioflinger close the channel. This can happen if we are
     * remotely suspended, clear REMOTE SUSPEND flag.
     */
    btif_av_clear_remote_suspend_flag(A2dpType::kSource);
    return a2dp_ack_to_bt_audio_ctrl_ack(A2DP_CTRL_ACK_SUCCESS);
  }

  void StopRequest() override {
    if (btif_av_get_peer_sep(A2dpType::kSource) == AVDT_TSEP_SNK &&
        !btif_av_stream_started_ready(A2dpType::kSource)) {
      btif_av_clear_remote_suspend_flag(A2dpType::kSource);
      return;
    }
    LOG(INFO) << __func__ << ": handling";
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_STOP;
    btif_av_stream_stop(RawAddress::kEmpty);
  }

  bool GetPresentationPosition(uint64_t* remote_delay_report,
                               uint64_t* total_bytes_read,
                               timespec* data_position) override {
    *remote_delay_report = remote_delay_report_;
    *total_bytes_read = total_bytes_read_;
    *data_position = data_position_;
    VLOG(2) << __func__ << ": delay=" << remote_delay_report_
            << "/10ms, data=" << total_bytes_read_
            << " byte(s), timestamp=" << data_position_.tv_sec << "."
            << data_position_.tv_nsec << "s";
    return true;
  }

  tA2DP_CTRL_CMD GetPendingCmd() const { return a2dp_pending_cmd_; }

  void ResetPendingCmd() {
    LOG(WARNING) << "ResetPendingCmd  " << a2dp_pending_cmd_;
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
 }

  void ResetPresentationPosition() override {
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
  }

  void LogBytesRead(size_t bytes_read) override {
    if (bytes_read != 0) {
      total_bytes_read_ += bytes_read;
      clock_gettime(CLOCK_MONOTONIC, &data_position_);
    }
  }

  // delay reports from AVDTP is based on 1/10 ms (100us)
  void SetRemoteDelay(uint16_t delay_report) {
    remote_delay_report_ = delay_report;
  }

  // TODO to review below functions
  void Init(SessionType sessionType, AudioConfiguration_2_1 audioConfig) {
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
    UpdateSessionType(sessionType);
    UpdateAudioConfiguration(audioConfig);
  }

  void Cleanup() {
    a2dp_pending_cmd_ = A2DP_CTRL_CMD_NONE;
    remote_delay_report_ = 0;
    total_bytes_read_ = 0;
    data_position_ = {};
    SessionType sessionType = SessionType::UNKNOWN;
    UpdateSessionType(sessionType);
  }

 private:
  tA2DP_CTRL_CMD a2dp_pending_cmd_;
  uint16_t remote_delay_report_;
  uint64_t total_bytes_read_;
  timespec data_position_;

};

A2dpTransport* a2dp_sink = nullptr;
A2dpTransport_2_1* a2dp_sink_2_1 = nullptr;
bluetooth::common::MessageLoopThread* death_handler_thread = nullptr;

// Common interface to call-out into Bluetooth Audio HAL
bluetooth::audio::qti_hidl::BluetoothAudioClientInterface* a2dp_hal_clientif = nullptr;
auto session_type = SessionType::UNKNOWN;
btav_a2dp_codec_index_t sw_codec_type = BTAV_A2DP_CODEC_INDEX_SOURCE_MIN;
uint16_t session_peer_mtu = 0;

// Save the value if the remote reports its delay before a2dp_sink is
// initialized
uint16_t remote_delay = 0;

bool is_session_started = false;
bool btaudio_a2dp_supported = false;
bool is_configured = false;
bool is_playing = false;
bool is_hal_version_fetched = false;
bool hal_2_1_enabled = false;
bool hal_2_0_enabled = false;

BluetoothAudioCtrlAck a2dp_ack_to_bt_audio_ctrl_ack(tA2DP_CTRL_ACK ack) {
  switch (ack) {
    case A2DP_CTRL_ACK_SUCCESS:
      return BluetoothAudioCtrlAck::SUCCESS_FINISHED;
    case A2DP_CTRL_ACK_PENDING:
      return BluetoothAudioCtrlAck::PENDING;
    case A2DP_CTRL_ACK_INCALL_FAILURE:
      return BluetoothAudioCtrlAck::FAILURE_BUSY;
    case A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS:
      return BluetoothAudioCtrlAck::FAILURE_DISCONNECTING;
    case A2DP_CTRL_ACK_UNSUPPORTED: /* Offloading but resource failure */
      return BluetoothAudioCtrlAck::FAILURE_UNSUPPORTED;
    case A2DP_CTRL_ACK_FAILURE:
      return BluetoothAudioCtrlAck::FAILURE;
    default:
      return BluetoothAudioCtrlAck::FAILURE;
  }
}

SampleRate a2dp_codec_to_hal_sample_rate(
    const btav_a2dp_codec_config_t& a2dp_codec_config) {
  switch (a2dp_codec_config.sample_rate) {
    case BTAV_A2DP_CODEC_SAMPLE_RATE_44100:
      return SampleRate::RATE_44100;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_48000:
      return SampleRate::RATE_48000;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_88200:
      return SampleRate::RATE_88200;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_96000:
      return SampleRate::RATE_96000;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_176400:
      return SampleRate::RATE_176400;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_192000:
      return SampleRate::RATE_192000;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_16000:
      return SampleRate::RATE_16000;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_24000:
      return SampleRate::RATE_24000;
    default:
      return SampleRate::RATE_UNKNOWN;
  }
}

BitsPerSample a2dp_codec_to_hal_bits_per_sample(
    const btav_a2dp_codec_config_t& a2dp_codec_config) {
  switch (a2dp_codec_config.bits_per_sample) {
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16:
      return BitsPerSample::BITS_16;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_24:
      return BitsPerSample::BITS_24;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_32:
      return BitsPerSample::BITS_32;
    default:
      return BitsPerSample::BITS_UNKNOWN;
  }
}

ChannelMode a2dp_codec_to_hal_channel_mode(
    const btav_a2dp_codec_config_t& a2dp_codec_config) {
  switch (a2dp_codec_config.channel_mode) {
    case BTAV_A2DP_CODEC_CHANNEL_MODE_MONO:
      return ChannelMode::MONO;
    case BTAV_A2DP_CODEC_CHANNEL_MODE_STEREO:
      return ChannelMode::STEREO;
    default:
      return ChannelMode::UNKNOWN;
  }
}

LdacQualityIndex a2dp_codec_to_hal_ldac_quality_index (
    const btav_a2dp_codec_config_t& a2dp_codec_config) {
  switch (a2dp_codec_config.codec_specific_1) {
    case 1000:
      return LdacQualityIndex::QUALITY_HIGH;
    case 1001:
      return LdacQualityIndex::QUALITY_MID;
    case 1002:
      return LdacQualityIndex::QUALITY_LOW;
    case 1003:
      return LdacQualityIndex::QUALITY_ABR;
    default:
      return LdacQualityIndex::QUALITY_ABR;
  }
}

static bool isScramblingEnabled() {
  uint8_t num_supported_freqs = 0;
  uint8_t *supported_frequencies =
      get_btm_client_interface().vendor.BTM_GetScramblingSupportedFreqs(&num_supported_freqs);
  if (num_supported_freqs == 0 || supported_frequencies == NULL) {
    LOG(WARNING) << __func__ << ":scrambling unsupported";
    return false;
  }

  A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
  if (a2dp_config == nullptr) {
    LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
    return false;
  }

  btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
  for (uint8_t i = 0; i < num_supported_freqs; i++) {
    if (supported_frequencies[i] == (uint8_t)current_codec.sample_rate) {
      LOG(WARNING) << __func__ << ":scrambling supported";
      return true;
    }
  }

  LOG(WARNING) << __func__ << ":scrambling unsupported";
  return false;
}

bool a2dp_get_selected_hal_codec_config(CodecConfiguration* codec_config) {
  A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
  uint8_t p_codec_info[AVDT_CODEC_SIZE];
  uint8_t codec_type;
  uint32_t bitrate = 0;
  tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
  if (codec_config == nullptr) return false;

  if (a2dp_config == nullptr) {
    LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
    return false;
  }
  btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
  tBT_A2DP_OFFLOAD a2dp_offload;
  a2dp_config->getCodecSpecificConfig(&a2dp_offload);
  memset(p_codec_info, 0, AVDT_CODEC_SIZE);
  if (!a2dp_config->copyOutOtaCodecConfig(p_codec_info))
  {
    LOG(ERROR) << "No valid codec config";
    return false;
  }

  // fill the scrambling support flag
  codec_config->isScramblingEnabled = isScramblingEnabled();

  switch (current_codec.codec_type) {
    case BTAV_A2DP_CODEC_INDEX_SOURCE_SBC:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC: {
      codec_config->codecType = CodecType::SBC;
      codec_config->config.sbcConfig = {};
      auto sbc_config = codec_config->config.sbcConfig;
      sbc_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (sbc_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__
                   << ": Unknown SBC sample_rate=" << current_codec.sample_rate;
        return false;
      }
      uint8_t channel_mode = a2dp_offload.codec_info[3] & A2DP_SBC_IE_CH_MD_MSK;
      switch (channel_mode) {
        case A2DP_SBC_IE_CH_MD_JOINT:
          sbc_config.channelMode = SbcChannelMode::JOINT_STEREO;
          break;
        case A2DP_SBC_IE_CH_MD_STEREO:
          sbc_config.channelMode = SbcChannelMode::STEREO;
          break;
        case A2DP_SBC_IE_CH_MD_DUAL:
          sbc_config.channelMode = SbcChannelMode::DUAL;
          break;
        case A2DP_SBC_IE_CH_MD_MONO:
          sbc_config.channelMode = SbcChannelMode::MONO;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC channel_mode=" << channel_mode;
          sbc_config.channelMode = SbcChannelMode::UNKNOWN;
          return false;
      }
      uint8_t block_length =
          a2dp_offload.codec_info[0] & A2DP_SBC_IE_BLOCKS_MSK;
      switch (block_length) {
        case A2DP_SBC_IE_BLOCKS_4:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_4;
          break;
        case A2DP_SBC_IE_BLOCKS_8:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_8;
          break;
        case A2DP_SBC_IE_BLOCKS_12:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_12;
          break;
        case A2DP_SBC_IE_BLOCKS_16:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_16;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC block_length=" << block_length;
          return false;
      }
      uint8_t sub_bands = a2dp_offload.codec_info[0] & A2DP_SBC_IE_SUBBAND_MSK;
      switch (sub_bands) {
        case A2DP_SBC_IE_SUBBAND_4:
          sbc_config.numSubbands = SbcNumSubbands::SUBBAND_4;
          break;
        case A2DP_SBC_IE_SUBBAND_8:
          sbc_config.numSubbands = SbcNumSubbands::SUBBAND_8;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown SBC Subbands=" << sub_bands;
          return false;
      }
      uint8_t alloc_method =
          a2dp_offload.codec_info[0] & A2DP_SBC_IE_ALLOC_MD_MSK;
      switch (alloc_method) {
        case A2DP_SBC_IE_ALLOC_MD_S:
          sbc_config.allocMethod = SbcAllocMethod::ALLOC_MD_S;
          break;
        case A2DP_SBC_IE_ALLOC_MD_L:
          sbc_config.allocMethod = SbcAllocMethod::ALLOC_MD_L;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC alloc_method=" << alloc_method;
          return false;
      }
      sbc_config.minBitpool = a2dp_offload.codec_info[1];
      sbc_config.maxBitpool = a2dp_offload.codec_info[2];
      sbc_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (sbc_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown SBC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.sbcConfig = sbc_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_AAC:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC: {
      codec_config->codecType = CodecType::AAC;
      codec_config->config.aacConfig = {};
      auto aac_config = codec_config->config.aacConfig;
      // TODO(cheneyni): add more supported types.
      aac_config.objectType = AacObjectType::MPEG2_LC;
      aac_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aac_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__
                   << ": Unknown AAC sample_rate=" << current_codec.sample_rate;
        return false;
      }
      aac_config.channelMode = a2dp_codec_to_hal_channel_mode(current_codec);
      if (aac_config.channelMode == ChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown AAC channel_mode="
                   << current_codec.channel_mode;
        return false;
      }
      uint8_t vbr_enabled =
              a2dp_offload.codec_info[1] & A2DP_AAC_VARIABLE_BIT_RATE_MASK;
      switch (vbr_enabled) {
        case A2DP_AAC_VARIABLE_BIT_RATE_ENABLED:
          LOG(INFO) << __func__ << " Enabled AAC VBR =" << +vbr_enabled;
          aac_config.variableBitRateEnabled = AacVariableBitRate::ENABLED;
          break;
        case A2DP_AAC_VARIABLE_BIT_RATE_DISABLED:
          LOG(INFO) << __func__ << " Disabled AAC VBR =" << +vbr_enabled;
          aac_config.variableBitRateEnabled = AacVariableBitRate::DISABLED;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown AAC VBR=" << +vbr_enabled;
          return false;
      }
      aac_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aac_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown AAC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.aacConfig = aac_config;
      break;
    }
    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_HD: {
      if (current_codec.codec_type == BTAV_A2DP_CODEC_INDEX_SOURCE_APTX) {
        codec_config->codecType = CodecType::APTX;
      } else {
        codec_config->codecType = CodecType::APTX_HD;
      }
      codec_config->config.aptxConfig = {};
      auto aptx_config = codec_config->config.aptxConfig;
      aptx_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aptx_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      aptx_config.channelMode = a2dp_codec_to_hal_channel_mode(current_codec);
      if (aptx_config.channelMode == ChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX channel_mode="
                   << current_codec.channel_mode;
        return false;
      }
      aptx_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aptx_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.aptxConfig = aptx_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_ADAPTIVE: {
      tA2DP_APTX_ADAPTIVE_CIE adaptive_cie;
      codec_config->codecType = CodecType::APTX_ADAPTIVE;
      codec_config->config.aptxAdaptiveConfig = {};
      auto aptx_adaptive_config = codec_config->config.aptxAdaptiveConfig;
      aptx_adaptive_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aptx_adaptive_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      aptx_adaptive_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aptx_adaptive_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX adaptive bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      if(!A2DP_GetAptxAdaptiveCIE(p_codec_info, &adaptive_cie)) {
        LOG(ERROR) << __func__ << ": Unable to get Aptx Adaptive CIE";
        return false;
      }
      aptx_adaptive_config.channelMode = static_cast<AptxAdaptiveChannelMode>
                                        (adaptive_cie.channelMode);
      if (aptx_adaptive_config.channelMode >=
                 AptxAdaptiveChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX adaptive channel_mode="
                   << adaptive_cie.channelMode;
        return false;
      }
      aptx_adaptive_config.aptxMode = static_cast<AptxMode>(0x1000)
                                      /*static_cast<AptxMode> (btif_av_get_aptx_mode_info())*/;
      aptx_adaptive_config.sinkBuffering = { 20, 50, 20, 50, 20, 50 };
      aptx_adaptive_config.ttp = { adaptive_cie.aptx_data.ttp_ll_0, adaptive_cie.aptx_data.ttp_ll_1,
                                   adaptive_cie.aptx_data.ttp_hq_0, adaptive_cie.aptx_data.ttp_hq_1,
                                   adaptive_cie.aptx_data.ttp_tws_0, adaptive_cie.aptx_data.ttp_tws_1
                                 };

      aptx_adaptive_config.inputMode = static_cast<InputMode>(0x00);
      aptx_adaptive_config.inputFadeDuration = 0xff;
      aptx_adaptive_config.aptxAdaptiveConfigStream[0] =
        adaptive_cie.aptx_data.cap_ext_ver_num;
      aptx_adaptive_config.aptxAdaptiveConfigStream[1] =
        adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x000000FF;
      aptx_adaptive_config.aptxAdaptiveConfigStream[2] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x0000FF00) >> 8);
      aptx_adaptive_config.aptxAdaptiveConfigStream[3] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x00FF0000) >> 16);
      aptx_adaptive_config.aptxAdaptiveConfigStream[4] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0xFF000000) >> 24);
      aptx_adaptive_config.aptxAdaptiveConfigStream[5] =
        adaptive_cie.aptx_data.first_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[6] =
        adaptive_cie.aptx_data.second_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[7] =
        adaptive_cie.aptx_data.third_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[8] =
        adaptive_cie.aptx_data.fourth_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[9] =
        adaptive_cie.aptx_data.eoc0;
      aptx_adaptive_config.aptxAdaptiveConfigStream[10] =
        adaptive_cie.aptx_data.eoc1;
      codec_config->config.aptxAdaptiveConfig = aptx_adaptive_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_LDAC: {
      codec_config->codecType = CodecType::LDAC;
      codec_config->config.ldacConfig = {};
      auto ldac_config = codec_config->config.ldacConfig;
      ldac_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (ldac_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      switch (a2dp_offload.codec_info[7]) {
        case A2DP_LDAC_CHANNEL_MODE_STEREO:
          ldac_config.channelMode = LdacChannelMode::STEREO;
          break;
        case A2DP_LDAC_CHANNEL_MODE_DUAL:
          ldac_config.channelMode = LdacChannelMode::DUAL;
          break;
        case A2DP_LDAC_CHANNEL_MODE_MONO:
          ldac_config.channelMode = LdacChannelMode::MONO;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown LDAC channel_mode="
                     << a2dp_offload.codec_info[7];
          ldac_config.channelMode = LdacChannelMode::UNKNOWN;
          return false;
      }
      switch (a2dp_offload.codec_info[6]) {
        case A2DP_LDAC_QUALITY_HIGH:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_HIGH;
          break;
        case A2DP_LDAC_QUALITY_MID:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_MID;
          break;
        case A2DP_LDAC_QUALITY_LOW:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_LOW;
          break;
        case A2DP_LDAC_QUALITY_ABR_OFFLOAD:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_ABR;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown LDAC QualityIndex="
                     << a2dp_offload.codec_info[6];
          return false;
      }
      ldac_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (ldac_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.ldacConfig = ldac_config;
      break;
    }
    case BTAV_A2DP_CODEC_INDEX_MAX:
      [[fallthrough]];
    default:
      LOG(ERROR) << __func__
                 << ": Unknown codec_type=" << current_codec.codec_type;
      codec_config->codecType = CodecType::UNKNOWN;
      codec_config->config = {};
      return false;
  }
  // Obtain the MTU
  RawAddress peer_addr = btif_av_source_active_peer();
  bta_av_co_get_peer_params(peer_addr, &peer_param);
  codec_type = A2DP_GetCodecType((const uint8_t*)p_codec_info);
  LOG(INFO) << __func__ << ": codec_type" << codec_type;
  // Obtain the MTU
  codec_config->peerMtu = peer_param.peer_mtu - A2DP_HEADER_SIZE;
  if (A2DP_MEDIA_CT_SBC == codec_type) {
    codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
  } else if (A2DP_MEDIA_CT_NON_A2DP == codec_type) {
    if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_LDAC_VENDOR_ID) {
      codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
      LOG(INFO) << __func__ << "LDAC bitrate" << codec_config->encodedAudioBitrate;
    }  else if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_APTX_ADAPTIVE_VENDOR_ID) {
      int samplerate = A2DP_GetTrackSampleRate(p_codec_info);
      int bits_per_sample = 16;
      codec_config->encodedAudioBitrate = (samplerate * bits_per_sample * 2)/4;
    } else {
      codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
    }
  } else if (A2DP_MEDIA_CT_AAC == codec_type) {
    uint32_t codec_based_bit_rate = 0;
    codec_based_bit_rate = a2dp_config->getTrackBitRate();
    codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
  }
  LOG(INFO) << __func__ << ": CodecConfiguration=" << toString(*codec_config);
  return true;
}
bool a2dp_get_selected_hal_codec_config_2_1(CodecConfiguration_2_1* codec_config) {
  A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
  uint8_t p_codec_info[AVDT_CODEC_SIZE];
  uint8_t codec_type;
  uint32_t bitrate = 0;
  int cis_count = 2;
  tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
  if (codec_config == nullptr) return false;

  /********LC3 Codec */
  if (a2dp_config == nullptr) {
    LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
    return false;
  }

  btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
  tBT_A2DP_OFFLOAD a2dp_offload;
  a2dp_config->getCodecSpecificConfig(&a2dp_offload);
  memset(p_codec_info, 0, AVDT_CODEC_SIZE);
  if (!a2dp_config->copyOutOtaCodecConfig(p_codec_info))
  {
    LOG(ERROR) << "No valid codec config";
    return false;
  }

  // fill the scrambling support flag
  codec_config->isScramblingEnabled = isScramblingEnabled();

  switch (current_codec.codec_type) {
    case BTAV_A2DP_CODEC_INDEX_SOURCE_SBC:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC: {
      codec_config->codecType = CodecType_2_1::SBC;
      codec_config->config.sbcConfig = {};
      auto sbc_config = codec_config->config.sbcConfig;
      sbc_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (sbc_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__
                   << ": Unknown SBC sample_rate=" << current_codec.sample_rate;
        return false;
      }
      uint8_t channel_mode = a2dp_offload.codec_info[3] & A2DP_SBC_IE_CH_MD_MSK;
      switch (channel_mode) {
        case A2DP_SBC_IE_CH_MD_JOINT:
          sbc_config.channelMode = SbcChannelMode::JOINT_STEREO;
          break;
        case A2DP_SBC_IE_CH_MD_STEREO:
          sbc_config.channelMode = SbcChannelMode::STEREO;
          break;
        case A2DP_SBC_IE_CH_MD_DUAL:
          sbc_config.channelMode = SbcChannelMode::DUAL;
          break;
        case A2DP_SBC_IE_CH_MD_MONO:
          sbc_config.channelMode = SbcChannelMode::MONO;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC channel_mode=" << channel_mode;
          sbc_config.channelMode = SbcChannelMode::UNKNOWN;
          return false;
      }
      uint8_t block_length =
          a2dp_offload.codec_info[0] & A2DP_SBC_IE_BLOCKS_MSK;
      switch (block_length) {
        case A2DP_SBC_IE_BLOCKS_4:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_4;
          break;
        case A2DP_SBC_IE_BLOCKS_8:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_8;
          break;
        case A2DP_SBC_IE_BLOCKS_12:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_12;
          break;
        case A2DP_SBC_IE_BLOCKS_16:
          sbc_config.blockLength = SbcBlockLength::BLOCKS_16;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC block_length=" << block_length;
          return false;
      }
      uint8_t sub_bands = a2dp_offload.codec_info[0] & A2DP_SBC_IE_SUBBAND_MSK;
      switch (sub_bands) {
        case A2DP_SBC_IE_SUBBAND_4:
          sbc_config.numSubbands = SbcNumSubbands::SUBBAND_4;
          break;
        case A2DP_SBC_IE_SUBBAND_8:
          sbc_config.numSubbands = SbcNumSubbands::SUBBAND_8;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown SBC Subbands=" << sub_bands;
          return false;
      }
      uint8_t alloc_method =
          a2dp_offload.codec_info[0] & A2DP_SBC_IE_ALLOC_MD_MSK;
      switch (alloc_method) {
        case A2DP_SBC_IE_ALLOC_MD_S:
          sbc_config.allocMethod = SbcAllocMethod::ALLOC_MD_S;
          break;
        case A2DP_SBC_IE_ALLOC_MD_L:
          sbc_config.allocMethod = SbcAllocMethod::ALLOC_MD_L;
          break;
        default:
          LOG(ERROR) << __func__
                     << ": Unknown SBC alloc_method=" << alloc_method;
          return false;
      }
      sbc_config.minBitpool = a2dp_offload.codec_info[1];
      sbc_config.maxBitpool = a2dp_offload.codec_info[2];
      sbc_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (sbc_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown SBC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.sbcConfig = sbc_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_AAC:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC: {
      codec_config->codecType = CodecType_2_1::AAC;
      codec_config->config.aacConfig = {};
      auto aac_config = codec_config->config.aacConfig;
      // TODO(cheneyni): add more supported types.
      aac_config.objectType = AacObjectType::MPEG2_LC;
      aac_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aac_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__
                   << ": Unknown AAC sample_rate=" << current_codec.sample_rate;
        return false;
      }
      aac_config.channelMode = a2dp_codec_to_hal_channel_mode(current_codec);
      if (aac_config.channelMode == ChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown AAC channel_mode="
                   << current_codec.channel_mode;
        return false;
      }
      uint8_t vbr_enabled =
              a2dp_offload.codec_info[1] & A2DP_AAC_VARIABLE_BIT_RATE_MASK;
      switch (vbr_enabled) {
        case A2DP_AAC_VARIABLE_BIT_RATE_ENABLED:
          LOG(INFO) << __func__ << " Enabled AAC VBR =" << +vbr_enabled;
          aac_config.variableBitRateEnabled = AacVariableBitRate::ENABLED;
          break;
        case A2DP_AAC_VARIABLE_BIT_RATE_DISABLED:
          LOG(INFO) << __func__ << " Disabled AAC VBR =" << +vbr_enabled;
          aac_config.variableBitRateEnabled = AacVariableBitRate::DISABLED;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown AAC VBR=" << +vbr_enabled;
          return false;
      }
      aac_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aac_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown AAC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.aacConfig = aac_config;
      break;
    }
    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX:
      [[fallthrough]];
    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_HD: {
      if (current_codec.codec_type == BTAV_A2DP_CODEC_INDEX_SOURCE_APTX) {
        codec_config->codecType = CodecType_2_1::APTX;
      } else {
        codec_config->codecType = CodecType_2_1::APTX_HD;
      }
      codec_config->config.aptxConfig = {};
      auto aptx_config = codec_config->config.aptxConfig;
      aptx_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aptx_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      aptx_config.channelMode = a2dp_codec_to_hal_channel_mode(current_codec);
      if (aptx_config.channelMode == ChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX channel_mode="
                   << current_codec.channel_mode;
        return false;
      }
      aptx_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aptx_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.aptxConfig = aptx_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_ADAPTIVE: {
      tA2DP_APTX_ADAPTIVE_CIE adaptive_cie;
      codec_config->codecType = CodecType_2_1::APTX_ADAPTIVE;
      codec_config->config.aptxAdaptiveConfig = {};
      auto aptx_adaptive_config = codec_config->config.aptxAdaptiveConfig;
      aptx_adaptive_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (aptx_adaptive_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      aptx_adaptive_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (aptx_adaptive_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX adaptive bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      if(!A2DP_GetAptxAdaptiveCIE(p_codec_info, &adaptive_cie)) {
        LOG(ERROR) << __func__ << ": Unable to get Aptx Adaptive CIE";
        return false;
      }
      aptx_adaptive_config.channelMode = static_cast<AptxAdaptiveChannelMode>
                                        (adaptive_cie.channelMode);
      if (aptx_adaptive_config.channelMode >=
                 AptxAdaptiveChannelMode::UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown aptX adaptive channel_mode="
                   << adaptive_cie.channelMode;
        return false;
      }
      aptx_adaptive_config.aptxMode = static_cast<AptxMode>(0x1000)
                                      /*static_cast<AptxMode> (btif_av_get_aptx_mode_info())*/;
      aptx_adaptive_config.sinkBuffering = { 20, 50, 20, 50, 20, 50 };
      aptx_adaptive_config.ttp = { adaptive_cie.aptx_data.ttp_ll_0, adaptive_cie.aptx_data.ttp_ll_1,
                                   adaptive_cie.aptx_data.ttp_hq_0, adaptive_cie.aptx_data.ttp_hq_1,
                                   adaptive_cie.aptx_data.ttp_tws_0, adaptive_cie.aptx_data.ttp_tws_1
                                 };

      aptx_adaptive_config.inputMode = static_cast<InputMode>(0x00);

      aptx_adaptive_config.inputFadeDuration = 0xff;
      aptx_adaptive_config.aptxAdaptiveConfigStream[0] =
        adaptive_cie.aptx_data.cap_ext_ver_num;
      aptx_adaptive_config.aptxAdaptiveConfigStream[1] =
        adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x000000FF;
      aptx_adaptive_config.aptxAdaptiveConfigStream[2] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x0000FF00) >> 8);
      aptx_adaptive_config.aptxAdaptiveConfigStream[3] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0x00FF0000) >> 16);
      aptx_adaptive_config.aptxAdaptiveConfigStream[4] =
        ((adaptive_cie.aptx_data.aptx_adaptive_sup_features & 0xFF000000) >> 24);
      aptx_adaptive_config.aptxAdaptiveConfigStream[5] =
        adaptive_cie.aptx_data.first_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[6] =
        adaptive_cie.aptx_data.second_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[7] =
        adaptive_cie.aptx_data.third_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[8] =
        adaptive_cie.aptx_data.fourth_setup_pref;
      aptx_adaptive_config.aptxAdaptiveConfigStream[9] =
        adaptive_cie.aptx_data.eoc0;
      aptx_adaptive_config.aptxAdaptiveConfigStream[10] =
        adaptive_cie.aptx_data.eoc1;
      codec_config->config.aptxAdaptiveConfig = aptx_adaptive_config;
      break;
    }

    case BTAV_A2DP_CODEC_INDEX_SOURCE_LDAC: {
      codec_config->codecType = CodecType_2_1::LDAC;
      codec_config->config.ldacConfig = {};
      auto ldac_config = codec_config->config.ldacConfig;
      ldac_config.sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
      if (ldac_config.sampleRate == SampleRate::RATE_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC sample_rate="
                   << current_codec.sample_rate;
        return false;
      }
      switch (a2dp_offload.codec_info[7]) {
        case A2DP_LDAC_CHANNEL_MODE_STEREO:
          ldac_config.channelMode = LdacChannelMode::STEREO;
          break;
        case A2DP_LDAC_CHANNEL_MODE_DUAL:
          ldac_config.channelMode = LdacChannelMode::DUAL;
          break;
        case A2DP_LDAC_CHANNEL_MODE_MONO:
          ldac_config.channelMode = LdacChannelMode::MONO;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown LDAC channel_mode="
                     << a2dp_offload.codec_info[7];
          ldac_config.channelMode = LdacChannelMode::UNKNOWN;
          return false;
      }
      switch (a2dp_offload.codec_info[6]) {
        case A2DP_LDAC_QUALITY_HIGH:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_HIGH;
          break;
        case A2DP_LDAC_QUALITY_MID:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_MID;
          break;
        case A2DP_LDAC_QUALITY_LOW:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_LOW;
          break;
        case A2DP_LDAC_QUALITY_ABR_OFFLOAD:
          ldac_config.qualityIndex = LdacQualityIndex::QUALITY_ABR;
          break;
        default:
          LOG(ERROR) << __func__ << ": Unknown LDAC QualityIndex="
                     << a2dp_offload.codec_info[6];
          return false;
      }
      ldac_config.bitsPerSample =
          a2dp_codec_to_hal_bits_per_sample(current_codec);
      if (ldac_config.bitsPerSample == BitsPerSample::BITS_UNKNOWN) {
        LOG(ERROR) << __func__ << ": Unknown LDAC bits_per_sample="
                   << current_codec.bits_per_sample;
        return false;
      }
      codec_config->config.ldacConfig = ldac_config;
      break;
    }
    case BTAV_A2DP_CODEC_INDEX_MAX:
      [[fallthrough]];
    default:
      LOG(ERROR) << __func__
                 << ": Unknown codec_type=" << current_codec.codec_type;
      codec_config->codecType = CodecType_2_1::UNKNOWN;
      codec_config->config = {};
      return false;
  }
  RawAddress peer_addr = btif_av_source_active_peer();
  bta_av_co_get_peer_params(peer_addr, &peer_param);
  codec_type = A2DP_GetCodecType((const uint8_t*)p_codec_info);
  LOG(INFO) << __func__ << ": codec_type" << codec_type;
  // Obtain the MTU
  codec_config->peerMtu = peer_param.peer_mtu - A2DP_HEADER_SIZE;
  if (A2DP_MEDIA_CT_SBC == codec_type) {
    codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
  }
  else if (A2DP_MEDIA_CT_NON_A2DP == codec_type) {
    if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_LDAC_VENDOR_ID) {
      codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
      LOG(INFO) << __func__ << "LDAC bitrate" << codec_config->encodedAudioBitrate;
    } else if ((A2DP_VendorCodecGetVendorId(p_codec_info)) == A2DP_APTX_ADAPTIVE_VENDOR_ID) {
      int samplerate = A2DP_GetTrackSampleRate(p_codec_info);
      int bits_per_sample = 16;
      codec_config->encodedAudioBitrate = (samplerate * bits_per_sample * 2)/4;
    } else {
      codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
    }
  }
  else if (A2DP_MEDIA_CT_AAC == codec_type) {
    uint32_t codec_based_bit_rate = 0;
    codec_based_bit_rate = a2dp_config->getTrackBitRate();
    codec_config->encodedAudioBitrate = a2dp_config->getTrackBitRate();
  }
  LOG(INFO) << __func__ << ": CodecConfiguration=" << toString(*codec_config);
  return true;
}

bool a2dp_get_selected_hal_pcm_config(PcmParameters* pcm_config) {
  if (pcm_config == nullptr) return false;
  A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
  if (a2dp_config == nullptr) {
    LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
    *pcm_config = ::bluetooth::audio::qti_hidl::BluetoothAudioClientInterface::
        kInvalidPcmConfiguration;
    return false;
  }

  btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
  pcm_config->sampleRate = a2dp_codec_to_hal_sample_rate(current_codec);
  pcm_config->bitsPerSample = a2dp_codec_to_hal_bits_per_sample(current_codec);
  pcm_config->channelMode = a2dp_codec_to_hal_channel_mode(current_codec);
  return (pcm_config->sampleRate != SampleRate::RATE_UNKNOWN &&
          pcm_config->bitsPerSample != BitsPerSample::BITS_UNKNOWN &&
          pcm_config->channelMode != ChannelMode::UNKNOWN);
}
}  // namespace

namespace bluetooth {
namespace audio {
namespace qti_hidl {
namespace a2dp {

// Checking if new bluetooth_audio is enabled
bool is_hal_2_0_enabled() {
  bool is_hal_2_1_enabled = (a2dp_sink_2_1 != nullptr);
  bool is_hal_2_0_enabled = (a2dp_sink != nullptr);
  LOG(WARNING) << __func__ << ": is_hal_2_1_enabled: " << is_hal_2_1_enabled
                           << ": is_hal_2_0_enabled: " << is_hal_2_0_enabled;

  return (is_hal_2_1_enabled || is_hal_2_0_enabled);
}

// Checking if new bluetooth_audio is supported
bool is_hal_2_0_supported() {
  if (!is_configured) {
    btaudio_a2dp_supported =
      osi_property_get_bool(BLUETOOTH_AUDIO_PROP_ENABLED, true);
    is_configured = true;
  }
  return btaudio_a2dp_supported;
}
void get_hal_version() {
  LOG(WARNING) << __func__;
  if (!is_hal_version_fetched) {
    bluetooth::audio::qti_hidl::BluetoothAudioClientInterface* hal_clientif = nullptr;
    IBluetoothTransportInstance_2_1* sink = nullptr;
    hal_clientif = new bluetooth::audio::qti_hidl::BluetoothAudioClientInterface(
     sink, nullptr, nullptr);
    if (!strcmp(hal_clientif->GetHalVersion(),"hal_2_1")) {
      LOG(WARNING) << __func__ << ":hal version 2.1";
      hal_2_1_enabled = true;
      is_hal_version_fetched = true;
    } else if (!strcmp(hal_clientif->GetHalVersion(),"hal_2_0")) {
      LOG(WARNING) << __func__ << ":hal version 2.0";
      hal_2_0_enabled = true;
      is_hal_version_fetched = true;
    }
  }
}

static bool is_qti_hal_enabled = false;

bool is_qc_hal_enabled() {
  LOG(WARNING) << __func__;
  get_hal_version();
  return (hal_2_0_enabled || hal_2_1_enabled);
}

bool is_hal_2_0_offloading() {
   if (!is_hal_2_0_enabled()) {
     return false;
   }

  return ((a2dp_sink_2_1 && a2dp_sink_2_1->GetSessionType() ==
                            SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH ) ||
          (a2dp_sink && a2dp_sink->GetSessionType() ==
                            SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH));
}

// Initialize BluetoothAudio HAL: openProvider
bool init( bluetooth::common::MessageLoopThread* message_loop) {
  LOG(WARNING) << __func__;
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_supported()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not supported";
    return false;
  }

  get_hal_version();
  if (hal_2_1_enabled) {
    AudioConfiguration_2_1 audio_config{};
    if (btif_av_is_a2dp_offload_enabled()) {
      CodecConfiguration_2_1 codec_config{};
      if (!a2dp_get_selected_hal_codec_config_2_1(&codec_config)) {
        LOG(ERROR) << __func__ << ": Failed to get CodecConfiguration";
        return false;
      }
      audio_config.codecConfig = codec_config;
      LOG(WARNING) << __func__ << ":Session type OFFLOAD";
      session_type = SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH;
    } else {
      PcmParameters pcm_config{};
      if (!a2dp_get_selected_hal_pcm_config(&pcm_config)) {
        LOG(ERROR) << __func__ << ": Failed to get PcmConfiguration";
        return false;
      }
      audio_config.pcmConfig = pcm_config;
      session_type = SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH;
    }

    if(a2dp_sink_2_1 == nullptr) {
      LOG(WARNING) << __func__ << "Init A2dpTransport_2_1";
      a2dp_sink_2_1 = new A2dpTransport_2_1(session_type, audio_config);
    } else {
      a2dp_sink_2_1->Init(session_type, audio_config);
    }
    if(a2dp_hal_clientif == nullptr) {
      a2dp_hal_clientif = new bluetooth::audio::qti_hidl::BluetoothAudioClientInterface(
      a2dp_sink_2_1, message_loop, &internal_mutex_);
      death_handler_thread = message_loop;
    } else if(death_handler_thread != message_loop) {
      death_handler_thread = message_loop;
     //update the client interface as well
      LOG(WARNING) << __func__ << ": updating death handler thread "
                   << death_handler_thread;
      a2dp_hal_clientif->UpdateDeathHandlerThread(death_handler_thread);
    }
    if (remote_delay != 0) {
      LOG(INFO) << __func__ << ": restore DELAY "
                << static_cast<float>(remote_delay / 10.0) << " ms";
      a2dp_sink_2_1->SetRemoteDelay(remote_delay);
      remote_delay = 0;
    }
  } else {
    AudioConfiguration audio_config{};
    if (btif_av_is_a2dp_offload_enabled()) {
      CodecConfiguration codec_config{};
      if (!a2dp_get_selected_hal_codec_config(&codec_config)) {
        LOG(ERROR) << __func__ << ": Failed to get CodecConfiguration";
        return false;
      }
      audio_config.codecConfig = codec_config;
      session_type = SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH;
    } else {
       PcmParameters pcm_config{};
      if (!a2dp_get_selected_hal_pcm_config(&pcm_config)) {
        LOG(ERROR) << __func__ << ": Failed to get PcmConfiguration";
        return false;
      }
      audio_config.pcmConfig = pcm_config;
      session_type = SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH;
    }
    if(a2dp_sink == nullptr) {
      a2dp_sink = new A2dpTransport(session_type, audio_config);
    } else {
      a2dp_sink->Init(session_type, audio_config);
    }
    if(a2dp_hal_clientif == nullptr) {
      a2dp_hal_clientif = new bluetooth::audio::qti_hidl::BluetoothAudioClientInterface(
        a2dp_sink, message_loop, &internal_mutex_);
      death_handler_thread = message_loop;
    } else if(death_handler_thread != message_loop) {
      death_handler_thread = message_loop;
      //update the client interface as well
      LOG(WARNING) << __func__ << ": updating death handler thread  "
                   << death_handler_thread;
      a2dp_hal_clientif->UpdateDeathHandlerThread(death_handler_thread);
    }

    if (remote_delay != 0) {
      LOG(INFO) << __func__ << ": restore DELAY "
                << static_cast<float>(remote_delay / 10.0) << " ms";
      a2dp_sink->SetRemoteDelay(remote_delay);
      remote_delay = 0;
    }
  }
  return true;
}

// Clean up BluetoothAudio HAL
void cleanup() {
  LOG(WARNING) << __func__ << ": end_session has been called.";
  end_session();
}


void update_session_params(SessionParamType param_type) {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled() || !is_session_started) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled/started";
    return;
  }
  if (a2dp_sink_2_1) {
    AudioConfiguration_2_1 audio_config = a2dp_sink_2_1->GetAudioConfiguration();
    CodecConfiguration_2_1 *codec_config = &audio_config.codecConfig;
    if(SessionParamType::MTU == param_type) {
      tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
      RawAddress peer_addr = btif_av_source_active_peer();
      bta_av_co_get_peer_params(peer_addr, &peer_param);
      if(session_peer_mtu != peer_param.peer_mtu) {
        LOG(INFO) << __func__ << ": updating peer mtu";
        if(session_type == SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH) {
          codec_config->peerMtu =  peer_param.peer_mtu -
                                               A2DP_HEADER_SIZE;
          a2dp_hal_clientif->UpdateAudioConfig_2_1(audio_config);
          SessionParams session_params{};
          session_params.paramType = SessionParamType::MTU;
          session_params.param.mtu = codec_config->peerMtu;
          a2dp_hal_clientif->updateSessionParams(session_params);
        } else if(session_type == SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH) {
          btif_a2dp_source_encoder_init();
        }
     }
   }
  } else {
    AudioConfiguration audio_config = a2dp_sink->GetAudioConfiguration();
    CodecConfiguration *codec_config = &audio_config.codecConfig;
    if(SessionParamType::MTU == param_type) {
      tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
      RawAddress peer_addr = btif_av_source_active_peer();
      bta_av_co_get_peer_params(peer_addr, &peer_param);
      if(session_peer_mtu != peer_param.peer_mtu) {
        LOG(INFO) << __func__ << ": updating peer mtu";
        if(session_type == SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH) {
          codec_config->peerMtu =  peer_param.peer_mtu -
                                               A2DP_HEADER_SIZE;
          a2dp_hal_clientif->UpdateAudioConfig(audio_config);
          SessionParams session_params{};
          session_params.paramType = SessionParamType::MTU;
          session_params.param.mtu = codec_config->peerMtu;
          a2dp_hal_clientif->updateSessionParams(session_params);
        } else if(session_type == SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH) {
          btif_a2dp_source_encoder_init();
        }
      }
    }
  }
}

// Set up the codec into BluetoothAudio HAL
bool setup_codec() {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  tA2DP_ENCODER_INIT_PEER_PARAMS peer_param;
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return false;
  }

  if (a2dp_sink_2_1) {
     AudioConfiguration_2_1 audio_config{};
     if (btif_av_is_a2dp_offload_enabled()) {
       CodecConfiguration_2_1 codec_config{};
       if (!a2dp_get_selected_hal_codec_config_2_1(&codec_config)) {
         LOG(ERROR) << __func__ << ": Failed to get CodecConfiguration";
         return false;
       }
       session_type = SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH;
       audio_config.codecConfig = codec_config;
     } else {
       PcmParameters pcm_config{};
       if (!a2dp_get_selected_hal_pcm_config(&pcm_config)) {
         LOG(ERROR) << __func__ << ": Failed to get PcmConfiguration";
         return false;
       }
       session_type = SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH;
       A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
       if (a2dp_config == nullptr) {
         LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
         return false;
       }
       btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
       audio_config.pcmConfig = pcm_config;
       sw_codec_type = current_codec.codec_type;
     }

    if(a2dp_sink_2_1 == nullptr) {
      LOG(WARNING) << __func__ << "Init A2dpTransport_2_1";
      a2dp_sink_2_1 = new A2dpTransport_2_1(session_type, audio_config);
    } else {
      a2dp_sink_2_1->Init(session_type, audio_config);
    }

     //store the MTU as well
     RawAddress peer_addr = btif_av_source_active_peer();
     bta_av_co_get_peer_params(peer_addr, &peer_param);
     session_peer_mtu = peer_param.peer_mtu;
     return a2dp_hal_clientif->UpdateAudioConfig_2_1(audio_config);
  } else {
    AudioConfiguration audio_config{};
    if (btif_av_is_a2dp_offload_enabled()) {
      CodecConfiguration codec_config{};
      if (!a2dp_get_selected_hal_codec_config(&codec_config)) {
        LOG(ERROR) << __func__ << ": Failed to get CodecConfiguration";
        return false;
      }
      session_type = SessionType::A2DP_HARDWARE_OFFLOAD_DATAPATH;
      audio_config.codecConfig = codec_config;
    } else {
      PcmParameters pcm_config{};
      if (!a2dp_get_selected_hal_pcm_config(&pcm_config)) {
        LOG(ERROR) << __func__ << ": Failed to get PcmConfiguration";
        return false;
      }
      session_type = SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH;
      A2dpCodecConfig* a2dp_config = bta_av_get_a2dp_current_codec();
      if (a2dp_config == nullptr) {
        LOG(WARNING) << __func__ << ": failure to get A2DP codec config";
        return false;
      }
      btav_a2dp_codec_config_t current_codec = a2dp_config->getCodecConfig();
      audio_config.pcmConfig = pcm_config;
      sw_codec_type = current_codec.codec_type;
    }

    if(a2dp_sink == nullptr) {
      a2dp_sink = new A2dpTransport(session_type, audio_config);
    } else {
      a2dp_sink->Init(session_type, audio_config);
    }
    //store the MTU as well
    RawAddress peer_addr = btif_av_source_active_peer();
    bta_av_co_get_peer_params(peer_addr, &peer_param);
    session_peer_mtu = peer_param.peer_mtu;
    return a2dp_hal_clientif->UpdateAudioConfig(audio_config);
  }
}

void start_session() {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return;
  } else if(is_session_started) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL session is already started";
    return;
  }
  LOG(WARNING) << __func__;
  is_playing = false;
  a2dp_hal_clientif->StartSession();
  is_session_started = true;
}

void end_session() {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return;
  } else if(!is_session_started) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL session is not started";
    return;
  }
  LOG(WARNING) << __func__;
  tA2DP_CTRL_CMD pending_cmd = A2DP_CTRL_CMD_NONE;
  if (a2dp_sink_2_1)
    pending_cmd = a2dp_sink_2_1->GetPendingCmd();
  else
    pending_cmd = a2dp_sink->GetPendingCmd();
  if (pending_cmd == A2DP_CTRL_CMD_START) {
    LOG(INFO) << __func__ << ":honoring pending A2DP_CTRL_CMD_START";
    a2dp_hal_clientif->StreamStarted(a2dp_ack_to_bt_audio_ctrl_ack
                    (A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
  } else if (pending_cmd == A2DP_CTRL_CMD_SUSPEND) {
    LOG(INFO) << __func__ << ":honoring pending A2DP_CTRL_CMD_SUSPEND/STOP";
    a2dp_hal_clientif->StreamSuspended(a2dp_ack_to_bt_audio_ctrl_ack
                    (A2DP_CTRL_ACK_DISCONNECT_IN_PROGRESS));
  }
  if (a2dp_sink_2_1) {
    a2dp_sink_2_1->Cleanup();
  } else {
    a2dp_sink->Cleanup();
  }
  is_playing = false;
  a2dp_hal_clientif->EndSession();
  sw_codec_type = BTAV_A2DP_CODEC_INDEX_SOURCE_MIN;
  session_peer_mtu = 0;
  session_type = SessionType::UNKNOWN;
  is_session_started = false;
  death_handler_thread = nullptr;
  remote_delay = 0;
}

void ack_stream_started(const tA2DP_CTRL_ACK& ack) {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return;
  }
  auto ctrl_ack = a2dp_ack_to_bt_audio_ctrl_ack(ack);
  LOG(INFO) << __func__ << ": result=" << ctrl_ack;
  tA2DP_CTRL_CMD pending_cmd = A2DP_CTRL_CMD_NONE;
  if (a2dp_sink_2_1)
    pending_cmd = a2dp_sink_2_1->GetPendingCmd();
  else
    pending_cmd = a2dp_sink->GetPendingCmd();
  if (pending_cmd == A2DP_CTRL_CMD_START) {
    a2dp_hal_clientif->StreamStarted(ctrl_ack);
  } else {
    LOG(WARNING) << __func__ << ": pending=" << pending_cmd
                 << " ignore result=" << ctrl_ack;
    return;
  }
  if(ctrl_ack == BluetoothAudioCtrlAck::SUCCESS_FINISHED) {
    is_playing = true;
  }
  if (ctrl_ack != BluetoothAudioCtrlAck::PENDING) {
    if (a2dp_sink_2_1)
      a2dp_sink_2_1->ResetPendingCmd();
    else
      a2dp_sink->ResetPendingCmd();
  }
}

void ack_stream_suspended(const tA2DP_CTRL_ACK& ack) {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return;
  }
  auto ctrl_ack = a2dp_ack_to_bt_audio_ctrl_ack(ack);
  LOG(INFO) << __func__ << ": result=" << ctrl_ack;
  tA2DP_CTRL_CMD pending_cmd = A2DP_CTRL_CMD_NONE;
  if (a2dp_sink_2_1)
    pending_cmd = a2dp_sink_2_1->GetPendingCmd();
  else
    pending_cmd = a2dp_sink->GetPendingCmd();
  if (pending_cmd == A2DP_CTRL_CMD_SUSPEND) {
    a2dp_hal_clientif->StreamSuspended(ctrl_ack);
  } else if (pending_cmd == A2DP_CTRL_CMD_STOP) {
    LOG(INFO) << __func__ << ": A2DP_CTRL_CMD_STOP result=" << ctrl_ack;
  } else {
    LOG(WARNING) << __func__ << ": pending=" << pending_cmd
                 << " ignore result=" << ctrl_ack;
    return;
  }
  if(ctrl_ack == BluetoothAudioCtrlAck::SUCCESS_FINISHED) {
    is_playing = false;
  }
  if (ctrl_ack != BluetoothAudioCtrlAck::PENDING) {
    if (a2dp_sink_2_1)
      a2dp_sink_2_1->ResetPendingCmd();
    else
      a2dp_sink->ResetPendingCmd();
  }
}

// Read from the FMQ of BluetoothAudio HAL
size_t read(uint8_t* p_buf, uint32_t len) {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  if (!is_hal_2_0_enabled()) {
    LOG(ERROR) << __func__ << ": BluetoothAudio HAL is not enabled";
    return 0;
  } else if (session_type != SessionType::A2DP_SOFTWARE_ENCODING_DATAPATH) {
    LOG(ERROR) << __func__ << ": session_type=" << toString(session_type)
               << " is not A2DP_SOFTWARE_ENCODING_DATAPATH";
    return 0;
  }
  return a2dp_hal_clientif->ReadAudioData(p_buf, len);
}

// Update A2DP delay report to BluetoothAudio HAL
void set_remote_delay(uint16_t delay_report) {
  std::unique_lock<std::mutex> guard(internal_mutex_);
  SessionParams session_params{};
  if (!is_hal_2_0_enabled()) {
    LOG(INFO) << __func__ << ":  not ready for DelayReport "
              << static_cast<float>(delay_report / 10.0) << " ms";
    remote_delay = delay_report;
    return;
  }
  LOG(INFO) << __func__ << ": DELAY " << static_cast<float>(delay_report / 10.0)
            << " ms";
  session_params.paramType = SessionParamType::SINK_LATENCY;
  session_params.param.sinkLatency = {delay_report, 0x00, {}};
  if (a2dp_sink_2_1)
    a2dp_sink_2_1->SetRemoteDelay(delay_report);
  else
    a2dp_sink->SetRemoteDelay(delay_report);

  if (is_session_started) {
   a2dp_hal_clientif->updateSessionParams(session_params);
  }

}

}  // namespace a2dp
}  // namespace qti_hidl
}  // namespace audio
}  // namespace bluetooth
