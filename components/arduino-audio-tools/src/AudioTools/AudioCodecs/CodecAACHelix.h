#pragma once

#include "AudioTools/AudioCodecs/AudioCodecsBase.h"
#ifndef HELIX_PRINT
#define HELIX_PRINT
#endif
#include "AACDecoderHelix.h"

namespace audio_tools {

/**
 * @brief AAC Decoder using libhelix:
 * https://github.com/pschatzmann/arduino-libhelix This is basically just a
 * simple wrapper to provide AudioInfo and AudioInfoSupport
 * @ingroup codecs
 * @ingroup decoder
 * @author Phil Schatzmann
 * @copyright GPLv3
 * modified by MLX on 2025-10-23, added support of bitrate(this file use bitRate).
 */
class AACDecoderHelix : public AudioDecoder {
 public:
  AACDecoderHelix() {
    TRACED();
    aac = new libhelix::AACDecoderHelix();
    if (aac != nullptr) {
      aac->setReference(this);
    } else {
      LOGE("Not enough memory for libhelix");
    }
  }
  /**
   * @brief Construct a new AACDecoderMini object
   *
   * @param out_stream
   */
  AACDecoderHelix(Print &out_stream) {
    TRACED();
    aac = new libhelix::AACDecoderHelix(out_stream);
    if (aac != nullptr) {
      aac->setReference(this);
    } else {
      LOGE("Not enough memory for libhelix");
    }
  }

  /**
   * @brief Construct a new AACDecoderMini object. The decoded output will go to
   * the print object.
   *
   * @param out_stream
   * @param bi
   */
  AACDecoderHelix(Print &out_stream, AudioInfoSupport &bi) {
    TRACED();
    aac = new libhelix::AACDecoderHelix(out_stream);
    if (aac != nullptr) {
      aac->setReference(this);
    } else {
      LOGE("Not enough memory for libhelix");
    }
    addNotifyAudioChange(bi);
  }

  /**
   * @brief Destroy the AACDecoderMini object
   *
   */
  ~AACDecoderHelix() {
    TRACED();
    if (aac != nullptr) delete aac;
  }

  // void setRaw(bool flag){
  //     if (aac!=nullptr) aac->setRaw(flag);
  // }

  /// Defines the output Stream
  virtual void setOutput(Print &out_stream) override {
    TRACED();
    AudioDecoder::setOutput(out_stream);
    if (aac != nullptr) aac->setOutput(out_stream);
  }

  /// Starts the processing - 新增帧长度设置
  bool begin() override {
    TRACED();
    if (aac != nullptr) {
      // aac->setDelay(CODEC_DELAY_MS);
      aac->setInfoCallback(infoCallback, this);
      aac->begin();
      // 配置AAC特定参数
      // aac->setMaxFrameSize(4096); // 设置合适的帧长度(没有效果)
    }
    return true;
  }

  /// Releases the reserved memory
  virtual void end() override {
    TRACED();
    if (aac != nullptr) aac->end();
  }

  virtual _AACFrameInfo audioInfoEx() { return aac->audioInfo(); }

  /// 重写audioInfo，新增比特率（为了方便在AudioInfo中直接获取）
  AudioInfo audioInfo() override {
    AudioInfo result; 
    auto i = audioInfoEx();
    if (i.nChans != 0 && i.bitsPerSample != 0 && i.sampRateOut != 0) {
      result.channels = i.nChans;
      result.sample_rate = i.sampRateOut;
      result.bits_per_sample = i.bitsPerSample;
      result.bit_rate = i.bitRate; // 新增存储比特率（需要确保在aacdec.h的_AACFrameInfo结构体有定义，同时AudioTypes.h添加bit_rate变量的定义）
    }
    return result;
  }

  void setAudioInfo(AudioInfo info) override {
    this->info = info;
    if (info_notifications_active) {
      notifyAudioChange(info);
    }
  }

  /// Write AAC data to decoder
  size_t write(const uint8_t *data, size_t len) override {
    LOGD("AACDecoderHelix::write: %d", (int)len);
    if (aac == nullptr) return 0;
    int open = len;
    int processed = 0;
    uint8_t *data8 = (uint8_t *)data;
    while (open > 0) {
      int act_write =
          aac->write(data8 + processed, min(open, DEFAULT_BUFFER_SIZE));
      open -= act_write;
      processed += act_write;
    }
    return processed;
  }

  /// checks if the class is active
  virtual operator bool() override { return aac != nullptr && (bool)*aac; }

  void flush() {
    //     aac->flush();
  }

  /// notifies the subscriber about a change
  static void infoCallback(_AACFrameInfo &i, void *ref) {
    AACDecoderHelix *p_helix = (AACDecoderHelix *)ref;
    if (p_helix != nullptr) {
      TRACED();
      AudioInfo baseInfo;
      baseInfo.channels = i.nChans;
      baseInfo.sample_rate = i.sampRateOut;
      baseInfo.bits_per_sample = i.bitsPerSample;
      baseInfo.bit_rate = i.bitRate;  // 新增传递比特率（需要确保在aacdec.h的_AACFrameInfo结构体有定义，同时AudioTypes.h添加bit_Rate变量的定义）
      // p_helix->audioChangeAACHelix->setAudioInfo(baseInfo);
      LOGW("sample_rate: %d", i.sampRateOut);
      p_helix->setAudioInfo(baseInfo);
    }
  }

  /// Provides the maximum frame size - this is allocated on the heap and you
  /// can reduce the heap size my minimizing this value
  size_t maxFrameSize() { return aac->maxFrameSize(); }

  /// Define your optimized maximum frame size
  void setMaxFrameSize(size_t len) { aac->setMaxFrameSize(len); }

  void setAudioInfoNotifications(bool active) {
    info_notifications_active = active;
  }

  /// Provides the maximum pwm buffer size - this is allocated on the heap and
  /// you can reduce the heap size my minimizing this value
  size_t maxPCMSize() { return aac->maxPCMSize(); }

  /// Define your optimized maximum pwm buffer size
  void setMaxPCMSize(size_t len) { aac->setMaxPCMSize(len); }

 protected:
  libhelix::AACDecoderHelix *aac = nullptr;
  bool info_notifications_active = true;
};

}  // namespace audio_tools
