#include "dcf77_emitter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <algorithm>

namespace esphome {
namespace dcf77_emitter {

static const char *TAG = "dcf77_emitter";

// -----------------------------------------------------------------------------
// Setup timer using ESP-IDF esp_timer (1 tick = 100 ms)
// -----------------------------------------------------------------------------
void DCF77Emitter::setup_timer_() {
  // create periodic timer
  const esp_timer_create_args_t timer_args = {
      .callback = [](void *arg) {
        auto *self = static_cast<DCF77Emitter *>(arg);
        self->dcf_out_tick();
      },
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "dcf77_tick"};

  esp_timer_create(&timer_args, &this->esp_timer_handle_);
  esp_timer_start_periodic(this->esp_timer_handle_, 100000);  // 100 ms
  ESP_LOGD(TAG, "ESP-IDF timer configured (100 ms period)");
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void DCF77Emitter::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DCF77 Emitter...");
  ESP_LOGI(TAG, "!!!!!!!!!!!!!!!!!!!!!!!!!!tesT!!!!!!!!!!!!!!!!!!!!!!!");
  this->led_pin_->setup();
  this->led_pin_->digital_write(false);
  this->antenna_pin_->setup();

  // Configure LEDC PWM for 77.5 kHz
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 77500,
      .clk_cfg = LEDC_USE_XTAL_CLK};
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
      .gpio_num = this->antenna_pin_->get_pin(),
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0};
  ledc_channel_config(&ledc_channel);

  this->pwm_channel_ = LEDC_CHANNEL_0;

  code_time_();

  this->sync_start_millis_ = millis();
  auto time = this->time_id_->now();
  if (time.is_valid()) {
    this->last_second_ = time.second;
  }

  this->last_tick_time_ = 0;
  this->timing_drift_ms_ = 0;
  this->last_sync_millis_ = millis();

  setup_timer_();

  ESP_LOGI(TAG, "DCF77 Emitter setup complete. Waiting for sync.");
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------
void DCF77Emitter::loop() {
  if (!this->sync_switch_->state) {
    if (this->is_initialized_) {
      ESP_LOGW(TAG, "DCF77 synchronization disabled by switch");
      this->is_initialized_ = false;
      stop_carrier_();
      this->led_pin_->digital_write(false);
    }
    return;
  }

  if (!this->is_initialized_) {
    auto current_time = this->time_id_->now();
    if (!current_time.is_valid()){
      ESP_LOGD(TAG, "time is not valid, leave loop");
      return;
    }
    if (current_time.second != this->last_second_) {
      ESP_LOGI(TAG, "Second transition detected after %u ms",
               millis() - this->sync_start_millis_);

      code_time_();
      this->impulse_count_ = 0;
      this->is_initialized_ = true;
      schedule_next_tick_();

      ESP_LOGI(TAG, "DCF77 synchronization enabled. Starting signal generation");
    } else if (millis() - this->sync_start_millis_ > 5000) {
      ESP_LOGW(TAG, "Second sync timeout - continuing anyway");
      code_time_();
      this->impulse_count_ = 0;
      this->is_initialized_ = true;
      schedule_next_tick_();
    }

    this->last_second_ = current_time.second;
  }

  const uint32_t now = millis();
  if (now - this->last_status_log_ >= 10000) {
    this->last_status_log_ = now;
    auto time = this->time_id_->now();
    if (time.is_valid()) {
      ESP_LOGD(TAG, "DCF77 Status: %s, Time: %02d:%02d:%02d, DST: %s",
               this->is_initialized_ ? "Transmitting" : "Initializing",
               time.hour, time.minute, time.second,
               time.is_dst ? "ON" : "OFF");
    } else {
      ESP_LOGE(TAG, "DCF77 Status: Waiting for valid time source");
    }
  }

  if (this->is_initialized_) {
    static uint32_t last_valid_time = millis();
    auto time = this->time_id_->now();

    if (time.is_valid()) {
      last_valid_time = millis();
    } else if (millis() - last_valid_time > 30000) {
      ESP_LOGE(TAG, "No valid time for 30 seconds - forcing resynchronization");
      this->is_initialized_ = false;
      this->sync_start_millis_ = millis();
    }
  }
}

// -----------------------------------------------------------------------------
// Schedule next tick with drift correction
// -----------------------------------------------------------------------------
void DCF77Emitter::schedule_next_tick_() {
  uint32_t now = millis();

  if (this->last_tick_time_ > 0) {
    uint32_t elapsed = now - this->last_tick_time_;
    int32_t drift = elapsed - 100;
    if (abs(drift) < 50) {
      this->timing_drift_ms_ += drift;
    } else {
      ESP_LOGW(TAG, "Abnormal timing drift detected: %dms", drift);
    }
  }

  this->last_tick_time_ = now;
  int32_t next_interval = 100;

  if (abs(this->timing_drift_ms_) > 5) {
    int32_t correction =
        std::clamp(this->timing_drift_ms_, static_cast<int32_t>(-30),
                   static_cast<int32_t>(30));
    next_interval -= correction;
    this->timing_drift_ms_ -= correction;

    this->consecutive_drift_corrections_++;
    if (this->consecutive_drift_corrections_ % 10 == 0) {
      ESP_LOGD(TAG, "Drift compensation: %dms correction, %dms remaining drift",
               correction, this->timing_drift_ms_);
    }
  } else {
    this->consecutive_drift_corrections_ = 0;
  }

  if ((now - this->last_sync_millis_ > 600000) ||
      (abs(this->timing_drift_ms_) > 100)) {
    ESP_LOGI(TAG, "Performing periodic resynchronization with second boundary");
    this->is_initialized_ = false;
    this->timing_drift_ms_ = 0;
    this->last_sync_millis_ = now;
    this->sync_start_millis_ = now;
    return;
  }

  App.scheduler.set_timeout(this, "dcf77_tick", next_interval, [this]() {
    this->dcf_out_tick();
    if (this->is_initialized_) {
      schedule_next_tick_();
    }
  });
}

// -----------------------------------------------------------------------------
// Tick handler
// -----------------------------------------------------------------------------
void DCF77Emitter::dcf_out_tick() {
  auto current_time = this->time_id_->now();
  if (!current_time.is_valid() || !this->is_initialized_)
    return;

  code_time_();

  int current_sec = current_time.second;

  if (current_sec != this->last_second_) {
    if ((this->last_second_ != -1) &&
        (current_sec != ((this->last_second_ + 1) % 60))) {
      ESP_LOGW(TAG, "Second transition irregular: %d → %d", this->last_second_,
               current_sec);
      this->timing_drift_ms_ = 0;
    }

    this->last_second_ = current_sec;
    this->impulse_count_ = 0;
  }

  generate_signal_(current_sec);
}

// -----------------------------------------------------------------------------
// Generate DCF77 modulation
// -----------------------------------------------------------------------------
void DCF77Emitter::generate_signal_(int current_sec) {
  switch (this->impulse_count_++) {
    case 0:
      if (this->impulse_array_[current_sec] != 0) {
        this->led_pin_->digital_write(false);
        stop_carrier_();
      } else {
        this->led_pin_->digital_write(true);
        setup_carrier_();
      }
      break;

    case 1:
      if (this->impulse_array_[current_sec] == 1) {
        this->led_pin_->digital_write(true);
        setup_carrier_();
      }
      break;

    case 2:
      this->led_pin_->digital_write(true);
      setup_carrier_();
      break;

    case 9:
      this->impulse_count_ = 0;
      if (current_sec == 59) {
        ESP_LOGD(TAG, "DCF77 minute complete. Time: %02d:%02d:%02d",
                 actual_hours_, actual_minutes_, actual_second_);
      }
      break;
    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Carrier control
// -----------------------------------------------------------------------------
void DCF77Emitter::setup_carrier_() {
  if (this->carrier_enabled_)
    return;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel_, 127);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel_);
  this->carrier_enabled_ = true;
}

void DCF77Emitter::stop_carrier_() {
  if (!this->carrier_enabled_)
    return;

  ledc_set_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel_, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel_);
  this->carrier_enabled_ = false;
}

// -----------------------------------------------------------------------------
// Dump config
// -----------------------------------------------------------------------------
void DCF77Emitter::dump_config() {
  ESP_LOGCONFIG(TAG, "DCF77 Emitter:");
  LOG_PIN("  Antenna Pin: ", this->antenna_pin_);
  LOG_PIN("  LED Pin: ", this->led_pin_);
}

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------
int DCF77Emitter::bin2bcd_(int dato) {
  if (dato < 10)
    return dato;
  int msb = (dato / 10) << 4;
  int lsb = dato % 10;
  return msb + lsb;
}

// -----------------------------------------------------------------------------
// Encode time into impulse array (same logic as original)
// -----------------------------------------------------------------------------
void DCF77Emitter::code_time_() {
  auto time = this->time_id_->now();
  if (!time.is_valid())
    return;

  this->day_of_week_ = time.day_of_week == 0 ? 7 : time.day_of_week;
  this->actual_day_ = time.day_of_month;
  this->actual_month_ = time.month;
  this->actual_year_ = time.year % 100;
  this->actual_hours_ = time.hour;
  this->actual_minutes_ = time.minute + 1;
  if (this->actual_minutes_ >= 60) {
    this->actual_minutes_ = 0;
    this->actual_hours_ = (this->actual_hours_ + 1) % 24;
  }
  this->actual_second_ = time.second;

  int n, Tmp, TmpIn, ParityCount = 0;
  for (n = 0; n < 20; n++)
    this->impulse_array_[n] = 1;

  if (!time.is_dst)
    this->impulse_array_[18] = 2;
  else
    this->impulse_array_[17] = 2;

  this->impulse_array_[20] = 2;

  ParityCount = 0;
  TmpIn = bin2bcd_(this->actual_minutes_);
  for (n = 21; n < 28; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  this->impulse_array_[28] = ((ParityCount & 1) == 0) ? 1 : 2;

  ParityCount = 0;
  TmpIn = bin2bcd_(this->actual_hours_);
  for (n = 29; n < 35; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  this->impulse_array_[35] = ((ParityCount & 1) == 0) ? 1 : 2;

  ParityCount = 0;
  TmpIn = bin2bcd_(this->actual_day_);
  for (n = 36; n < 42; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = bin2bcd_(this->day_of_week_);
  for (n = 42; n < 45; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = bin2bcd_(this->actual_month_);
  for (n = 45; n < 50; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  TmpIn = bin2bcd_(this->actual_year_);
  for (n = 50; n < 58; n++) {
    Tmp = TmpIn & 1;
    this->impulse_array_[n] = Tmp + 1;
    ParityCount += Tmp;
    TmpIn >>= 1;
  }
  this->impulse_array_[58] = ((ParityCount & 1) == 0) ? 1 : 2;
  this->impulse_array_[59] = 0;
}

}  // namespace dcf77_emitter
}  // namespace esphome
