#include <Arduino.h>
#include <algorithm>

#include "OC_apps.h"
#include "OC_bitmaps.h"
#include "OC_calibration.h"
#include "OC_config.h"
#include "OC_core.h"
#include "OC_gpio.h"
#include "OC_menus.h"
#include "OC_strings.h"
#include "OC_ui.h"
#include "OC_version.h"
#include "OC_options.h"
#include "src/drivers/display.h"

extern uint_fast8_t MENU_REDRAW;
const uint8_t PROGMEM UNWN_LOGO_BITMAP[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 
    0x00, 0x08, 0x01, 0x80, 0x10, 0x00, 
    0x00, 0x1c, 0x01, 0x80, 0x38, 0x00, 
    0x00, 0x1c, 0x01, 0x80, 0x38, 0x00, 
    0x00, 0x1c, 0x01, 0x80, 0x30, 0x00, 
    0x00, 0x1c, 0x01, 0x80, 0x30, 0x00, 
    0x00, 0x0e, 0x07, 0xe0, 0x70, 0x00, 
    0x00, 0x0f, 0x1f, 0xf8, 0xf0, 0x00, 
    0x00, 0x07, 0xbe, 0x7d, 0xe0, 0x00, 
    0x00, 0x03, 0xf8, 0x1f, 0xc0, 0x00, 
    0x00, 0x01, 0xf0, 0x0f, 0x80, 0x00, 
    0x00, 0x00, 0xf0, 0x07, 0x00, 0x00, 
    0x00, 0x18, 0x60, 0x07, 0x08, 0x00, 
    0x00, 0x38, 0xe0, 0x07, 0x1c, 0x00, 
    0x00, 0x38, 0x60, 0x06, 0x1c, 0x00, 
    0x00, 0x38, 0x70, 0x0e, 0x1c, 0x00, 
    0x00, 0x18, 0x70, 0x0e, 0x18, 0x00, 
    0x00, 0x1c, 0x38, 0x1c, 0x38, 0x00, 
    0x00, 0x1c, 0x7f, 0xfe, 0x38, 0x00, 
    0x00, 0x0e, 0xff, 0xff, 0x70, 0x00, 
    0x00, 0x0f, 0xe3, 0xc7, 0xf0, 0x00, 
    0x00, 0x07, 0xc1, 0x83, 0xe0, 0x00, 
    0x00, 0x03, 0xc1, 0x83, 0xc0, 0x00, 
    0x00, 0x01, 0xe1, 0x87, 0x80, 0x00, 
    0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 
    0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 
    0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

 const uint8_t test_bitmap[8] = {
       0xf0, 0xf0, 0xf0, 0xf0, 0x0f, 0x0f, 0x0f, 0x0f
     };

namespace OC {

Ui ui;

void Ui::Init() {
  ticks_ = 0;
  set_screensaver_timeout(SCREENSAVER_TIMEOUT_S);

  static const int button_pins[] = { but_top, but_bot, butL, butR };
  for (size_t i = 0; i < CONTROL_BUTTON_LAST; ++i) {
    buttons_[i].Init(button_pins[i], OC_GPIO_BUTTON_PINMODE);
  }
  std::fill(button_press_time_, button_press_time_ + 4, 0);
  button_state_ = 0;
  button_ignore_mask_ = 0;
  screensaver_ = false;
  preempt_screensaver_ = false;

  encoder_right_.Init(OC_GPIO_ENC_PINMODE);
  encoder_left_.Init(OC_GPIO_ENC_PINMODE);

  event_queue_.Init();
}

void Ui::configure_encoders(EncoderConfig encoder_config) {
  SERIAL_PRINTLN("Configuring encoders: %s (%x)", OC::Strings::encoder_config_strings[encoder_config], encoder_config);

  encoder_right_.reverse(encoder_config & ENCODER_CONFIG_R_REVERSED);
  encoder_left_.reverse(encoder_config & ENCODER_CONFIG_L_REVERSED);
}

void Ui::set_screensaver_timeout(uint32_t seconds) {
  uint32_t timeout = seconds * 1000U;
  if (timeout < kLongPressTicks * 2)
    timeout = kLongPressTicks * 2;

  screensaver_timeout_ = timeout;
  SERIAL_PRINTLN("Set screensaver timeout to %lu", timeout);
  event_queue_.Poke();
}

void FASTRUN Ui::_Poke() {
  event_queue_.Poke();
}

void Ui::_preemptScreensaver(bool v) {
  preempt_screensaver_ = v;
}

void FASTRUN Ui::Poll() {

  uint32_t now = ++ticks_;
  uint16_t button_state = 0;

  for (size_t i = 0; i < CONTROL_BUTTON_LAST; ++i) {
    if (buttons_[i].Poll())
      button_state |= control_mask(i);
  }

  for (size_t i = 0; i < CONTROL_BUTTON_LAST; ++i) {
    auto &button = buttons_[i];
    if (button.just_pressed()) {
      button_press_time_[i] = now;
    } else if (button.released()) {
      if (now - button_press_time_[i] < kLongPressTicks)
        PushEvent(UI::EVENT_BUTTON_PRESS, control_mask(i), 0, button_state);
      else
        PushEvent(UI::EVENT_BUTTON_LONG_PRESS, control_mask(i), 0, button_state);
    }
  }

  encoder_right_.Poll();
  encoder_left_.Poll();

  int32_t increment;
  increment = encoder_right_.Read();
  if (increment)
    PushEvent(UI::EVENT_ENCODER, CONTROL_ENCODER_R, increment, button_state);

  increment = encoder_left_.Read();
  if (increment)
    PushEvent(UI::EVENT_ENCODER, CONTROL_ENCODER_L, increment, button_state);

  button_state_ = button_state;
}

UiMode Ui::DispatchEvents(App *app) {

  while (event_queue_.available()) {
    const UI::Event event = event_queue_.PullEvent();
    if (IgnoreEvent(event))
      continue;

    switch (event.type) {
      case UI::EVENT_BUTTON_PRESS:
        app->HandleButtonEvent(event);
        break;
      case UI::EVENT_BUTTON_LONG_PRESS:
        if (OC::CONTROL_BUTTON_UP == event.control) {
          if (!preempt_screensaver_) 
            screensaver_ = true;
        }
        else if (OC::CONTROL_BUTTON_R == event.control)
          return UI_MODE_APP_SETTINGS;
        else
          app->HandleButtonEvent(event);
        break;
      case UI::EVENT_ENCODER:
        app->HandleEncoderEvent(event);
        break;
      default:
        break;
    }
    MENU_REDRAW = 1;
  }

  if (idle_time() > screensaver_timeout())
    screensaver_ = true;

  if (screensaver_)
    return UI_MODE_SCREENSAVER;
  else
    return UI_MODE_MENU;
}

UiMode Ui::Splashscreen(bool &reset_settings) {

  UiMode mode = UI_MODE_MENU;

  unsigned long start = millis();
  unsigned long now = start;
 

    do {



    now = millis();

    GRAPHICS_BEGIN_FRAME(false);


    weegfx::coord_t y = menu::CalcLineY(0);

    graphics.setPrintPos(19,5);
   
    graphics.print("UNKNOWN DEVICES");
  
   weegfx::coord_t w,h;
    if (now - start < SPLASHSCREEN_DELAY_MS){
      w = 128;
      h = 64;
    }else{
      w = ((start + SPLASHSCREEN_DELAY_MS + SPLASHSCREEN_TIMEOUT_MS - now) << 7) / SPLASHSCREEN_TIMEOUT_MS;
      h = ((start + SPLASHSCREEN_DELAY_MS + SPLASHSCREEN_TIMEOUT_MS - now) << 6) / SPLASHSCREEN_TIMEOUT_MS;
    }
    graphics.drawRect(126, 62, w, 2);
    graphics.drawRect(0, 62, w, 2);


    //graphics.drawBitmap8(40,13,48,UNWN_LOGO_BITMAP);



    /* fixes spurious button presses when booting ? */
    while (event_queue_.available())
      (void)event_queue_.PullEvent();

    GRAPHICS_END_FRAME();

  } while (now - start < SPLASHSCREEN_TIMEOUT_MS + SPLASHSCREEN_DELAY_MS);


start = millis();
now = start;

  do {


    mode = UI_MODE_MENU;
    if (read_immediate(CONTROL_BUTTON_L))
      mode = UI_MODE_CALIBRATE;
    if (read_immediate(CONTROL_BUTTON_R))
      mode = UI_MODE_APP_SETTINGS;

    reset_settings = 
    #if defined(BUCHLA_4U) && !defined(IO_10V)
       read_immediate(CONTROL_BUTTON_UP) && read_immediate(CONTROL_BUTTON_R);
    #else
       read_immediate(CONTROL_BUTTON_UP) && read_immediate(CONTROL_BUTTON_DOWN);
    #endif

    now = millis();

    GRAPHICS_BEGIN_FRAME(false);

    menu::DefaultTitleBar::Draw();

    #ifdef BUCHLA_cOC
      graphics.print("NLM card O_C");
    #else
      graphics.print("Ornaments & Crimes");
    #endif


    weegfx::coord_t y = menu::CalcLineY(0);

    graphics.setPrintPos(menu::kIndentDx, y + menu::kTextDy);
    graphics.print("[L] => Calibration");
    if (UI_MODE_CALIBRATE == mode)
      graphics.invertRect(menu::kIndentDx, y, 128, menu::kMenuLineH);

    y += menu::kMenuLineH;
    graphics.setPrintPos(menu::kIndentDx, y + menu::kTextDy);
    graphics.print("[R] => Select app");
    if (UI_MODE_APP_SETTINGS == mode)
      graphics.invertRect(menu::kIndentDx, y, 128, menu::kMenuLineH);

    y += menu::kMenuLineH;
    graphics.setPrintPos(menu::kIndentDx, y + menu::kTextDy);
    if (reset_settings)
      graphics.print("!! RESET EEPROM !!");

    y += menu::kMenuLineH;
    graphics.setPrintPos(menu::kIndentDx, y + menu::kTextDy);
    graphics.print(OC_VERSION);

    weegfx::coord_t w;
    if (now - start < SPLASHSCREEN_DELAY_MS)
      w = 128;
    else
      w = ((start + SPLASHSCREEN_DELAY_MS + SPLASHSCREEN_TIMEOUT_MS - now) << 7) / SPLASHSCREEN_TIMEOUT_MS;
    graphics.drawRect(0, 62, w, 2);

    /* fixes spurious button presses when booting ? */
    while (event_queue_.available())
      (void)event_queue_.PullEvent();

    GRAPHICS_END_FRAME();

  } while (now - start < SPLASHSCREEN_TIMEOUT_MS + SPLASHSCREEN_DELAY_MS);
  SERIAL_PRINTLN("UI OC - SET BTN IGNORE");

  SetButtonIgnoreMask();
    SERIAL_PRINTLN("UI OC - BYE BYE");

  return mode;
}

} // namespace OC
