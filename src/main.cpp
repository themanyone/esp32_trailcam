/*

  This is a motion-sensing trail cam Using the ESP32-CAM Development Board
  from DFROBOT. It wakes from deep sleep, samples a camera frame from the
  OV2640 camera, compares it to a previous frame, and saves the new image
  to an SD card if the change is beyond a certain threshold.

*/

#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- CONFIGURATION ---
#define SLEEP_INTERVAL_SECONDS 6      // Time to sleep between checks
#define MINIMUM_THRESHOLD 15           // Minimum percentage change to trigger, to avoid triggers on very low noise
#define COMPARISON_FRAME_SIZE FRAMESIZE_QQVGA // Low-res for faster comparison
#define CAPTURE_FRAME_SIZE FRAMESIZE_XGA       // High-res for saving
#define ENABLE_LED_FLASH false // flash LED?
#define DEBUG_SAVE_RAW_IMAGES false // Set to true to save the two comparison images

const char* NEW_IMG_PATH = "/new_img.raw";

// --- PIN DEFINITIONS ---
// On the AI-THINKER board, GPIO 4 is used by the SD card.
// To avoid conflicts, we will use a different GPIO for the flash LED.
// Pin 4 would normally work
#define FLASH_LED_PIN 33

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// --- PERSISTENT DATA (survives deep sleep) ---
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int pictureNumber = 0;
// History of the last 10 image differences to calculate a moving average
const int DIFF_HISTORY_SIZE = 10;
RTC_DATA_ATTR int diff_history[DIFF_HISTORY_SIZE] = {0};
RTC_DATA_ATTR int diff_history_index = 0;
RTC_DATA_ATTR int diff_history_count = 0;

// --- CONSTANTS ---
const char* REF_IMG_PATH = "/ref_img.raw";

bool setup_camera(framesize_t frameSize);
void save_picture(camera_fb_t *fb);
int compare_images(const camera_fb_t *fb1, const uint8_t *fb2_buf);
bool update_reference_image(const camera_fb_t *fb, const char* path = REF_IMG_PATH);
bool read_reference_image(uint8_t *buf, size_t len);

void setup() {
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("Waking up...");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Initialize SD card on every boot
  // Use 4-bit mode to avoid conflicts with the flash LED on GPIO 4
  if (!SD_MMC.begin("/sdcard", false, true)) {
    Serial.println("SD Card Mount Failed. Going to sleep.");
    esp_deep_sleep_start();
    return;
  }

  if (bootCount == 0) {
    Serial.println("First boot. SD card initialized.");
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached. Going to sleep.");
      esp_deep_sleep_start();
      return;
    }
    bootCount++;

    // On first boot, take a reference picture
    Serial.println("Taking reference picture...");
    if (setup_camera(COMPARISON_FRAME_SIZE)) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        update_reference_image(fb);
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Failed to capture reference frame.");
      }
      esp_camera_deinit();
    }
  } else {
    // On wake, explicitly access RTC variables to prevent optimizer from removing them.
    // This ensures 'pictureNumber' is loaded from RTC memory.
    volatile int dummy_read = pictureNumber;

    // Buffer to hold the reference image read from SD card
    // We allocate it here because it's not needed on the very first boot.
    uint8_t *ref_buf = (uint8_t*) malloc(160 * 120);
    if (!ref_buf) {
        Serial.println("Failed to allocate memory for reference buffer!");
        esp_deep_sleep_start();
        return;
    }

    // Waking from sleep, check for motion
    Serial.println("Checking for motion...");
    if (setup_camera(COMPARISON_FRAME_SIZE)) {
      // Add a delay to allow the sensor to stabilize its auto-exposure and white-balance
      delay(1000); 

      camera_fb_t *fb = esp_camera_fb_get();
      // Read the previous reference image from the SD card
      if (fb && read_reference_image(ref_buf, fb->len)) {
        // --- DEBUG: Save the two images being compared ---
        if (DEBUG_SAVE_RAW_IMAGES) {
          // The reference image is already on the SD card as /ref_img.raw
          // We just need to save the new one for comparison.
          update_reference_image(fb, NEW_IMG_PATH);
        }
        int diff = compare_images(fb, ref_buf);
        Serial.printf("Image difference: %d%%.\n", diff);

        // --- DYNAMIC THRESHOLD CALCULATION ---
        // Add current difference to our history for the moving average
        diff_history[diff_history_index] = diff;
        diff_history_index = (diff_history_index + 1) % DIFF_HISTORY_SIZE;
        if (diff_history_count < DIFF_HISTORY_SIZE) {
          diff_history_count++;
        }

        // Calculate the moving average
        float moving_average = 0;
        for (int i = 0; i < diff_history_count; i++) {
          moving_average += diff_history[i];
        }
        if (diff_history_count > 0) {
            moving_average /= diff_history_count;
        }

        // Dynamic threshold is 10% above the moving average, with a minimum value
        int dynamic_threshold = (int)(moving_average * 1.10);
        dynamic_threshold = max(dynamic_threshold, MINIMUM_THRESHOLD);
        Serial.printf("Moving average: %.2f%%, Dynamic threshold: %d%%\n", moving_average, dynamic_threshold);

        if (diff > dynamic_threshold) {
          Serial.println("Motion detected! Capturing high-res image.");
          esp_camera_deinit(); // De-init low-res camera
          if (setup_camera(CAPTURE_FRAME_SIZE)) {
            // Flash on
            if (ENABLE_LED_FLASH)
                digitalWrite(FLASH_LED_PIN, HIGH);

            // Allow the camera sensor to stabilize by capturing and discarding a few frames.
            // This gives the auto-exposure and auto-white-balance algorithms time to adjust.
            for (int i=0; i<2; i++) {
                camera_fb_t *burn_fb = esp_camera_fb_get();
                esp_camera_fb_return(burn_fb);
            }

            camera_fb_t *fb_high = esp_camera_fb_get();

            if (fb_high) {
              save_picture(fb_high);
              esp_camera_fb_return(fb_high);
            } else {
              Serial.println("Failed to capture high-res image.");
            }

            if (ENABLE_LED_FLASH)
                digitalWrite(FLASH_LED_PIN, LOW);
          }
        } else {
          Serial.println("No significant motion detected.");
          // Update the reference image only when the scene is static.
          // This prevents a second photo when the scene returns to normal.
          update_reference_image(fb);
        }
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Failed to capture comparison frame.");
      }
      free(ref_buf); // Free the memory after all camera operations are done.
      esp_camera_deinit();
    }
  }

  Serial.printf("Going to sleep for %d seconds...\n\n", SLEEP_INTERVAL_SECONDS);
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_SECONDS * 1000000);
  esp_deep_sleep_start();
}

void loop() {
  // This will not be called, as the ESP32 will be in deep sleep.
}

bool setup_camera(framesize_t frameSize) {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  if (frameSize == CAPTURE_FRAME_SIZE) {
    config.pixel_format = PIXFORMAT_JPEG;
    config.jpeg_quality = 12; // 0-63 lower number means higher quality
    config.fb_count = 1; // We only need one frame for capture
  } else {
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.fb_count = 1;
  }

  // Frame parameters
  config.frame_size = frameSize;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  // Get a pointer to the sensor object
  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("Failed to get camera sensor.");
    return false;
  }

  // --- TUNE SENSOR SETTINGS TO IMPROVE IMAGE QUALITY ---
  // The default settings for the OV2640 sensor can result in dark, greenish images.
  // These settings help improve color balance and exposure.
  s->set_whitebal(s, 1);       // 1 = enable automatic white balance
  s->set_awb_gain(s, 1);       // 1 = enable AWB gain
  // Set the LED flash pin in the camera driver
  // This is important if you use a pin other than the default GPIO 4
  s->set_reg(s, 0x00, s->get_reg(s, 0x00, 0xff) | 0x04, 0x04); // REG00
  s->set_reg(s, 0x11, (s->get_reg(s, 0x11, 0xff) & 0xBF) | (0x1 << 6), 0x40); // CLKRC, set bit 6 to 1
  s->set_exposure_ctrl(s, 1);  // 1 = enable automatic exposure control
  s->set_aec2(s, 1);           // 1 = enable automatic exposure correction

  return true;
}

void save_picture(camera_fb_t *fb) {
  // Poor man's file name
  String path = "/picture" + String(pictureNumber++) + ".jpg";
  Serial.printf("Saving picture to: %s\n", path.c_str());

  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.write(fb->buf, fb->len)) {
    Serial.printf("File saved: %s\n", path.c_str());
  } else {
    Serial.println("File write failed");
  }
  file.close();
}

bool update_reference_image(const camera_fb_t *fb, const char* path) {
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("Failed to open %s for writing\n", path);
    return false;
  }
  if (file.write(fb->buf, fb->len) != fb->len) {
    Serial.printf("Failed to write to %s\n", path);
    file.close();
    return false;
  }
  Serial.printf("Image saved to %s (%d bytes).\n", path, fb->len);
  file.close();
  return true;
}

bool read_reference_image(uint8_t *buf, size_t len) {
  File file = SD_MMC.open(REF_IMG_PATH, FILE_READ);
  if (!file) {
    Serial.println("Failed to open reference image for reading");
    return false;
  }
  if (file.read(buf, len) != len) {
    Serial.println("Failed to read reference image");
    file.close();
    return false;
  }
  file.close();
  return true;
}

int compare_images(const camera_fb_t *fb1, const uint8_t *fb2_buf) {
  if (!fb1 || !fb2_buf) {
    // This can happen if the reference image file doesn't exist yet
    Serial.println("Cannot compare NULL frames");
    return 101; // Return a value > 100 to indicate error
  }

  if (fb1->format != PIXFORMAT_GRAYSCALE) {
    Serial.println("Comparison only works with Grayscale images");
    return 102;
  }

  int pixels_changed = 0;
  int total_pixels = fb1->width * fb1->height;

  for (int i = 0; i < total_pixels; i++) {
    // Simple pixel difference check
    if (fb1->buf[i] != fb2_buf[i]) {
      pixels_changed++;
    }
  }

  // Calculate percentage of changed pixels
  float diff_percentage = (float)pixels_changed * 100.0 / (float)total_pixels;

  return (int)diff_percentage;
}