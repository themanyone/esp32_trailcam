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
#define DIFFERENCE_THRESHOLD 5        // Percentage of pixels that need to change to trigger a save (1-100)
#define COMPARISON_FRAME_SIZE FRAMESIZE_QQVGA // Low-res for faster comparison
#define CAPTURE_FRAME_SIZE FRAMESIZE_XGA       // High-res for saving

// --- PIN DEFINITIONS ---
// The flash LED is on GPIO 4
#define FLASH_LED_PIN 4

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

// --- CONSTANTS ---
const char* REF_IMG_PATH = "/ref_img.raw";

bool setup_camera(framesize_t frameSize);
void save_picture(camera_fb_t *fb);
int compare_images(camera_fb_t *fb1, uint8_t *fb2_buf);
bool update_reference_image(camera_fb_t *fb);
bool read_reference_image(uint8_t *buf, size_t len);

void setup() {
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("Waking up...");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  // Initialize SD card on every boot
  if (!SD_MMC.begin()) {
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
        int diff = compare_images(fb, ref_buf);
        Serial.printf("Image difference: %d%%.\n", diff);

        // The new reference image should be the one we just captured for comparison.
        // Update it *before* de-initializing the camera or taking another picture.
        update_reference_image(fb);

        if (diff > DIFFERENCE_THRESHOLD) {
          Serial.println("Motion detected! Capturing high-res image.");
          esp_camera_deinit(); // De-init low-res camera
          if (setup_camera(CAPTURE_FRAME_SIZE)) {
            // Flash on
            digitalWrite(FLASH_LED_PIN, HIGH);
            // Add a longer delay to allow the sensor to adjust to the flash
            delay(1200); 

            camera_fb_t *fb_high = esp_camera_fb_get();
            digitalWrite(FLASH_LED_PIN, LOW);

            if (fb_high) {
              save_picture(fb_high);
              esp_camera_fb_return(fb_high);
            } else {
              Serial.println("Failed to capture high-res image.");
            }
          }
        } else {
          Serial.println("No significant motion detected.");
        }
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Failed to capture comparison frame.");
      }
      esp_camera_deinit();
    }
    free(ref_buf); // Free the memory
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

bool update_reference_image(camera_fb_t *fb) {
  File file = SD_MMC.open(REF_IMG_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open reference image for writing");
    return false;
  }
  if (file.write(fb->buf, fb->len) != fb->len) {
    Serial.println("Failed to write reference image");
    file.close();
    return false;
  }
  Serial.printf("Reference image updated on SD card (%d bytes).\n", fb->len);
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

int compare_images(camera_fb_t *fb1, uint8_t *fb2_buf) {
  if (!fb1 || !fb2_buf) {
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