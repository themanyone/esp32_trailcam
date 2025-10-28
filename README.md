# ESP32-CAM Motion-Sensing Trail Camera

This project turns an ESP32-CAM development board into a low-power, motion-activated, offline trail camera. It wakes from a deep sleep state, captures a low-resolution image to check for changes, and if significant motion is detected, it captures a high-resolution photo with flash and saves it to an SD card.

## Features

- **Motion Detection:** Compares sequential images to detect motion without a PIR sensor.
- **Low Power:** Utilizes ESP32's deep sleep to conserve battery, waking only periodically to check for motion.
- **High-Quality Capture:** Saves images in XGA resolution (1024x768) to a MicroSD card.
- **Low-Light Operation:** Uses the onboard LED as a flash for captures in the dark.
- **Configurable:** Easily adjust sleep interval, motion sensitivity, and camera model.

## Future directions

- **Time and Date** Clock is reset each time the device goes to sleep, but RTC clock board could be added.
- **Web Server** A web server could be added to let you view stored images over wifi.

## How It Works

The core logic is designed for power efficiency:

1.  **First Boot:** On the very first power-up, the device initializes the SD card and takes a low-resolution grayscale photo to serve as the first "reference" image, saving it to the SD card.
2.  **Deep Sleep:** The ESP32 enters deep sleep for a configured interval (e.g., 6 seconds).
3.  **Wake & Compare:** Upon waking up, the device:
    - Initializes the camera in low-resolution grayscale mode.
    - Allows the image sensor to stabilize.
    - Captures a new image.
    - Reads the previous reference image from the SD card.
    - Compares the new image with the old one, calculating the percentage of different pixels.
4.  **Motion Trigger:**
    - **If motion is detected** (the difference exceeds a set threshold):
        - The camera is re-initialized for high-resolution JPEG capture.
        - The flash LED is turned on.
        - A high-resolution photo is taken and saved to the SD card with a unique filename (e.g., `picture1.jpg`).
    - **If no motion is detected:** The device proceeds to the next step.
5.  **Update & Repeat:** The new low-resolution image becomes the reference for the next cycle and is saved to the SD card, overwriting the old one. The device then goes back to deep sleep.

## Hardware Required

- **ESP32-CAM Board:** This project is configured for the **AI-THINKER** model.
- **MicroSD Card:** For storing the photos.
- **FTDI Programmer (3.3V):** A USB-to-serial converter to upload the code.
- **Jumper Wires:** For connecting the programmer to the ESP32-CAM.

## Software & Installation

This project is built using PlatformIO with Visual Studio Code.

### 1. Setup VS Code and PlatformIO

- Install Visual Studio Code.
- Open VS Code, go to the Extensions view, and install the official **PlatformIO IDE** extension.

### 2. Get the Code

- Clone this repository to your local machine:
  ```bash
  git clone <your-repository-url>
  ```
- Open the cloned project folder in VS Code. PlatformIO should automatically recognize it.

### 3. Configure the Project (Optional)

Open `src/main.cpp` to adjust the behavior:

- `SLEEP_INTERVAL_SECONDS`: How long the device sleeps between motion checks.
- `DIFFERENCE_THRESHOLD`: The percentage of pixels (1-100) that must change to trigger a high-res capture. Lower is more sensitive.
- `CAMERA_MODEL_AI_THINKER`: If you have a different ESP32-CAM model, comment this line and uncomment the one that matches your board.

### 4. Compile and Upload

**Hardware Connection:**

You must connect the FTDI programmer to the ESP32-CAM to upload code.

1.  Set your FTDI programmer to **3.3V**.
2.  Connect the pins as follows:

    | FTDI Programmer | ESP32-CAM      |
    | --------------- | -------------- |
    | VCC (5V)        | 5V             |
    | GND             | GND            |
    | TX              | U0R (RX)       |
    | RX              | U0T (TX)       |
    |                 | GPIO0--GND     |


3.  **IMPORTANT:** To enable flashing mode, create a jumper wire connection between the **GND** and **GPIO0** pins on the ESP32-CAM.

**Upload Steps:**

1.  With the hardware connected and `GPIO0` grounded, plug the FTDI programmer into your computer's USB port.
2.  In VS Code, open the PlatformIO sidebar.
3.  Under `env:esp32cam` > `General`, click **Upload**.
4.  PlatformIO will compile the code and upload it to the board.
5.  Once the upload is complete, **disconnect the jumper between GND and GPIO0**.
6.  Press the onboard `RST` button to start the program.

You can monitor the device's output by using the **Serial Monitor** task in PlatformIO.
Choose port, probably /dev/ttyUSB0, and set baud rate to 115200.

## License

Copyright (c) 2025 Henry Kroll III, www.thenerdshow.com
This project is licensed under the BSD 3-Clause License. See the LICENSE.txt file for details.
