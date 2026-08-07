#### Background:

We are using [Xiao ESP32-S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)

The default camera sensor is [OV3660](https://datasheet.iiic.cc/datasheets-0/omnivision_technologies/OV03660-A51A.pdf)

#### Current:

##### How to calibrate?
- Flash BlobDetectionCalibrate.ino on the Xiao ESP32S3
- Keep the ESP32 connected to the computer via USB
- Run BlobDetectionCalibrate.py on the computer
- A window should pop up with the default mask applied.
- Use the <1, Q, 2, W, 3, E, 4, R, 5, T, 6, Y> to adjust the mask.
  - L_min -/+: 1 / Q
  - L_max -/+: 2 / W
  - A_min -/+: 3 / E
  - A_max -/+: 4 / R
  - B_min -/+: 5 / T
  - B_max -/+: 6 / Y
- Every time you make a change to the mask, the current mask details are shown in the terminal
- You change the default mask inside of BlobDetectionCalibrate.py (Line 14-16)

##### How to apply the mask to the blob detector?
- Copy and paste the current mask details from the terminal to the line starting with: `static const LabThreshold THRESHOLD_BLIMP =`
  - This applies to both the version built on ESP-IDF and Ardunio IDE

##### How to use the blob detector?
- You need to flash one of the BlobDetectors on to the Xiao ESP32S3
- If you want to view the detections, the ESP32 needs to be connected to the computer via USB
- Run ViewDetections.py to view the live detections
- To see the image after the color mask is applied by uncomment: `applyColorMask(&work_fb);`

##### How to build the ESP-IDF version of blob detector?
1. cd .\BlobDetectionESP-IDF
2. idf.py fullclean
    - You can also directly delete: build, managed_components, dependencies.lock, sdkconfig
3. idf.py set-target esp32s3
4. idf.py -p <PORT> build flash

##### How to switch between video streaming + telemetry and telemetry only in BlobDetectionESP-IDF?
- For telemetry only, set DEBUG_STREAM 0
- For video & telemetry, set DEBUG_STREAM 1

##### Notes:
1. PSRAM is required for the scripts to work, due to memory use from precaching the mask.
2. You will have to use the reset button on the ESP32 before each flashing attempt since the programs use the USB port as well.
3. Image Path: JPEG -> RGB565 - JPEG
  - We use JPEG initially so we have access to the hardware denoising
  - We use RGB565 to decrease the size of the precached mask data.
  - We use JPEG on the output for management and transmission.
4. We use the LAB based mask to minimize the affect of differing lighting conditions.
  - May change this in the future to HSV for faster processing

#### Past:
1. CameraWebServer was the initial reference for the project
2. BlobDetection was the first version (eg. doesn't have precaching)
3. BlobDetectionSoftAP adds a Wifi access point / Web Server for visual debugging + precaching
4. BlobDetectionFast is directly based on BlobDetectionSoftAP, but without Wifi or Web Server
  - Same inputs and outputs as BlobDetection, except faster
  - Used for initial benchmarking, proving that Xiao ESP32S3 Sense is capable of similar speeds to relative to Nicla Vision
5. AutoCalibrateWithStdOverROI was the initial attempt at building a calibration algorithm
  - It uses mean +/-1 standard deviation over a region of interest to create the mask
  - viewfinder.py allows you to align the region of interest with your chosen target
  - Uses RGB565 directly from the camera instead of using JPEG and its hardware denoising
6. AutoCalibrateWithStdOverROIWithMasking allows you to visualize the mask in real time using viewfinder.py
  - Uses the same mask creation algorithm as AutoCalibrateWithStdOverROI
  - The masking is performed inside of AutoCalibrateWithStdOverROIWithMasking.ino
  - The viewfinder.py is the same as AutoCalibrateWithStdOverROI
5. BlobDetectionSpeedTest is a slightly modified version of BlobDetectionESP-IDF that removes image streaming and adds timing
  - Managed to get 130ms loop
  - Works with speedTest.py
