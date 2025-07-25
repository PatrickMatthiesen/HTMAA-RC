# HTMAA-RC

## Setting up the IDE

1. Open the project in your IDE.
2. Go to `File > Preferences > Additional Boards Manager URLs`
   1. Add the following URL:
      `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
3. Go to `Tools > Board > Boards Manager`
   1. Search for `esp32` and install the latest version of `esp32 by Espressif Systems` and `esp32_bluepad32 by Ricardo Quesada`
4. Go to `Tools > Board` and select `NodeMCU-32s`
5. Now install libraries:
   1. Go to `Tools > Library Manager`
   2. Search for `ESP32Servo` and install version `3.0.7` of `ESP32Servo by Kevin Harrington`
6. You are now ready to upload the code to your board.

> [!NOTE]
> If the board is not recognized, you may need to install the CP2102 USB to Serial Driver. You can find it [here](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads).

## Uploading the code

1. Connect the board to your computer via USB.
2. Press upload in the IDE.
3. Wait for the `connecting` message to appear in the console.
4. Hold down the `Right` button on the board.
5. Press the `Reset / EN` button on the board.
6. Release the `Right` button.
7. Wait for the upload to complete.

## Relevant links

- [CP2102 USB to Serial Driver](https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads)
- [Bluepad32](https://bluepad32.com/)
- [Blue H-bridge](https://hobbycomponents.com/motor-drivers/264-l9110s-dc-stepper-motor-driver-h-bridge)
