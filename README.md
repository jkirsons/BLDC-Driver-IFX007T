# BLDC Driver

Hardware: https://oshwlab.com/jkirsons/ifx007t-bldc-driver

Datasheets:
* [IFX007T Driver](https://www.infineon.com/dgdl/Infineon-IFX007T-DS-v01_00-EN.pdf?fileId=5546d46265f064ff0166433484070b75)
* [MA702 Encoder](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA702/document_id/3561)
* [TPS560430 Buck](https://www.ti.com/lit/ds/symlink/tps560430.pdf)
* [ESP32 Microcontroller](https://www.espressif.com/sites/default/files/documentation/esp32-pico-v3_datasheet_en.pdf)
* [SN65HVD230QDR CAN Tranceiver](https://www.ti.com/lit/ds/symlink/sn65hvd230q.pdf)

This example code is flashed to the board using VSCode & PlatformIO, using the Espressif 32 Platform, and [SimpleFOC](https://simplefoc.com) (2.1.0) Library.

You will need a USB to UART dongle connected to the UART port to Flash/Monitor.

To enter Bootloader flash mode, hold the boot button, tap reset, then let go of the boot button.


CAN is not yet implemented in the test code.

![PCB Image](/images/PCB-Front.png)

![PCB Image 2](/images/PCB-Back.png)

