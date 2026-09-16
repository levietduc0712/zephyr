.. zephyr:code-sample:: x-nucleo-iks4a1-i3c
   :name: X-NUCLEO-IKS4A1 I3C sensors
   :relevant-api: sensor_interface i3c_interface

   Read I3C and legacy I2C sensors on the same bus.

Overview
********

This sample reads the LSM6DSV16X accelerometer and LPS22DF pressure sensor
over I3C, and the LSM6DSO16IS accelerometer and LIS2MDL magnetometer using
legacy I2C transfers through the I3C controller. It prints one set of
measurements per second.

Requirements
************

Configure the :ref:`x-nucleo-iks4a1` shield for Mode 1 and attach it to a
board exposing an ``arduino_i3c`` controller. The board overlay must enable
that controller and disable any I2C controller sharing its pins.

On :zephyr:board:`nucleo_h7s3l8`, select I3C using the ``I2C_NI3C`` option
byte as described in the board documentation before running the sample.
The sample enables cache management on this board so that instruction and
data caches are active when executing from external flash. Servicing I3C
FIFOs with interrupts requires sufficient CPU throughput; uncached external
flash execution can cause FIFO underruns or overruns.

Building and running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/shields/x_nucleo_iks4a1/standard_i3c
   :board: nucleo_h7s3l8
   :goals: build flash
   :compact:

For larger configurations, use external flash:

.. zephyr-app-commands::
   :zephyr-app: samples/shields/x_nucleo_iks4a1/standard_i3c
   :board: nucleo_h7s3l8/stm32h7s3xx/ext_flash_app
   :west-args: --sysbuild
   :goals: build flash
   :compact:

DMA
***

On NUCLEO-H7S3L8, enable DMA for longer transfers or higher bus rates:

.. zephyr-app-commands::
   :zephyr-app: samples/shields/x_nucleo_iks4a1/standard_i3c
   :board: nucleo_h7s3l8/stm32h7s3xx/ext_flash_app
   :west-args: --sysbuild
   :gen-args: -DEXTRA_CONF_FILE=dma.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nucleo_h7s3l8_dma.overlay
   :goals: build flash
   :compact:

This configuration uses GPDMA1 channels 0 through 3 for RX, TX, control,
and status transfers. Other enabled peripherals must not use these channels.
CCC commands and dynamic address assignment continue to use interrupts.
