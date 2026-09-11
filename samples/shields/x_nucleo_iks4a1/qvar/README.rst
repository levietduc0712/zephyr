.. zephyr:code-sample:: x-nucleo-iks4a1-qvar
   :name: X-NUCLEO-IKS4A1 shield - QVAR Mode 5
   :relevant-api: sensor_interface

   Poll the LSM6DSV16X AH/QVAR channel on an X-NUCLEO-IKS4A1 shield.

Overview
********

This sample polls the LSM6DSV16X AH/QVAR charge variation sensing output
through the Zephyr sensor API. It prints a filtered QVAR envelope computed from
a short burst of raw samples so that a stable touch is reported as a stable
positive value. It is intended for X-NUCLEO-IKS4A1 Mode 5, where QVAR1 and
QVAR2 are routed through J4 and J5 to the external electrode connection.

Requirements
************

Use an X-NUCLEO-IKS4A1 shield on a board with an Arduino connector, such as
the :zephyr:board:`nucleo_u575zi_q` board.

Configure the shield jumpers for Mode 5 before testing electrode interaction:

* J4 on pins 3-4
* J5 on pins 3-4
* QVAR electrode connected through JP6/JP7

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/shields/x_nucleo_iks4a1/qvar/
   :host-os: unix
   :board: nucleo_u575zi_q
   :shield: x_nucleo_iks4a1
   :goals: build flash
   :compact:

Output Format
=============

	.. code-block:: console

	   LSM6DSV16X QVAR polling
	   QVAR: <millivolts> mV
	   QVAR: <millivolts> mV clipped=<clipped samples>/32

The optional ``clipped`` field appears when raw AH/QVAR conversions hit the ADC
rails inside the burst. A high clipped count usually means the electrode or
direct touch setup is overdriving the QVAR input.
