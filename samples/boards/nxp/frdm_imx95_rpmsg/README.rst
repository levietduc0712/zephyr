.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

.. zephyr:code-sample:: frdm-imx95-rpmsg
   :name: FRDM-i.MX95 RPMsg echo

   Echo RPMsg traffic between Cortex-A55 Linux and Cortex-M7 Zephyr.

The Cortex-M7 firmware publishes a resource table with one RPMsg virtio
device. Linux attaches through remoteproc and binds the advertised ``rpmsg-tty``
channels. The first Zephyr endpoint echoes the payload of each received message.
A second endpoint at ``0x401`` responds to heartbeat probes from Linux.

The sample uses the FRDM Linux 6.18.2 BSP memory layout, where the first
vrings start at ``0x88000000`` and ``0x88008000``, the buffer pool starts at
``0x88020000``, and the resource table starts at ``0x88220000``. Zephyr
maps the shared window as non-cacheable. MU7 channel 1 carries kicks.

Build and run
*************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/nxp/frdm_imx95_rpmsg
   :board: frdm_imx95/mimx9596/m7/ddr
   :goals: build

Package ``zephyr.bin`` as the DDR M7 payload in a FRDM boot container with
load and entry addresses of ``0x80000000``. Use the FRDM BSP Device Tree
and the ``mx95evk`` System Manager configuration. See
:ref:`frdm_imx95_boot_container` for the boot image layout.

After flashing, reset the system so Linux and Zephyr both start with fresh
vrings. On the M7 UART, wait for ``RPMsg TTY echo endpoint ready``. On Linux,
load ``rpmsg_tty`` and identify the device with ``ls /dev/ttyRPMSG*``. To
test a single device named ``/dev/ttyRPMSG0`` from Linux, run:

.. code-block:: console

   modprobe rpmsg_tty
   python3 - <<'PY'
   import os, select, termios, tty

   fd = os.open('/dev/ttyRPMSG0', os.O_RDWR | os.O_NOCTTY)
   original = termios.tcgetattr(fd)
   try:
       tty.setraw(fd)
       termios.tcflush(fd, termios.TCIOFLUSH)
       os.write(fd, b'ping')
       if not select.select([fd], [], [], 3)[0]:
           raise SystemExit('Timed out waiting for the RPMsg echo')
       reply = os.read(fd, 4)
       if reply != b'ping':
           raise SystemExit(f'Unexpected RPMsg response: {reply!r}')
       print(reply.decode())
   finally:
       termios.tcsetattr(fd, termios.TCSANOW, original)
       os.close(fd)
   PY

The test prints ``ping``.

On a Linux-only reboot, Zephyr detects the host clearing the virtio ready bit,
releases its local queue state, and waits for the new host session. It announces
the two RPMsg endpoints again once Linux has reinitialized the queues.

Heartbeat
*********

Identify the heartbeat TTY by ``0x401`` in its ``device/dst`` sysfs attribute.
The Linux peer sends a single 12-byte message every second with ``HBQ1`` followed
by an 8-byte token. Zephyr sends a 20-byte reply containing ``HBR1``, the same
token, and its 64-bit little-endian uptime in milliseconds. Zephyr logs when it
first receives a probe and when three seconds pass without a valid probe.

Onboard LED
***********

The optional LED overlay uses TPM3 channel 0 on ``GPIO_IO04`` to drive the
green channel of D19. It starts off and blinks 500 ms on and 500 ms off while
Zephyr receives heartbeat probes. After three seconds without a valid probe,
Zephyr turns it off. The next valid probe resumes blinking.

.. zephyr-app-commands::
   :zephyr-app: samples/boards/nxp/frdm_imx95_rpmsg
   :board: frdm_imx95/mimx9596/m7/ddr
   :gen-args: -DEXTRA_DTC_OVERLAY_FILE=led.overlay -DEXTRA_CONF_FILE=led.conf
   :goals: build

In the ``mx95evk`` System Manager configuration, move ownership of ``LPTPM3``
and ``PIN_GPIO_IO04`` from the nonsecure AP to the M7. Add ``CLK_BUSWAKEUP GET``
to the M7 API entries so Zephyr can read the TPM3 clock rate. Regenerate the
configuration and package the resulting M33 binary and the Zephyr M7 binary
in the boot container.

After a full board reset, start the A55 heartbeat client and check that D19
blinks green. Stop the client and check that D19 goes dark after the timeout.
The M7 UART prints ``D19 green blinking via TPM3 channel 0`` and
``D19 green off (no A55 heartbeat)`` when the output changes state.
