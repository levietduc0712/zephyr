.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

.. zephyr:board:: frdm_imx95

Overview
********

FRDM-i.MX95 uses an i.MX9596 in a 15 x 15 package with LPDDR4X. These targets run
Zephyr on Cortex-M7 while Cortex-M33 runs System Manager (SM). A55 Linux
coexistence requires a matching boot container, SM configuration and Linux
memory reservations. Compilation and static image checks do not establish
hardware operation or safe Linux coexistence.

The supported targets are ``frdm_imx95/mimx9596/m7`` (TCM) and
``frdm_imx95/mimx9596/m7/ddr`` (DDR). A55 is not implemented by this board port.

.. zephyr:board-supported-runners::

Console and System Manager
**************************

LPUART3 at ``0x42570000`` is the console and shell UART, using 115200 baud,
8 data bits, no parity and one stop bit. GPIO_IO14 is TX and GPIO_IO15 is RX.
The pin configuration follows the FRDM MCUXpresso SDK 26.09.00 reference:
pull-down, DSE=15 (``x4``), FSEL1=2 (``slightly_fast``). The SDK's AVZXN device
selection and existing Zephyr pin definitions are reused.

The board requests 24 MHz for ``IMX95_CLK_LPUART3`` through the SCMI clock
driver after its initialization and before serial initialization. The UART
driver then enables the clock and queries its actual rate. Pinmux uses the
SCMI pinctrl driver; there are no direct SDK board helper calls.

SM must grant the M7 agent access to LPUART3, its clock, the GPIO_IO14/15 pads,
the UART input daisies, MU5 and LPTMR1. The inherited SoC transport uses MU5
doorbell 0 and a 128-byte SCMI shared-memory channel at ``0x44611000``.
Verify the actual SM image and configuration before attempting to boot.
An authorization or transport failure can prevent both the timer and console
from initializing, so absence of output is not proof of a UART wiring fault.

The board enables ``CONFIG_ARM_SCMI_NXP_VENDOR_EXTENSIONS``, which handles
CRC32 on the M7 SMT channel used by the ``mx95evk`` SM configuration. Match
the channel format, MU endpoint numbering, shared-memory address and resource
permissions when selecting an SM image. A different SM configuration can
assign LPUART3 to another core.

Building and inspecting images
*****************************

With a configured west workspace and Zephyr SDK, run from the Zephyr repository:

.. code-block:: console

   west build -p always -b frdm_imx95/mimx9596/m7 samples/hello_world \
     -d build/frdm_imx95_m7_tcm
   west build -p always -b frdm_imx95/mimx9596/m7/ddr samples/hello_world \
     -d build/frdm_imx95_m7_ddr

For interactive UART testing, substitute ``samples/subsys/console/echo`` and
use separate build directories. Select the M7 USB UART channel at 115200 baud
using the FRDM documentation and host enumeration; EVK COM-port numbers do not
identify a FRDM port.

To build Hello World with a diagnostic shell, enable these options:

.. code-block:: console

   west build -b frdm_imx95/mimx9596/m7/ddr samples/hello_world \
     -d build/frdm_imx95_m7_ddr_shell -- -DCONFIG_SHELL=y \
     -DCONFIG_SHELL_MINIMAL=y -DCONFIG_KERNEL_SHELL=y -DCONFIG_DEVICE_SHELL=y

After a system reset, enter ``kernel uptime`` twice on the M7 console to
verify that the kernel clock advances. ``device list`` reports whether the
SCMI mailbox, shared memory, clock protocol and LPUART3 initialized. On the
tested FRDM board the M7 console was ``/dev/ttyACM2`` on the J1 USB bridge.

Each build produces ``zephyr/zephyr.elf``, ``zephyr/zephyr.bin``,
``zephyr/zephyr.map``, ``zephyr/zephyr.dts`` and ``zephyr/.config``. The binary
is raw memory content, with the vector table in its first bytes. The ELF has
addresses, symbols and load segments and is not the AHAB raw-binary input.

The verification script requires ``pyelftools`` from Zephyr's Python
requirements. It checks vector words, stack and reset entry, section bounds,
map symbols, load addresses and every file-backed allocated section against
the raw binary. It performs no device access:

.. code-block:: console

   python boards/nxp/frdm_imx95/support/verify_image.py build/frdm_imx95_m7_tcm
   python boards/nxp/frdm_imx95/support/verify_image.py build/frdm_imx95_m7_ddr

For independent inspection, use the ARM toolchain's ``readelf -h -S -l``,
``nm -n`` and ``objdump -d`` on the ELF and inspect the first eight binary bytes
as two little-endian 32-bit words. The script reports exact addresses for each
build; application options can change the stack and reset-handler locations.

Memory and startup
******************

.. list-table:: Linker allocation windows (exclusive ends)
   :header-rows: 1
   :widths: 20 25 25 30

   * - Target
     - Executable image
     - Writable runtime memory
     - Configuration
   * - TCM
     - ``0x00000000`` to ``0x00040000``
     - ``0x20004000`` to ``0x2001e000``
     - ``CONFIG_XIP=y``
   * - DDR
     - ``0x80000000`` to ``0x8005a000``
     - Same DDR window
     - ``CONFIG_XIP=n``

TCM uses Zephyr's existing i.MX95 M7 linker with ITCM as its flash-like code
region. This is RAM execution; ``CONFIG_XIP=y`` expresses separate instruction
and data regions and permits startup to copy initialized data from the ITCM
binary to DTCM. A separate, bounded SRAM node excludes the SDK's suspend,
non-cacheable and resource-table windows. The full hardware DTCM node is
disabled to prevent extra linker allocation through its original full-size
region. The hardware TCM is not powered down by this devicetree status.

DDR follows the existing EVK non-XIP convention. Its 360 KiB allocation window
is bounded by the combined SDK vector, text and data envelope. Zephyr places
vectors first, then code, constants and writable data using its standard
linker and MPU alignment. It does not reproduce the SDK's 4 KiB vector slot,
text start at ``0x80001000``, data start at ``0x80040000`` or stack top at
``0x8005a000``. Those internal boundaries are not an AHAB loading requirement:
the loader copies the complete raw image at its load address. Inspect every
new application with the script and confirm its entire footprint is reserved.

Both variants retain Zephyr's ``z_arm_reset`` (the reset handler), initial MSP
from ``z_main_stack + CONFIG_MAIN_STACK_SIZE`` and standard ``z_prep_c``.
There is no ``Reset_Handler`` symbol or SDK startup object. Startup programs
SCB VTOR to ``_vector_start``, initializes BSS and, for TCM, copies initialized
data. No board code clears all of DTCM.

The supplied SDK clears ``0x20000000`` through ``0x2003ffff`` for ECC, including
areas outside the application allocation. That routine must not be added to
Zephyr. Before a cold boot, establish that the selected silicon revision and
ROM/OEI/SM combination initializes ECC for the M7-owned TCM before the first
stack access. The pinned mkimage recipe omits a TCM initialization OEI for B0;
this does not prove the current board's silicon revision or ECC readiness.
DDR needs DDR training before either its vectors or its stack can be read.

The existing SoC enables the M7 caches and generic ARM MPU. With non-XIP, the
RAM MPU region is executable Normal memory; generic MPU regions use normal
cacheable attributes, not the SDK's communication-buffer configuration.
ARMv7-M MPU granularity can map more memory than the linker allocation. These
regions do not establish inter-core ownership or Linux reservations. No shared
IPC buffers or non-cacheable application regions are configured by this port.

.. _frdm_imx95_boot_container:

Boot-container integration
**************************

NXP `AN14748`_ revision 2.0 describes staging a raw application as
``iMX95/m7_image.bin`` and assembling an AHAB ``flash.bin``. Its example is a
19 x 19 LPDDR5 EVK; its DDR arguments and boot-switch settings do not apply to
this 15 x 15 LPDDR4X FRDM.

The Linux 6.18.2 BSP metadata identifies imx-mkimage revision
``c0debd7c0b4a125bd3ca66bc68f1915882b2bb62`` and SM revision
``c450f53974ea1df826970ad399cb338625b5638d``. The following are source findings
for those revisions, not evidence about a running board:

* `iMX95/soc.mak`_ stages ``M7_IMG=m7_image.bin``. ``flash_all`` includes SM,
  M7, A55 SPL and an appended ATF/U-Boot container; ``flash_all_ddr`` selects
  DDR for M7. These targets support the later A55 Linux boot chain; they do
  not embed a Linux kernel/root filesystem automatically.
* ``flash_all`` passes M7 vector/entry ``0x00000000`` and system-view load
  address ``0x303c0000``. ``flash_all_ddr`` passes ``0x80000000`` for both.
  These are not the ELF reset-handler address. Do not substitute its Thumb
  pointer for the AHAB entry metadata.
* ``src/mkimage_imx8.c`` parses the M7 file as raw bytes. ``src/imx8qxb0.c``
  records separate destination and entry fields, identifies M7P in the image
  metadata and warns if the entry is not 8-byte aligned. With the default
  IMX9 SD/eMMC option, file offsets and image sizes are padded to 1 KiB.
  Hashing has its own alignment; let mkimage create the padding. The ARM
  vector-table alignment constraint must also hold.
* The `SM ROM handover implementation`_ consumes the ROM image list.
  ``LMM_CpuInit`` saves that boot vector; the CPU boot path programs it via
  ``CPU_ResetVectorSet``. For M7 this writes AONMIX ``INITVTOR`` with no
  address shift. SM configures the isolation/resource domains and CPU/power
  startup sequence. ``DEV_SM_Init`` also requests ELE M7 release. Image
  loading belongs to the ROM/AHAB/OEI boot chain, followed by SM core startup;
  SM does not load an ELF or infer a reset handler from it. The proprietary
  ROM/ELE copy implementation and actual hardware sequence remain unverified.

The FRDM BSP machine is ``imx95-15x15-lpddr4x-frdm``. Its metadata selects
``DDR_TYPE=lpddr4x``, ``OEI_BOARD=mx95lp4x-15`` and the FRDM U-Boot configuration.
Use the BSP-selected DDR training firmware, OEI, AHAB firmware, SM, ATF,
U-Boot/SPL and security policy as one matching set. Confirm actual silicon
revision, SM configuration and evaluated BitBake variables before building.
``OEI=YES`` only includes optional OEI files found by the recipe; it does not
prove DDR initialization is present. The default recipe stages an SDK power
sample as M7 input, so a subsequent BitBake build can replace a manually
staged Zephyr binary. Inspect the final staging and container contents.

For a BSP checkout, these read-only inspections expose the relevant inputs:

.. code-block:: console

   bitbake -e imx-boot | rg '^(MACHINE|IMXBOOT_TARGETS|DDR_TYPE|MKIMAGE_EXTRA_ARGS)='
   bitbake -e imx-boot | rg '^(SYSTEM_MANAGER_CONFIG|M4_DEFAULT_IMAGE|BOOT_STAGING|SRCREV)='
   rg -n 'flash_all|M7_IMG|M7_DDR_ADDR|M7_TCM_ADDR|OEI_OPT_M33' iMX95/soc.mak
   rg -n 'case M7:|img->dst|img->entry|sector_size' src/imx8qxb0.c
   rg -n 'LPUART3|PIN_GPIO_IO14|PIN_GPIO_IO15|LPTMR1|MAILBOX|crc=' configs/mx95evk.cfg

Stage each verified ``zephyr.bin`` in a separate regular-file directory as
``m7_image.bin``, keeping its ELF, map and hashes. Assemble a container only in
a separate, populated BSP staging copy after resolving the firmware and
ownership requirements above. The DDR Hello World sample has been verified
on hardware with the FRDM Linux 6.18.2 BSP, ``mx95evk`` SM build 819, and the
M7 boot image loaded at ``0x80000000``. The M7 console printed the Zephyr
banner and target name at 115200 baud while A55 Linux booted from eMMC.
The diagnostic shell also verified serial receive and kernel uptime. A Linux
reboot can leave M7 running its previous image. For a new M7 image, cleanly
halt Linux and request a system reset from the SM console, or power cycle after
shutting down Linux.

Programming eMMC with west
**************************

The SPSDK runner uses ``nxpuuu`` to send the FRDM BSP bootloader over USB and
write the assembled AHAB image to the eMMC boot partition. Install SPSDK and
make sure ``nxpuuu`` is on the host's ``PATH``. Use BSP files for the same board
and silicon revision, including the BSP's ``imx-boot`` staging files and the
corresponding imx-mkimage checkout.

On Linux, install the USB access rules from NXP's Universal Update Utility
(``uuu -udev``), reload the udev rules, and reconnect J3.

For the FRDM Linux 6.18.2 BSP on B0 silicon, prepare a private copy of the
imx-mkimage checkout. Set ``bsp`` to the directory containing the BSP's
``imx-boot-tools`` and ``imx-boot-*-flash_all`` files, and set ``mkimage`` to
the checkout path. Run the following from the Zephyr repository after building
the DDR target:

.. code-block:: bash

   build="$PWD/build/frdm_imx95_m7_ddr"
   bsp_tools="$bsp/imx-boot-tools"
   machine=imx95-15x15-lpddr4x-frdm
   cp "$bsp_tools/mkimage_imx8" "$mkimage/"
   cp "$bsp_tools"/lpddr4x_*_v202409.bin "$mkimage/iMX95/"
   cp "$bsp_tools"/{mx95b0-ahab-container.img,oei-m33-ddr.bin,tee.bin} "$mkimage/iMX95/"
   cp "$bsp_tools/bl31-imx95.bin-optee" "$mkimage/iMX95/bl31.bin"
   cp "$bsp_tools/m33_image-mx95evk.bin" "$mkimage/iMX95/m33_image.bin"
   cp "$bsp_tools/u-boot-$machine.bin-sd" "$mkimage/iMX95/u-boot.bin"
   cp "$bsp_tools/u-boot-spl.bin-$machine-sd" "$mkimage/iMX95/u-boot-spl.bin"
   cp "$build/zephyr/zephyr.bin" "$mkimage/iMX95/m7_image.bin"
   make -C "$mkimage/iMX95" -f soc.mak REV=B0 OEI=YES LPDDR_TYPE=lpddr4x flash_all_ddr
   cp "$mkimage/iMX95/flash.bin" "$build/flash.bin"

For TCM, use the TCM build directory and the ``flash_all`` make target.
Reassemble ``flash.bin`` after each Zephyr build.

Shut down Linux before switching modes. Set SW1-1 to OFF and SW1-2 to ON,
connect J3 to the host with a USB data cable, and reset the board into Serial
Downloader mode. J1 provides the console. See the `FRDM-i.MX95 Quick Start Guide`_
for the connector locations and physical switch positions.

.. code-block:: bash

   west flash -d "$build" --no-rebuild --bootdevice=emmc \
     --bootloader "$bsp/imx-boot-$machine-sd.bin-flash_all" \
     --flashbin "$build/flash.bin"

After programming, set SW1-1 to ON and SW1-2 to OFF for eMMC boot and reset the
board. On J1, look for the Zephyr boot banner and the Hello World message on
the M7 console.

To load the image into RAM for one boot, use ``--bootdevice=spl`` in the same
command. Check the M7 console on J1 for the Zephyr banner and sample output.

Linux coexistence and SD test procedure
**************************************

Before deploying, collect current board state using read-only commands:

.. code-block:: console

   uname -a
   cat /proc/cmdline
   cat /proc/iomem
   lsblk -o NAME,SIZE,TYPE,FSTYPE,MOUNTPOINTS
   findmnt -no SOURCE,TARGET /
   for r in /sys/class/remoteproc/remoteproc*; do
     printf '\n%s\n' "$r"
     cat "$r/name" "$r/state" "$r/firmware"
   done
   dtc -I fs -O dts /proc/device-tree

The last command prints the active device tree; capture stdout on the host if
needed. Previous observations of kernel 6.18.2, an OS08A20 DTB and
``root=/dev/mmcblk0p2`` are not current ownership evidence.

Review the active ``reserved-memory`` nodes, M7 remoteproc node and boot-loader
load/relocation ranges against the complete image, stacks and BSS. The entire
DDR window ``[0x80000000, 0x8005a000)`` is only a candidate until excluded from
Linux allocation and other firmware loads. A Zephyr DTS node cannot reserve
RAM in Linux. Do not change the board's memory configuration until the active
SM and Linux views have been independently checked.

Keep vrings and IPC buffers, including the SDK references ``0x88000000`` and
``0x88010000``, outside application allocations. Reserve matching resource-table
and shared-buffer regions and agree on cache maintenance before adding IPC.
These samples do not provide an RPMsg resource table or Linux attach protocol.
Remoteproc must not load, clear, stop or restart M7 after the boot container
has started Zephyr. Any Linux auto-attach behavior requires matching firmware,
resource tables and driver support; the previous remoteproc sample is a
different boot flow. Resolve that policy in the SD-specific boot configuration
before testing, without writing live remoteproc controls.

For a human-reviewed SD test:

#. Identify SD and eMMC from current hardware information, card identity and
   mounts. Keep the working eMMC boot image, environment and filesystem intact.
   Do not infer media identity from ``mmcblk`` numbering.
#. Prepare a separate SD boot/rootfs set with the reviewed FRDM LPDDR4X
   container, Linux reservations and remoteproc policy. Device programming is
   a separate manual step; this procedure provides no raw device-write command.
#. Confirm boot-switch settings from documentation for the exact FRDM revision.
   Do not use the AN14748 EVK switch table. Confirm that both the boot image
   and Linux root filesystem select SD: ROM boot-media selection alone does
   not change ``root=``. An SD boot with eMMC mounted read/write would not keep
   the working eMMC untouched.
#. Start with TCM hello_world, capturing M33 SM, M7 LPUART3 and A55 consoles.
   Verify the banner, kernel clock progress and current root mount. Then use
   TCM echo to check both UART directions. After DDR ownership is proven,
   repeat with DDR hello_world and echo.
#. Keep capture logs, the container input manifest and hashes for each result.
   Restore the original ROM boot-media selection to return to the existing
   eMMC setup if the SD test fails; do not alter its saved environment.

RPMsg with Cortex-A55 Linux
***************************

The :zephyr:code-sample:`frdm-imx95-rpmsg` uses the DDR target and the reserved
memory in the FRDM Linux 6.18.2 Device Tree. System Manager starts the M7 from
the boot container. Linux remoteproc attaches to the running M7 and registers
the RPMsg TTY service. Use a system reset when updating the M7 firmware so both
cores initialize the shared queues together.

.. _AN14748: https://www.nxp.com/docs/en/application-note/AN14748.pdf
.. _FRDM-i.MX95 Quick Start Guide: https://www.nxp.com/docs/en/quick-reference-guide/FRDMIMX95QSG.pdf
.. _iMX95/soc.mak: https://github.com/nxp-imx/imx-mkimage/blob/c0debd7c0b4a125bd3ca66bc68f1915882b2bb62/iMX95/soc.mak
.. _SM ROM handover implementation: https://github.com/nxp-imx/imx-sm/blob/c450f53974ea1df826970ad399cb338625b5638d/devices/MIMX95/sm/dev_sm_rom.c
