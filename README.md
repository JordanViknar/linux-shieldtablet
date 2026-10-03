# Mainline Linux for the NVIDIA SHIELD Tablet

> [!IMPORTANT]
> 
> This kernel is being tested exclusively on an NVIDIA SHIELD Tablet K1 with the NVIDIA SHIELD Tablet K1's Android 5.0/L bootloader. It **should** work on other models, but not necessarily other bootloader versions.
>
> **Use this kernel at your own risk.** It is still in a bit of an experimental state. When using this kernel, you agree to the possibility of hardware damage. I've had none so far, but cannot guarantee you won't.

## Compatibility and build notes

- Compatibility with the Android M/N bootloader is not immediately planned due to seemingly stricter requirements (no initramfs unless bundled in-kernel, smaller kernel image size). The structure to support it is in place though.
- A U-Boot port is planned, but not yet done.
- This port uses `shieldtablet_defconfig` to store its default kernel configuration.
- **Recommended compiler flags:**

  ```text
  -march=armv7ve -mtune=cortex-a15
  ```

## Current status

### ✅ Fully functional

- **Backlight**
- **Battery fuel gauge:** onsemi LC709203F (custom driver: `lc709203f`)
- **Charger:** TI BQ24192 (driver: `bq24190-charger`)
- **CPU:** 4× 2.1 GHz Cortex-A15
- **DRM:** `tegradrm` & `simpledrm`
- **eMMC:** SanDisk SDW16G
- **Hardware keys:** "Power", "Volume Up", "Volume Down", and "Lid Switch" function perfectly.
  - Stylus Pen Detection is untested, as the K1 model doesn't have it.
  - "Camera Focus" is configured by downstream, but seems to be an oddity.
- **Magnetometer:** Asahi Kasei AK8963 (driver: `ak8975`)
- **Motion tracking:** Invensense MPU6515 (driver: `inv-mpu6050-i2c`)
- **microSD card slot:** Enabled, but UHS-I support is untested. **At your own risk.**
- **Thermal sensors:** Using `tegra_soctherm` & `generic_adc_thermal`
- **USB:** Host and device mode (OTG) on the micro-USB port, with role and VBUS switching through the Palmas PMIC's extcon. Uses the ChipIdea EHCI/UDC controller (`ci_hdrc_tegra`), XUSB is not used.
  - Tested with a USB mass-storage stick (host) and a CDC-ACM gadget (device).
- **Voltage monitor:** TI INA3221 (driver: `ina3221`)

### 🟡 Fully functional, with quirks

- **Audio:** Realtek RT5639 Codec
- **Bluetooth:** Broadcom BCM43241B0 (using `hci_bcm`)
- **Touchscreen:** Raydium RM31080 (using custom driver `raydium-rm31080-spi-bridge`)
- **Wi-Fi:** Broadcom BCM43241B0 (using `brcmfmac`)

### ⚠️ Partially functional

- **Display:** Minor instabilities (strange jagged lines) found under high system load.
  - Can on very rare occasion crash on startup with a screen whiteout effect. I suggest quickly emergency rebooting with the power button should this happen.
- **GPU:** Using `nouveau`. Pretty unstable, and works only with a patch to Mesa: [mesa-tegra](https://github.com/JordanViknar/mesa-tegra/tree/tegra-fixes).
  - Xorg seems to have major issues; Wayland works a bit better.
  - Personally, I have `MESA_LOADER_DRIVER_OVERRIDE=tegra` in `/etc/environment` and then enable software rendering for applications that are too buggy.
  - Example for GTK apps: `GSK_RENDERER=cairo`
- **HDMI:** Audio is fully functional, but HDMI in general is unstable.
  - It can crash the tablet if connected in some situations (framebuffer usage, some monitor setups ?).

### Configured, untested

- **Thermal zones**
- **Headset connected through jack**
- **HDMI CEC**

### ❌ Non-functional / Not worked on yet

- **GPS**
- **Camera**
- **Suspend**

### Unplanned (excluding external contributions)

- **3G/LTE:** My SHIELD Tablet is a K1, it only has Wi-Fi.
- **Stylus:** I do not have the stylus, so I cannot test it.

> Elements not listed above are either not tested/worked on yet, or not applicable to this device.

## Custom drivers

Drivers that are unique to this kernel port and not mainlined:

- **Battery fuel gauge - `lc709203f`**
  - Custom driver for the fuel gauge chip, based on the one from Zephyr RTOS.
  - See `drivers/power/supply/lc709203f-fuel-gauge.c`

- **Touchscreen - `raydium-rm31080-spi-bridge`**
  - Reproduces the `/dev/touch` misc-device protocol that the original out-of-tree downstream Raydium kernel driver exposed, so the closed-source Android userspace touch stack can be used unmodified on a mainline kernel.
  - It does not decode touch data itself, and is not intended to be upstreamed.
  - See `drivers/input/touchscreen/raydium-rm31080-spi-bridge.c`.
  - Unlike the L4T port driver, no separate Xorg configuration is required, and this driver has been confirmed to work with Wayland as a result.

## Quirks

### Minor

- **Audio:** Userspace needs custom ALSA UCM and Wireplumber configuration to work properly with the tablet's audio hardware.
  - See [linux-shieldtablet-rootfs](https://github.com/JordanViknar/linux-shieldtablet-rootfs)
- **Wi-Fi & Bluetooth:** Requires firmware files inside initramfs.
  - See [linux-shieldtablet-firmware](https://github.com/JordanViknar/linux-shieldtablet-firmware)
- **GPU:** Requires firmware files inside initramfs.
  - See [linux-shieldtablet-firmware](https://github.com/JordanViknar/linux-shieldtablet-firmware)

### Major

- **Bootloader:** Baking too many modules into the kernel itself will cause kernel and/or initramfs failures on boot. Reason unknown.
	- U-Boot may help avoid this issue in the future.
- **Display:** Do not reboot directly from mainline into downstream kernel. It glitches the display and may or may not cause hardware damage if left in this state too long. Rebooting from downstream into mainline is unaffected.
- **Touchscreen:** Requires the closed-source downstream Android userspace touch stack (`ts.default.so` / `librm31080.so`), run with `rm-wrapper`.
  - Check my old [Xubuntu 22.04 rootfs builder](https://github.com/JordanViknar/shieldtablet-l4t-kernel-multirom-builder) for a working example in its code. The current kernel driver alone produces no touch events.

## Implementation notes

### Audio

The SHIELD Tablet does not wire up a dedicated headphone-detect GPIO. Instead, the RT5639 uses its built-in jack detection (`realtek,jack-detect-source`), requiring a small generic addition to `tegra_asoc_machine.c` to support codecs with `.set_jack()` but no HP detect GPIO.

Audio routing (speaker/headphones, microphone selection, etc.) is supposed to be handled entirely in userspace via ALSA UCM + WirePlumber.

Because the RT5639 exposes speaker and headphone outputs through a shared PCM, UCM presents them as separate profiles rather than switchable ports. WirePlumber's `auto-profile` rule provides automatic switching.

This seems to be an upstream rt5640 UCM limitation also affecting other devices.

### HDMI Audio

The HPD GPIO backing the HDMI connector has no hardware debounce, and some sinks briefly pulse HPD low after a mode change/resync.

`tegra_hdmi_connector_detect()` cleared `HDMI_NV_PDISP_SOR_AUDIO_HDA_PRESENSE` on any such blip with no way to restore it short of a full encoder disable/enable, so audio could break until a lucky reconnect (video was unaffected, since it doesn't use this register).

Fixed by corroborating HPD against the DDC bus in `tegra_output_connector_detect()`, since DDC only responds while a sink is genuinely present. `tegra_output_ddc_alive()` waits for consecutive reads to agree, since connector pins don't all make/break contact at once.

A sturdier version would compare sink identity across the debounce window instead, closer to `amdgpu_dm`'s handling of the same problem.

See `drivers/gpu/drm/tegra/output.c`.

### SMP

On the Android L bootloader (may not be exclusive to it), PSCI CPU_ON claims success but never actually powers on CPU1-3.

Worked around by using a PSCI CPU_ON call on CPU0 (already running) purely to register the reset vector with firmware, then powering CPU1-3 on directly via PMC instead of PSCI, matching what the Tegra4Linux kernel does.

See:

- `arch/arm/mach-tegra/reset.c`
- `arch/arm/mach-tegra/platsmp.c`

### USB Device Mode

VBUS and ID on the micro-USB port are sensed by the Palmas PMIC (reported through its extcon), and ChipIdea switched to device mode correctly, but a PC never saw the tablet.

Presumably the SoC's own VBUS/session-valid inputs do not see VBUS on this board. U-Boot and the downstream `tegra_udc` driver both force those inputs to "valid" in software, which mainline's PHY driver never did.

Added an opt-in `nvidia,pmu-vbus-detection` property that makes the PHY force them while it is powered in device/OTG mode, and release them again on power off so the hardware VBUS wake path is untouched during suspend.

Removing the property again breaks device mode, so it is required on this board. It may also be what keeps device mode from working on other Tegra124 boards with PMIC-sensed VBUS such as the Xiaomi Mi Pad (`xiaomi-mocha`), but I have not tested that.

See `drivers/usb/phy/phy-tegra-usb.c`.

### USB Host Mode

USB mass-storage devices enumerated but never produced a block device: a bogus "Max LUN 117", then repeated port resets as soon as the SCSI scan started, with no errors reported by the controller.

ChipIdea's `CI_HDRC_REQUIRES_ALIGNED_DMA` workaround (used by Tegra) bounces URBs whose length is not a multiple of 4 through a temporary buffer.

`usb-storage` sends its command/status wrappers (31/13 bytes) and the GET_MAX_LUN request through a buffer it already DMA-mapped, so the USB core does not map the temporary one: the controller keeps using the original buffer, but on completion the never-written temporary buffer was copied back over the received data.

Replies came back as uninitialised memory, so the status wrappers failed `usb-storage`'s signature check and triggered its reset recovery.

The fix is simply to not bounce URBs whose buffer is already mapped.

See `drivers/usb/chipidea/host.c`.

## AI Notice

This port is a prototype, and due to time constraints and the complexity of this port, I was helped by LLMs for some of the C code and research.

It has been fully tested on my personal tablet, and I made sure to understand the underlying issues and suggested solutions.

If this is problematic for you, you are free to develop your own port and to use mine as reference. Do note a lot of the code will be rewritten manually to ensure upstream-worthy quality.

## Q&A

### Wasn't this port called "linux-tn8"?

**A:** The NVIDIA SHIELD Tablet's official codename is "Tegra Note 8" (TN8). The device tree names were based on the downstream kernel's naming scheme.

However, considering:

1. The Xiaomi Mi Pad is also a TN8 device technically speaking: [Xiaomi Kernel OpenSource](https://github.com/MiCode/Xiaomi_Kernel_OpenSource/commit/79b4898e25fe3b506ff902e182b47068598c838a)
2. Earlier revisions of the board are also referred to as TN8.
3. Most community projects use "shieldtablet" as codename.

I have switched the kernel's codename to **`shieldtablet`** to avoid confusion.

## Credits

### Previous mainline port attempts

- **Alexandre Courbot** - original basic Linux 3.17 mainline port, luckily saved from deletion by Andrew Querol:
  - [winsock/linux](https://github.com/winsock/linux/tree/st8)
- **Aaron Kling** - more recent partial device trees for the NVIDIA SHIELD Tablet and Ardbeg:
  - [`tegra124-tn8.dts`](https://gitlab.incom.co/CM-Shield/android_hardware_nvidia_t124_lineage/-/blob/lineage-24.0/tegra124-tn8.dts)
  - [`tegra124-ardbeg.dts`](https://gitlab.incom.co/CM-Shield/android_hardware_nvidia_t124_lineage/-/blob/lineage-24.0/tegra124-ardbeg.dts)

### Other devices' mainline ports

- **`tegra124-xiaomi-mocha.dts`** - the other publicly-released TN8 device, extremely similar to the SHIELD Tablet and already upstreamed. Used for some comparisons and general structure. Extremely helpful.
- **`tegra30-asus-nexus7-grouper-common.dtsi`** - helped me with audio support, as they use the same RT5640/RT5639 codec.
- **`tegra234-p3737-0000+p3701.dtsi`** and **`tegra234-p3740-0002+p3701-0008.dts`** - helped me tremendously with headphones support, since they also use the RT5640/RT5639 codec's `jack-detect-source` property.

### Downstream kernels

- LineageOS's downstream kernel for the NVIDIA SHIELD Tablet was used as base:
  - [android_kernel_nvidia_shield](https://github.com/LineageOS/android_kernel_nvidia_shield)
- The original Linux4Tegra port's downstream DTS was also used as base:
  - [XDA thread](https://xdaforums.com/t/linux4tegra-r23-1-r24-1-beta-for-the-shield-tablet.2985238/)
  - [STLinux-Kernel](https://github.com/bogsen/STLinux-Kernel)
  - [Decompiled L4T DTS](https://gist.github.com/JordanViknar/8fef373312b60890d718846927095e6b)
