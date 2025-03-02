# SEGA SC-3000 Emulator for Raspberry Pi Pico

![BASIC](/pictures/screenshot01.jpg)

---
# Brief

SEGA SC-3000 this is emulator.
Implemented functions:

- Base RAM (32KB)
- VDP (16KB/NTSC)
- PSG
- ROM-cartridge (up to 32KB)
- Tape audio in/out
- Joypad

---
# Schema

- GPIO0 VGA:H-SYNC
- GPIO1 VGA:V-SYNC
- GPIO2 VGA:Blue0 (330 Ohm)
- GPIO3 VGA:Blue1 (680 Ohm)
- GPIO4 VGA:Red0 (330 Ohm)
- GPIO5 VGA:Red1 (680 Ohm)
- GPIO6 VGA:Red2 (1.2K Ohm)
- GPIO7 VGA:Green0 (330 Ohm)
- GPIO8 VGA:Green1 (680 Ohm)
- GPIO9 VGA:Green2 (1.2K Ohm)
- GPIO10 Audio

VGA signals:

```
Blue0 --- 330 Ohm resister ---+
                              |
Blue1 --- 680 Ohm resister ---+---> VGA Blue

Red0  --- 330 Ohm resister ---+
                              |
Red1  --- 680 Ohm resister ---+
                              |
Red2  --- 1.2k Ohm resister --+---> VGA Red

Green0--- 330 Ohm resister ---+
                              |
Green1--- 680 Ohm resister ---+
                              |
Green2--- 1.2k Ohm resister --+---> VGA Green
```

++ GND to connected also

---
# Tuturial

Write the `sc3000emulator.uf2` file located in the `prebuild` folder to the Pico.
The ROM files to be used are stored on LittleFS.

---
# Keyboard

Connect a USB keyboard to the Pico via an OTG cable or other means. The keys that are not present on the USB keyboard are mapped as follows:

 - Break → Pause/Break
 - カナ (Kana) → Katakana/Hiragana
 - GRAPH → ALT
 - RESET → ESC

Additionally, for games, the following keys are mapped:

 - 無変換 (No Conversion) ← Home/CLR
 - 変換 (Conversion) ← INS/DEL

Pressing F12 will bring up the menu screen, where you can manage ROM files and tape images.

---
# Joystick

Supports DirectInput-compatible gamepads (1 device). Button mappings can be changed in the joystick.c file.

---
# ROM Cartridges

Supports ROMs up to 32 KiB in size. 40 KiB cartridges and mega ROMs are not supported.

After placing the ROM file on LittleFS, it can be loaded via the F12 menu.

For information on using LittleFS, please refer to [this article](https://shippoiincho.github.io/posts/39/).

---
# Tape

Supports file input/output in BASIC. Files should be placed on LittleFS.

Note that the SC-3000 does not support hardware control of the tape's `REMOTE` function, so when loading, make sure to start the `LOAD` command in BASIC first, then set the file. (In HomeBASIC, tape control is not implemented at all.)

---
Licenses

This emulator uses the following libraries:

- [Z80](https://github.com/redcode/Z80/tree/master)
- [Zeta](https://github.com/redcode/Zeta)
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)
- [vrEmuTms9918](https://github.com/visrealm/vrEmuTms9918)
- [C-BIOS (フォントのみ)](https://cbios.sourceforge.net/)
- [HID Parser(おそらくLUFAの改変)](https://gist.github.com/SelvinPL/99fd9af4566e759b6553e912b6a163f9)

---
# Limitations

- ROMs larger than 32KiB (including mega ROMs) are not supported.

---
# Gallary

![GAME](/pictures/screenshot00.jpg)
