# SEGA SC-3000 Emulator for Raspberry Pi Pico

![BASIC](/pictures/screenshot01.jpg)

---
# 概要

SEGA SC-3000 のエミュレータです。
以下の機能を実装しています。

- メイン RAM (32KB)
- VDP (16KB/NTSC)
- PSG
- ROM カートリッジ (32KiB まで)
- テープ
- Joypad

---
# 配線

Pico と VGA コネクタやブザー/スピーカーなどを以下のように配線します。

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

VGA の色信号は以下のように接続します

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

このほかに VGA、Audio の　GND に Pico の　GND を接続してください。

---
# 使い方

`prebuild` 以下にある `sc3000emulator.uf2` を Pico に書き込みます。
使用する ROM ファイルを　LittleFS 上において使用します。

---
# キーボード

Pico の USB 端子に、OTG ケーブルなどを介して USB キーボードを接続します。
USB キーボードに存在しないキーは以下のように割り当てています。

- Break   → Pause/Break
- カナ　 　→ カタカナ・ひらがな
- GRAPH  　→ ALT
- RESET    → ESC

また、ゲームのために以下のキーも割り当てています

- 無変換  ← Home/CLR
- 変換    ← INS/DEL

また F12 でメニュー画面に移ります。
ROM ファイルや テープイメージの操作ができます。

---
# Joystick

DirectInput 対応のゲームパッド(1台)に対応しています。
ボタンの割り当ては、`joystick.c` で変更できます。

---
# ROM カートリッジ

32KiB までの rom に対応しています。
40KiB カートリッジやメガ ROM には対応していません。

rom ファイルを LittleFS 上に置いた後で、F12 のメニューからロードできます。

LittleFS の使い方については、
[こちらの記事](https://shippoiincho.github.io/posts/39/)をご覧ください。

---
# Tape

BASIC における、ファイルの入出力に対応しています。LittleFS 上においてください。

なお SC-3000 はハード的にテープの REMOTE 制御に対応していませんので、ロードの際は BASIC で　LOAD コマンドを開始してからファイルをセットしてください。
(HomeBASIC ではそもそも制御すらしていないので)

---
# ライセンスなど

このエミュレータは以下のライブラリを使用しています。

- [Z80](https://github.com/redcode/Z80/tree/master)
- [Zeta](https://github.com/redcode/Zeta)
- [VGA ライブラリ(一部改変)](https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics)
- [LittleFS](https://github.com/littlefs-project/littlefs)
- [vrEmuTms9918](https://github.com/visrealm/vrEmuTms9918)
- [C-BIOS (フォントのみ)](https://cbios.sourceforge.net/)
- [HID Parser(おそらくLUFAの改変)](https://gist.github.com/SelvinPL/99fd9af4566e759b6553e912b6a163f9)

---
# 制限事項

- 32KiB を超える ROM (メガROM含む) には非対応です

---
# Gallary

![GAME](/pictures/screenshot00.jpg)
