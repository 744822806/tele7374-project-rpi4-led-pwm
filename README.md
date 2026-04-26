# TELE 7374 - RPi4 LED PWM Controller

A Linux kernel module and Rust userspace program that controls two LEDs based on how fast two pushbuttons are alternately pressed.

## How it works

The faster you alternate between P1 and P2, the brighter the LEDs get. When you slow down or stop pressing, the brightness gradually decreases. L1 stays on at minimum brightness at all times. L2 only lights up when the press speed is above the minimum threshold.

```
slow pressing  →  L1 dim,   L2 off
fast pressing  →  L1 bright, L2 bright
stop pressing  →  brightness fades back down smoothly
```

## Hardware

**Raspberry Pi 4** (BCM2711), kernel 6.1

| Signal | BCM GPIO | Physical Pin |
|--------|----------|--------------|
| P1 (button 1) | GPIO 17 | Pin 11 |
| P2 (button 2) | GPIO 27 | Pin 13 |
| L1 (LED 1)    | GPIO 22 | Pin 15 |
| L2 (LED 2)    | GPIO 23 | Pin 16 |

Buttons: one leg to GPIO pin, other leg to GND (pull-up configured in software).  
LEDs: GPIO → 220Ω resistor → LED → GND.

## Implementation

### Kernel module (`project.c`)

- Uses `ioremap(0xFE200000)` to directly access BCM2711 GPIO registers (GPFSEL, GPSET, GPCLR, GPFEN, GPPUPPDN)
- Software PWM via `hrtimer`: 100µs tick, 100 ticks per period = **10ms PWM period**
- Button speed detection via GPIO falling-edge interrupts (ISR records timestamps of alternating P1→P2 presses)
- Speed is computed from the interval between the last two presses, also factoring in time elapsed since the last press — so brightness starts dropping immediately when the user slows down, with no hard timeout cliff
- Character device `/dev/project`:
  - `read` → `"speed=<N>\n"` (presses per 10 seconds)
  - `write` → `"L1=<0-100> L2=<0-100>"` (sets PWM duty cycle)

### Userspace (`main.rs`)

- Polls `/dev/project` every 200ms
- Maps speed to duty cycle via linear interpolation
- Ramps brightness up gradually when pressing faster
- Follows kernel-computed decay immediately when pressing slower or stopping
- Prints live table of speed and duty cycles to console

## Build & Run

```bash
# build kernel module
make

# load module
sudo insmod project.ko

# verify /dev/project was created
ls -l /dev/project

# compile rust program
rustc main.rs -o main

# run
sudo ./main
```

To unload:
```bash
sudo rmmod project
```

## Console output example

```
TELE 7374 project - LED speed controller
device: /dev/project
speed/10s    L1%        L2%
--------------------------------
0            10         0
0            10         0
8            13         0          <- start pressing
15           16         5
24           19         11
45           28         28
80           55         50         <- pressing fast
80           58         53
0            55         50         <- stopped, starts fading
0            52         47
0            49         44
0            10         0          <- fully faded
```

## Files

| File | Description |
|------|-------------|
| `project.c` | Linux kernel module (PWM + IRQ + char device) |
| `main.rs` | Rust userspace controller |
