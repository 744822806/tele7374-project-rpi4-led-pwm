// TELE 7374 - Class Project
// Userspace program: reads button speed from /dev/project,
// maps it to LED duty cycles, and writes them back.
//
// Compile: rustc main.rs -o main
// Run:     sudo ./main

use std::fs::OpenOptions;
use std::io::{self, Read, Write};
use std::thread;
use std::time::Duration;

const DEVICE: &str = "/dev/project";

// speed range we measured experimentally on the hardware
const MIN_SPEED: u32 = 1;
const MAX_SPEED: u32 = 150;

// L1 stays on at minimum brightness even when idle
const L1_MIN: u32 = 10;
const L1_MAX: u32 = 100;
const L2_MIN: u32 = 0;
const L2_MAX: u32 = 100;

// how fast to ramp up brightness when pressing faster
const RAMP_STEP: u32 = 3;

const POLL_MS: u64 = 200;

fn parse_speed(buf: &[u8]) -> Option<u32> {
    let s = std::str::from_utf8(buf).ok()?;
    for line in s.lines() {
        if let Some(val) = line.trim().strip_prefix("speed=") {
            return val.trim().parse::<u32>().ok();
        }
    }
    None
}

// linear map: speed -> duty cycle
fn map_to_duty(speed: u32, out_min: u32, out_max: u32) -> u32 {
    if speed <= MIN_SPEED { return out_min; }
    if speed >= MAX_SPEED { return out_max; }
    out_min + (speed - MIN_SPEED) * (out_max - out_min) / (MAX_SPEED - MIN_SPEED)
}

fn read_speed() -> io::Result<u32> {
    let mut f = OpenOptions::new().read(true).open(DEVICE)?;
    let mut buf = [0u8; 32];
    let n = f.read(&mut buf)?;
    parse_speed(&buf[..n]).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            format!("bad device output: {:?}", String::from_utf8_lossy(&buf[..n])),
        )
    })
}

fn write_duty(l1: u32, l2: u32) -> io::Result<()> {
    let mut f = OpenOptions::new().write(true).open(DEVICE)?;
    f.write_all(format!("L1={} L2={}", l1, l2).as_bytes())
}

fn step_toward(cur: u32, target: u32, step: u32) -> u32 {
    if cur < target { (cur + step).min(target) }
    else { cur.saturating_sub(step).max(target) }
}

fn main() {
    println!("TELE 7374 project - LED speed controller");
    println!("device: {}", DEVICE);
    println!("{:<12} {:<10} {:<10}", "speed/10s", "L1%", "L2%");
    println!("{}", "-".repeat(32));

    let mut cur_l1 = L1_MIN;
    let mut cur_l2 = L2_MIN;

    loop {
        match read_speed() {
            Ok(speed) => {
                let target_l1 = map_to_duty(speed, L1_MIN, L1_MAX);
                let target_l2 = if speed <= MIN_SPEED { 0 } else { map_to_duty(speed, L2_MIN, L2_MAX) };

                // ramp up gradually, but follow the kernel's decay immediately
                // (kernel already smooths the decay by factoring in time since last press)
                if target_l1 > cur_l1 || target_l2 > cur_l2 {
                    cur_l1 = step_toward(cur_l1, target_l1, RAMP_STEP);
                    cur_l2 = step_toward(cur_l2, target_l2, RAMP_STEP);
                } else {
                    cur_l1 = target_l1;
                    cur_l2 = target_l2;
                }

                println!("{:<12} {:<10} {:<10}", speed, cur_l1, cur_l2);

                if let Err(e) = write_duty(cur_l1, cur_l2) {
                    eprintln!("write error: {}", e);
                }
            }
            Err(e) => eprintln!("read error: {}", e),
        }

        thread::sleep(Duration::from_millis(POLL_MS));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse() {
        assert_eq!(parse_speed(b"speed=5\n"), Some(5));
        assert_eq!(parse_speed(b"speed=0\n"), Some(0));
        assert_eq!(parse_speed(b"garbage"),   None);
    }

    #[test]
    fn test_map() {
        assert_eq!(map_to_duty(0,        10, 100), 10);
        assert_eq!(map_to_duty(MIN_SPEED, 10, 100), 10);
        assert_eq!(map_to_duty(MAX_SPEED, 10, 100), 100);
    }

    #[test]
    fn test_step() {
        assert_eq!(step_toward(10, 100, 3), 13);
        assert_eq!(step_toward(100, 10, 3), 97);
        assert_eq!(step_toward(10, 10, 3),  10);
    }
}
