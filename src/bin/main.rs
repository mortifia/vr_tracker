#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use libm::{atan2f, sqrtf};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

fn rad_to_deg(rad: f32) -> f32 {
    let mut deg = rad * 180.0 / core::f32::consts::PI;
    while deg > 180.0 {
        deg -= 360.0;
    }
    while deg < -180.0 {
        deg += 360.0;
    }
    deg
}

fn low_pass_filter(prev: f32, current: f32, alpha: f32) -> f32 {
    alpha * current + (1.0 - alpha) * prev
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, freq).unwrap();
    let _led = SmartLedsAdapter::new(
        rmt.channel0,
        peripherals.GPIO8,
        smartLedBuffer!(1),
    );

    info!("Embassy initialized!");

    let mut i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_scl(peripherals.GPIO5)
        .with_sda(peripherals.GPIO4);

    let mpu_addr = 0x68;
    Timer::after(Duration::from_millis(100)).await;

    i2c.write(mpu_addr, &[0x6B, 0x00]).unwrap();
    Timer::after(Duration::from_millis(10)).await;

    let mut whoami = [0u8; 1];
    i2c.write_read(mpu_addr, &[0x75], &mut whoami).unwrap();
    Timer::after(Duration::from_millis(10)).await;

    if whoami[0] != 0x68 {
        info!("MPU6050 not detected!");
        return;
    }

    info!("MPU6050 WHO_AM_I: {:#X}", whoami[0]);
    info!("Calibrating MPU6050...");

    const CALIB_SAMPLES: usize = 100;
    let mut accel_offset = [0f32; 3];
    let mut gyro_offset = [0f32; 3];

    for _ in 0..CALIB_SAMPLES {
        let mut data = [0u8; 14];
        i2c.write_read(mpu_addr, &[0x3B], &mut data).unwrap();

        let ax = i16::from_be_bytes([data[0], data[1]]);
        let ay = i16::from_be_bytes([data[2], data[3]]);
        let az = i16::from_be_bytes([data[4], data[5]]);
        let gx = i16::from_be_bytes([data[8], data[9]]);
        let gy = i16::from_be_bytes([data[10], data[11]]);
        let gz = i16::from_be_bytes([data[12], data[13]]);

        accel_offset[0] += ax as f32 / 16384.0;
        accel_offset[1] += ay as f32 / 16384.0;
        accel_offset[2] += az as f32 / 16384.0;

        gyro_offset[0] += gx as f32 / 131.0;
        gyro_offset[1] += gy as f32 / 131.0;
        gyro_offset[2] += gz as f32 / 131.0;

        Timer::after(Duration::from_millis(5)).await;
    }

    for i in 0..3 {
        accel_offset[i] /= CALIB_SAMPLES as f32;
        gyro_offset[i] /= CALIB_SAMPLES as f32;
    }

    accel_offset[2] -= 1.0;
    let delay = Delay::new();

    let mut yaw = 0.0f32;
    let mut pitch = 0.0f32;
    let mut roll = 0.0f32;

    let mut filtered_pitch = 0.0f32;
    let mut filtered_roll = 0.0f32;
    let mut filtered_yaw = 0.0f32;

    loop {
        let mut data = [0u8; 14];
        i2c.write_read(mpu_addr, &[0x3B], &mut data).unwrap();

        let ax = i16::from_be_bytes([data[0], data[1]]);
        let ay = i16::from_be_bytes([data[2], data[3]]);
        let az = i16::from_be_bytes([data[4], data[5]]);
        let gx = i16::from_be_bytes([data[8], data[9]]);
        let gy = i16::from_be_bytes([data[10], data[11]]);
        let gz = i16::from_be_bytes([data[12], data[13]]);

        let mut ax_g = ax as f32 / 16384.0 - accel_offset[0];
        let mut ay_g = ay as f32 / 16384.0 - accel_offset[1];
        let mut az_g = az as f32 / 16384.0 - accel_offset[2];

        let gx_dps = gx as f32 / 131.0 - gyro_offset[0];
        let gy_dps = gy as f32 / 131.0 - gyro_offset[1];
        let gz_dps = gz as f32 / 131.0 - gyro_offset[2];

        let dt = 0.05;

        let acc_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        let alpha = if (acc_mag - 1.0).abs() < 0.1 { 0.98 } else { 0.995 };

        let threshold = 0.02;
        if ax_g.abs() < threshold { ax_g = 0.0; }
        if ay_g.abs() < threshold { ay_g = 0.0; }
        if az_g.abs() < threshold { az_g = 0.0; }

        let norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
        if norm > 0.0 {
            ax_g /= norm;
            ay_g /= norm;
            az_g /= norm;
        }

        let accel_pitch = rad_to_deg(atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)));
        let accel_roll  = rad_to_deg(atan2f(ay_g, az_g));

        pitch = alpha * (pitch + gy_dps * dt) + (1.0 - alpha) * accel_pitch;
        roll  = alpha * (roll  + gx_dps * dt) + (1.0 - alpha) * accel_roll;

        if gz_dps.abs() > 0.5 {
            yaw += gz_dps * dt;
        } else {
            yaw *= 0.999;
        }
        yaw = rad_to_deg(yaw);

        filtered_pitch = low_pass_filter(filtered_pitch, pitch, 0.1);
        filtered_roll  = low_pass_filter(filtered_roll,  roll,  0.1);
        filtered_yaw   = low_pass_filter(filtered_yaw,   yaw,   0.1);

        info!("Pitch: {}°  Roll: {}°  Yaw: {}°", filtered_pitch, filtered_roll, filtered_yaw);
        delay.delay_millis(50);
    }
}
