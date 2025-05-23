#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    //Spawn some tasks
    spawner.spawn(hello_world()).unwrap();
    spawner.spawn(rainbow(peripherals.RMT, peripherals.GPIO8.into())).unwrap();
}

#[embassy_executor::task]
async fn hello_world() {
    let mut count = 0;
    loop {
        info!("Hello world! {}", count);
        count += 1;
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn rainbow(rmt: esp_hal::peripherals::RMT, gpio: esp_hal::gpio::AnyPin) {
    let freq = Rate::from_mhz(80);

    let rmt = Rmt::new(rmt, freq).unwrap();
    let rmt_buffer = smartLedBuffer!(1);

    let mut led = SmartLedsAdapter::new(rmt.channel0, gpio, rmt_buffer);
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    
    let mut data;
    loop {
        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = [hsv2rgb(color)];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 8))
                .unwrap();
            // delay.delay_millis(10);
            Timer::after_millis(10).await;
        }
    }
}


