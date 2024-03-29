#![no_std]
#![no_main]

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

extern crate alloc;
use alloc::string::String;
use embedded_graphics::{
    geometry::Size,
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    prelude::Point,
    primitives::{Line, Primitive, PrimitiveStyle, Rectangle},
    text::Text,
    Drawable,
};
use embedded_graphics_framebuf::FrameBuf;
use embedded_hal::digital::InputPin;
use esp_println::println;
use hal::{
    clock::{ClockControl, CpuClock},
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::{Input, PullUp},
    interrupt,
    ledc::{channel, timer, LSGlobalClkSource, LEDC},
    peripherals::{self, Peripherals, TIMG0, TIMG1},
    prelude::*,
    riscv,
    spi::{
        master::{prelude::_esp_hal_spi_master_dma_WithDmaSpi2, Spi},
        SpiMode,
    },
    timer::{Timer, Timer0, TimerGroup},
    Delay, Rng, IO,
};
use micromath::{vector::Vector3d, F32Ext, Quaternion};
use spi_dma_displayinterface::spi_dma_displayinterface;

const LCD_H_RES: u16 = 240;
const LCD_V_RES: u16 = 240;
const COLOR_DEPTH_BYTES: usize = core::mem::size_of::<embedded_graphics::pixelcolor::Rgb565>();
const LCD_PIXELS: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);
const LCD_MEMORY_SIZE: usize = LCD_PIXELS * COLOR_DEPTH_BYTES;

use esp_backtrace as _;

use core::{borrow::BorrowMut, cell::RefCell, mem::MaybeUninit};
use critical_section::Mutex;

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;

// Timer for encoder polling
static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>>>>> = Mutex::new(RefCell::new(None));

// Timer for frame rate calculation
static TIMER1: Mutex<RefCell<Option<Timer<Timer0<TIMG1>>>>> = Mutex::new(RefCell::new(None));

struct Encoder<A, B> {
    pin_a: A,
    pin_b: B,
    prev_a: bool,
    prev_b: bool,
    first_update: bool,
}

enum EncoderDirection {
    Clockwise,
    Anticlockwise,
    None,
}

impl<A, B> Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
{
    pub fn new(pin_a: A, pin_b: B) -> Self {
        Encoder {
            pin_a,
            pin_b,
            prev_a: false,
            prev_b: false,
            first_update: true,
        }
    }
    pub fn update(&mut self) -> EncoderDirection {
        if self.first_update {
            self.prev_a = self.pin_a.is_high().unwrap_or_default();
            self.prev_b = self.pin_b.is_high().unwrap_or_default();
            self.first_update = false;
            return EncoderDirection::None;
        }
        let a = self.pin_a.is_high().unwrap_or_default();
        let b = self.pin_b.is_high().unwrap_or_default();
        let direction = if a && !self.prev_a {
            if b {
                EncoderDirection::Clockwise
            } else {
                EncoderDirection::Anticlockwise
            }
        } else if !a && self.prev_a {
            if b {
                EncoderDirection::Anticlockwise
            } else {
                EncoderDirection::Clockwise
            }
        } else {
            EncoderDirection::None
        };
        self.prev_a = a;
        self.prev_b = b;
        direction
    }
}

static ROTARY_ENCODER: Mutex<
    RefCell<
        Option<
            Encoder<hal::gpio::GpioPin<Input<PullUp>, 6>, hal::gpio::GpioPin<Input<PullUp>, 10>>,
        >,
    >,
> = Mutex::new(RefCell::new(None));

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

static ANGLE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(180.0 / 2.0));

#[interrupt]
fn TG0_T0_LEVEL() {
    critical_section::with(|cs| {
        let mut angle = ANGLE.borrow_ref_mut(cs);
        if let Some(ref mut rotary_encoder) =
            ROTARY_ENCODER.borrow_ref_mut(cs).borrow_mut().as_mut()
        {
            let direction = rotary_encoder.update();
            match direction {
                EncoderDirection::Clockwise => {
                    *angle = f32::min(*angle + 10.0, 180.0);
                    println!("Clockwise");
                }
                EncoderDirection::Anticlockwise => {
                    *angle = f32::max(*angle - 10.0, 0.0);
                    println!("Anticlockwise");
                }
                EncoderDirection::None => {}
            }
        }

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        timer0.clear_interrupt();
        timer0.start(1u64.millis());
    });
}

static FRAME_COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static TIMER_INTERVAL: u64 = 5_000;

#[interrupt]
fn TG1_T0_LEVEL() {
    critical_section::with(|cs| {
        // Calculate and print frame rate
        let frame_count = *FRAME_COUNTER.borrow_ref_mut(cs);
        let frame_rate = frame_count as f64 / (TIMER_INTERVAL as f64 / 1_000.0);
        println!("Frame Rate: {:.2} FPS", frame_rate);

        // Reset frame counter
        *FRAME_COUNTER.borrow_ref_mut(cs) = 0;

        // Reset and restart the timer for the next interval
        let mut timer1 = TIMER1.borrow_ref_mut(cs);
        let timer1 = timer1.as_mut().unwrap();
        timer1.clear_interrupt();
        timer1.start(TIMER_INTERVAL.millis());
    });
}

// Call this function at the end of each frame rendering in your main loop
fn increment_frame_counter() {
    critical_section::with(|cs| {
        *FRAME_COUNTER.borrow_ref_mut(cs) += 1;
    });
}

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    // With DMA we have sufficient throughput, so we can clock down the CPU to 80MHz
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock80MHz).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut timer1 = timer_group1.timer0;

    let mut delay = Delay::new(&clocks);

    println!("About to initialize the SPI LED driver");
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c3/esp32-c3-lcdkit/user_guide.html#gpio-allocation

    let lcd_sclk = io.pins.gpio1;
    let lcd_sda = io.pins.gpio0;
    let lcd_cs = io.pins.gpio7;
    let lcd_dc = io.pins.gpio2.into_push_pull_output();
    let mut lcd_backlight = io.pins.gpio5.into_push_pull_output();

    let rotary_dt = io.pins.gpio10.into_pull_up_input();
    let rotary_clk = io.pins.gpio6.into_pull_up_input();
    let rotary_switch = io.pins.gpio9.into_pull_up_input();

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (_, mut descriptors, _, mut rx_descriptors) = dma_buffers!(4096);

    let spi = Spi::new(peripherals.SPI2, 40u32.MHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(lcd_sclk),
            Some(lcd_sda),
            hal::gpio::NO_PIN,
            Some(lcd_cs),
        )
        .with_dma(dma_channel.configure(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));

    println!("SPI ready");

    let di = spi_dma_displayinterface::new_no_cs(LCD_MEMORY_SIZE, spi, lcd_dc);

    let mut display = mipidsi::Builder::new(mipidsi::models::GC9A01, di)
        .color_order(mipidsi::options::ColorOrder::Bgr)
        .invert_colors(mipidsi::options::ColorInversion::Inverted)
        .init(&mut delay)
        .unwrap();

    let _ = lcd_backlight.set_high();

    let mut rng = Rng::new(peripherals.RNG);
    let mut seed_buffer = [0u8; 32];
    rng.read(&mut seed_buffer).unwrap();

    let rotary_encoder = Encoder::new(rotary_clk, rotary_dt);

    interrupt::enable(
        peripherals::Interrupt::TG0_T0_LEVEL,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    interrupt::enable(
        peripherals::Interrupt::TG1_T0_LEVEL,
        interrupt::Priority::Priority1,
    )
    .unwrap();

    timer0.start(10u64.millis());
    timer0.listen();

    // Meassure FPS
    timer1.start(TIMER_INTERVAL.millis());
    timer1.listen();

    critical_section::with(|cs| {
        ROTARY_ENCODER.borrow_ref_mut(cs).replace(rotary_encoder);
        TIMER0.borrow_ref_mut(cs).replace(timer0);
        TIMER1.borrow_ref_mut(cs).replace(timer1);
    });

    unsafe {
        riscv::interrupt::enable();
    }

    // GPIO4 is the IR tx / rx pin on the LCD dev kit, broken out to a header we can use
    let servo_pin = io.pins.gpio4;

    // let timer_driver = LedcTimer::new(&peripherals.LEDC, &clocks).unwrap();
    let mut servo_driver = LEDC::new(peripherals.LEDC, &clocks);
    servo_driver.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut servo_timer = servo_driver.get_timer(hal::ledc::timer::Number::Timer0);
    servo_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50u32.Hz(),
        })
        .unwrap();

    let mut servo_channel =
        servo_driver.get_channel(hal::ledc::channel::Number::Channel0, servo_pin);

    let mut data = [Rgb565::BLACK; LCD_PIXELS];
    let mut fbuf = FrameBuf::new(&mut data, LCD_H_RES as usize, LCD_V_RES as usize);

    let mut prev_angle = 0.0;

    // Configure timer for servo channel. Default to 50% duty cycle; in the loop we'll use the hardware PWM
    // configuration to set duty cycle based on desired angle.
    servo_channel
        .configure(channel::config::Config {
            timer: &servo_timer,
            duty_pct: 50,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();
    servo_channel.configure_hw().unwrap();

    loop {
        let angle = critical_section::with(|cs| {
            return *ANGLE.borrow_ref_mut(cs);
        });

        if rotary_switch.is_low().unwrap_or(false) {
            println!("Switch pressed");
            critical_section::with(|cs| {
                *ANGLE.borrow_ref_mut(cs) = 180.0 / 2.0;
            });
        }

        let angle_fraction = angle / 180.0;
        let duty_range = (2 as u32).pow(14);
        let duty = (angle_fraction * duty_range as f32) as u32;
        let duty_max = duty_range - 1; // Register overflows at 100% duty; see https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/ledc.html#_CPPv413ledc_set_duty11ledc_mode_t14ledc_channel_t8uint32_t

        if angle != prev_angle {
            prev_angle = angle;
            servo_channel.set_duty_hw(duty.clamp(0, duty_max));
        }

        Rectangle::new(
            Point::new(0, 0),
            Size::new(LCD_H_RES.into(), LCD_V_RES.into()),
        )
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut fbuf)
        .unwrap();

        f32::sin(angle);

        const Z_AXIS: Vector3d<f32> = Vector3d {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };

        let arrow = Quaternion::axis_angle(Z_AXIS, angle.to_radians())
            .scale(LCD_H_RES / 2)
            .rotate(Vector3d {
                x: -1.0,
                y: 0.0,
                z: 0.0,
            });

        let end = arrow
            + Vector3d {
                x: LCD_H_RES as f32 / 2.0,
                y: LCD_V_RES as f32 / 2.0,
                z: 0.0,
            };

        Line::new(
            Point::new((LCD_H_RES / 2).into(), (LCD_V_RES / 2).into()),
            Point::new(end.x as i32, end.y as i32),
        )
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
        .draw(&mut fbuf)
        .unwrap();

        let duty_percent = duty as f32 / duty_range as f32 * 100.0;
        let duty_percent_rounded = (duty_percent * 100.0).round() / 100.0;

        let mut text = String::new();
        text.push_str(ryu::Buffer::new().format(angle));
        text.push_str(" deg\n");
        text.push_str(ryu::Buffer::new().format(duty_percent_rounded));
        text.push_str("% duty");

        Text::new(
            &text,
            Point::new((LCD_H_RES / 2 - 25).into(), (LCD_V_RES - 50).into()),
            MonoTextStyle::new(&FONT_8X13, RgbColor::WHITE),
        )
        .draw(&mut fbuf)
        .unwrap();

        display
            .set_pixels(
                0,
                0,
                LCD_H_RES - 1,
                LCD_V_RES,
                fbuf.into_iter().map(|pixel| pixel.1),
            )
            .unwrap();

        increment_frame_counter();
    }
}
