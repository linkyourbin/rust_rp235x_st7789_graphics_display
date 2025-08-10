# 硬件接线
> 忽略touch_cs即可，若是手里屏幕不带触摸功能，屏幕尺寸为240*240
>
```rust
let bl = p.PIN_13;    // LCD backlight control
let rst = p.PIN_15;   // LCD reset pin
let display_cs = p.PIN_9;  // LCD chip select
let dcx = p.PIN_8;    // Data/Command control (0=command, 1=data)
let miso = p.PIN_12;  // SPI MISO (Master In Slave Out)
let mosi = p.PIN_11;  // SPI MOSI (Master Out Slave In)
let clk = p.PIN_10;   // SPI clock
let touch_cs = p.PIN_16; // Touch controller chip select
```
# 关键依赖，推荐直接照抄，防止出现依赖错误，无法编译（挺难处理的）
> 可能有些依赖用不到，但是总比缺了好
```toml
[package]
name = "rp2350_st7789"
version = "0.1.0"
edition = "2024"

[dependencies]
embassy-embedded-hal = { version = "0.3.0", features = ["defmt"] }
embassy-sync = { version = "0.6.2", features = ["defmt"] }
embassy-executor = { version = "0.7.0", features = ["arch-cortex-m", "executor-thread", "executor-interrupt", "defmt"] }
embassy-time = { version = "0.4.0", features = ["defmt", "defmt-timestamp-uptime"] }
embassy-rp = { version = "0.4.0", features = ["defmt", "unstable-pac", "time-driver", "critical-section-impl", "rp235xa", "binary-info"] }
embassy-futures = { version = "0.1.1"}
cyw43 = { version = "0.3.0",  features = ["defmt", "firmware-logs"] }
cyw43-pio = { version = "0.4.0", features = ["defmt"] }

defmt = "1.0.1"
defmt-rtt = "1.0.0"
fixed = "1.23.1"
fixed-macro = "1.2"

serde = { version = "1.0.203", default-features = false, features = ["derive"] }
serde-json-core = "0.5.1"


#cortex-m = { version = "0.7.6", features = ["critical-section-single-core"] }
cortex-m = { version = "0.7.6", features = ["inline-asm"] }
cortex-m-rt = "0.7.0"
critical-section = "1.1"
panic-probe = { version = "1.0.0", features = ["print-defmt"] }
display-interface-spi = "0.5.0"
embedded-graphics = "0.8.1"
mipidsi = "0.8.0"
display-interface = "0.5.0"
byte-slice-cast = { version = "1.2.0", default-features = false }
smart-leds = "0.3.0"
heapless = "0.8"
usbd-hid = "0.8.1"
```


# embedded-hal-1 = { package = "embedded-hal", version = "1.0.0" }
embedded-hal-async = "1.0"
embedded-hal-bus = { version = "0.1", features = ["async"] }
embedded-io-async = { version = "0.6.1", features = ["defmt-03"] }
embedded-storage = { version = "0.3" }
static_cell = "2.1"
portable-atomic = { version = "1.5", features = ["critical-section"] }
log = "0.4"
embedded-sdmmc = "0.7.0"
micromath = "2.1.0"

[profile.release]
debug = 2

[profile.dev]
lto = true
opt-level = "z"


# `main.rs`
```rust
//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2350 chip.
//! 
//! Example written for a display using the ST7789 chip. Specifically designed for the Waveshare Pico-ResTouch LCD 2.8"
//! (https://www.waveshare.com/wiki/Pico-ResTouch-LCD-2.8)
//!
//! The code implements a comprehensive test suite to verify various display functionalities:
//! 1. Color accuracy test (RGB primaries and grayscales)
//! 2. Geometric primitive drawing (rectangles, circles, lines)
//! 3. Text rendering with different fonts and styles
//! 4. Dynamic animation test (rotating pattern)
//! 5. Image rendering test (Ferris logo)
//! 6. Screen fill performance test
//!
//! Note: The display resolution is 240x240 pixels, though the physical display is 320x240.
//! The example configures the display to use only the central 240x240 area for compatibility with the demo assets.

#![no_std]
#![no_main]

// Core Rust libraries
use core::cell::RefCell;

// Logging and debugging
use defmt::*;
use {defmt_rtt as _, panic_probe as _};

// Display interface and graphics libraries
use display_interface_spi::SPIInterface;
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10}; // Standard ASCII fonts for text rendering
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565; // 16-bit RGB color format (5-6-5)
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, Rectangle, PrimitiveStyle}; // Basic drawing primitives
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;

// Embassy framework components for RP2040/RP2350
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output}; // GPIO control
use embassy_rp::spi; // SPI driver
use embassy_rp::spi::{Blocking, Spi}; // SPI interface types
use embassy_sync::blocking_mutex::raw::NoopRawMutex; // Mutex implementation without OS support
use embassy_sync::blocking_mutex::Mutex; // Blocking mutex for shared resources
use embassy_time::{Delay, Duration, Timer}; // Timing functions

// Math library for embedded systems
use micromath::F32Ext; // Provides trigonometric functions for f32

// Display driver library
use mipidsi::models::ST7789; // Specific display controller model
use mipidsi::options::{Orientation, Rotation}; // Display orientation options
use mipidsi::Builder; // Display initialization builder

// Configuration constants for SPI communication
const DISPLAY_FREQ: u32 = 80_000_000; // 64 MHz - Maximum supported by this display
const TOUCH_FREQ: u32 = 200_000;      // 200 kHz - Lower speed for touch controller

// Static storage for shared SPI bus (thread-safe access)
static SPI_BUS: static_cell::StaticCell<Mutex<NoopRawMutex, RefCell<Spi<'static, embassy_rp::peripherals::SPI1, Blocking>>>> = static_cell::StaticCell::new();

/// Test Case 1: Color Accuracy Test
/// 
/// Purpose: Verify the display's ability to render primary colors, secondary colors, and grayscale accurately.
/// Implementation:
/// - Creates a 3x3 grid of color blocks (80x80 pixels each)
/// - Top row: RGB primary colors
/// - Middle row: CMY secondary colors
/// - Bottom row: 8-step grayscale
/// - Labels the test with "COLOR TEST" text
fn test_color_accuracy<T>(display: &mut T) 
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running color accuracy test");
    display.clear(Rgb565::BLACK).unwrap();
    
    // RGB primary colors (top row)
    Rectangle::new(Point::new(0, 0), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(80, 0), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(160, 0), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(display)
        .unwrap();
    
    // Secondary colors (middle row)
    Rectangle::new(Point::new(0, 80), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::CYAN))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(80, 80), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
        .draw(display)
        .unwrap();
    Rectangle::new(Point::new(160, 80), Size::new(80, 80))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .draw(display)
        .unwrap();
    
    // Grayscale test (bottom row) - 8 steps from black to white
    for i in 0..8 {
        let gray = (i * 36) as u8; // 0, 36, 72, ..., 252
        let color = Rgb565::new(gray, gray, gray);
        Rectangle::new(Point::new(i * 30, 160), Size::new(30, 80))
            .into_styled(PrimitiveStyle::with_fill(color))
            .draw(display)
            .unwrap();
    }
    
    // Test identification label
    let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
    Text::new("COLOR TEST", Point::new(10, 220), small_text_style)
        .draw(display)
        .unwrap();
}

/// Test Case 2: Geometric Primitive Drawing Test
/// 
/// Purpose: Verify the display's ability to render basic geometric shapes with different styles.
/// Implementation:
/// - Draws filled and stroked rectangles with different colors
/// - Draws filled and stroked circles
/// - Draws diagonal lines across the display
/// - Labels the test with "GEOMETRIC TEST" text
fn test_geometric_primitives<T>(display: &mut T)
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running geometric primitives test");
    display.clear(Rgb565::BLACK).unwrap();
    
    // Rectangles with different styles
    // Solid red rectangle (top left)
    Rectangle::new(Point::new(10, 10), Size::new(100, 60))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
        .draw(display)
        .unwrap();
    
    // Green rectangle with thick border (top right)
    Rectangle::new(Point::new(130, 10), Size::new(100, 60))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 3))
        .draw(display)
        .unwrap();
    
    // Solid blue rectangle (middle)
    Rectangle::new(Point::new(60, 90), Size::new(120, 40))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
        .draw(display)
        .unwrap();
    
    // Circles
    // Solid yellow circle (bottom left)
    Circle::new(Point::new(50, 180), 30)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .draw(display)
        .unwrap();
    
    // Cyan circle with thin border (bottom right)
    Circle::new(Point::new(190, 180), 30)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::CYAN, 2))
        .draw(display)
        .unwrap();
    
    // Lines
    // Magenta diagonal line (top-left to bottom-right)
    Line::new(Point::new(0, 0), Point::new(240, 240))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::MAGENTA, 1))
        .draw(display)
        .unwrap();
    
    // White diagonal line (top-right to bottom-left)
    Line::new(Point::new(240, 0), Point::new(0, 240))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
        .draw(display)
        .unwrap();
    
    // Test identification label
    let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
    Text::new("GEOMETRIC TEST", Point::new(10, 220), small_text_style)
        .draw(display)
        .unwrap();
}

/// Test Case 3: Text Rendering Test
/// 
/// Purpose: Verify the display's ability to render text with different styles, sizes, and colors.
/// Implementation:
/// - Renders text in two different font sizes
/// - Tests multi-line text rendering
/// - Tests text within a bordered region
/// - Tests different colored text
/// - Labels the test with "TEXT TEST" text
fn test_text_rendering<T>(display: &mut T)
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running text rendering test");
    display.clear(Rgb565::BLACK).unwrap();
    
    // Define text styles for the test
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);      // Larger font, green
    let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW); // Smaller font, yellow
    
    // Different font sizes
    Text::new("BIG TEXT", Point::new(10, 20), text_style)
        .draw(display)
        .unwrap();
    
    Text::new("Medium Text", Point::new(10, 50), small_text_style)
        .draw(display)
        .unwrap();
    
    // Multi-line text
    Text::new(
        "Line 1\nLine 2\nLine 3",
        Point::new(10, 80),
        small_text_style,
    )
    .draw(display)
    .unwrap();
    
    // Border test - text within a defined region
    Rectangle::new(Point::new(10, 140), Size::new(220, 60))
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::CSS_GRAY, 1))
        .draw(display)
        .unwrap();
    
    Text::new(
        "Text within bounds",
        Point::new(20, 160),
        small_text_style,
    )
    .draw(display)
    .unwrap();
    
    // Color test - different colored text
    Text::new("RED", Point::new(10, 200), MonoTextStyle::new(&FONT_6X10, Rgb565::RED))
        .draw(display)
        .unwrap();
    Text::new("GREEN", Point::new(50, 200), MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN))
        .draw(display)
        .unwrap();
    Text::new("BLUE", Point::new(110, 200), MonoTextStyle::new(&FONT_6X10, Rgb565::BLUE))
        .draw(display)
        .unwrap();
}

/// Test Case 4: Dynamic Animation Test
/// 
/// Purpose: Verify the display's ability to render smooth animations.
/// Implementation:
/// - Creates a rotating star pattern with 8 lines
/// - Each line rotates at a different speed and has a different color
/// - Displays "ANIMATION" label at the bottom
/// - Runs for approximately 3 seconds (100 frames at 30ms/frame)
async fn test_animation<T>(display: &mut T)
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running animation test");
    display.clear(Rgb565::BLACK).unwrap();
    
    let mut angle = 0.0; // Starting rotation angle
    
    // Run animation for 100 frames (approximately 3 seconds)
    for _ in 0..100 {
        display.clear(Rgb565::BLACK).unwrap();
        
        // Draw rotating lines (star pattern)
        for i in 0..8 {
            // Calculate rotation angle for this line
            let rad = (angle + i as f32 * 0.785).to_radians();
            
            // Use micromath trigonometric functions to calculate line endpoints
            let x = 120.0 + 80.0 * rad.cos();
            let y = 120.0 + 80.0 * rad.sin();
            
            // Draw the line with appropriate color and thickness
            Line::new(
                Point::new(120, 120),
                Point::new(x as i32, y as i32),
            )
            .into_styled(PrimitiveStyle::with_stroke(
                match i % 3 {
                    0 => Rgb565::RED,
                    1 => Rgb565::GREEN,
                    _ => Rgb565::BLUE,
                },
                2,
            ))
            .draw(display)
            .unwrap();
        }
        
        // Add animation identification label
        let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
        Text::new("ANIMATION", Point::new(80, 220), small_text_style)
            .draw(display)
            .unwrap();
        
        // Increment rotation angle for next frame
        angle += 5.0;
        
        // Wait before drawing next frame (30ms = ~33 FPS)
        Timer::after(Duration::from_millis(1)).await;
    }
}

/// Test Case 5: Image Rendering Test
/// 
/// Purpose: Verify the display's ability to render bitmap images.
/// Implementation:
/// - Loads and displays the Ferris logo (Rust mascot)
/// - Adds text overlay with "EMBEDDED GRAPHICS" and "IMAGE TEST"
/// - Uses a pre-compiled raw image file (ferris.raw)
fn test_image_rendering<T>(display: &mut T, ferris: &Image<ImageRawLE<Rgb565>>)
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running image rendering test");
    display.clear(Rgb565::BLACK).unwrap();
    
    // Draw the pre-loaded Ferris logo
    ferris.draw(display).unwrap();
    
    // Add text overlay for identification
    let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    Text::new(
        "EMBEDDED GRAPHICS",
        Point::new(10, 20),
        small_text_style,
    )
    .draw(display)
    .unwrap();
    
    Text::new(
        "IMAGE TEST",
        Point::new(80, 220),
        small_text_style,
    )
    .draw(display)
    .unwrap();
}

/// Test Case 6: Screen Fill Performance Test
/// 
/// Purpose: Measure and display the time required to fill the screen with different colors.
/// Implementation:
/// - Cycles through 7 different colors
/// - For each color:
///   * Measures time to fill the entire screen
///   * Displays the measured time in milliseconds
///   * Shows a test label
/// - Pauses for 1 second between colors
async fn test_screen_fill<T>(display: &mut T)
where 
    T: DrawTarget<Color = Rgb565>,
    T::Error: core::fmt::Debug,
{
    info!("Running screen fill performance test");
    
    // Colors to test (primary, secondary, and white)
    let colors = [
        Rgb565::RED,
        Rgb565::GREEN,
        Rgb565::BLUE,
        Rgb565::YELLOW,
        Rgb565::CYAN,
        Rgb565::MAGENTA,
        Rgb565::WHITE,
    ];
    
    // Test each color
    for &color in colors.iter() {
        // Measure time to fill screen with current color
        let start = embassy_time::Instant::now();
        display.clear(color).unwrap();
        let duration = embassy_time::Instant::now() - start;
        
        // Clear screen to prepare for timing display
        display.clear(Rgb565::BLACK).unwrap();
        
        // Define text styles for timing display
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
        let small_text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
        
        // Display test title
        Text::new(
            "FILL TEST",
            Point::new(10, 50),
            text_style,
        )
        .draw(display)
        .unwrap();
        
        // Display "TIME: " label
        Text::new(
            "TIME: ",
            Point::new(10, 100),
            text_style,
        )
        .draw(display)
        .unwrap();
        
        // Convert timing to string (avoiding format! macro for no_std compatibility)
        let ms = duration.as_millis() as u16;
        let hundreds = ms / 100;
        let tens = (ms % 100) / 10;
        let ones = ms % 10;
        
        // Create a string representation of the time (e.g., "123ms")
        let mut time_str = [0u8; 8];
        time_str[0] = b'0' + hundreds as u8;
        time_str[1] = b'0' + tens as u8;
        time_str[2] = b'0' + ones as u8;
        time_str[3] = b'm';
        time_str[4] = b's';
        
        // Display the timing value
        if let Ok(time_str) = core::str::from_utf8(&time_str[..5]) {
            Text::new(
                time_str,
                Point::new(80, 100),
                text_style,
            )
            .draw(display)
            .unwrap();
        }
        
        // Display test identification label
        Text::new(
            "SCREEN FILL TEST",
            Point::new(10, 200),
            small_text_style,
        )
        .draw(display)
        .unwrap();
        
        // Wait before testing next color
        Timer::after(Duration::from_millis(1000)).await;
    }
}

/// Main application entry point
/// 
/// Initializes the hardware and runs the test suite in a loop.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize the RP2350 chip with default settings
    let p = embassy_rp::init(Default::default());
    info!("Hello World! Starting display tests...");
    
    // Pin assignments based on Waveshare Pico-ResTouch LCD 2.8" wiring
    let bl = p.PIN_13;    // LCD backlight control
    let rst = p.PIN_15;   // LCD reset pin
    let display_cs = p.PIN_9;  // LCD chip select
    let dcx = p.PIN_8;    // Data/Command control (0=command, 1=data)
    let miso = p.PIN_12;  // SPI MISO (Master In Slave Out)
    let mosi = p.PIN_11;  // SPI MOSI (Master Out Slave In)
    let clk = p.PIN_10;   // SPI clock
    let touch_cs = p.PIN_16; // Touch controller chip select
    //let touch_irq = p.PIN_17; // Touch interrupt (not used in this example)
    
    // Configure SPI parameters for display
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ; // 64 MHz for fast display updates
    // SPI mode 3 (CPOL=1, CPHA=1) - Idle high, capture on second edge
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;
    
    // Configure SPI parameters for touch controller
    let mut touch_config = spi::Config::default();
    touch_config.frequency = TOUCH_FREQ; // 200 kHz for touch (lower speed for stability)
    // Note: Other touch_config parameters could be set here if needed
    
    // Initialize SPI peripheral with touch configuration (will be shared)
    let spi: Spi<'static, _, Blocking> = Spi::new_blocking(p.SPI1, clk, mosi, miso, touch_config.clone());
    
    // Create a mutex-protected SPI bus for safe sharing between display and touch
    let spi_bus = SPI_BUS.init(Mutex::new(RefCell::new(spi)));
    
    // Create SPI devices with their respective configurations
    // Display SPI device with display-specific configuration
    let display_spi = SpiDeviceWithConfig::new(spi_bus, Output::new(display_cs, Level::High), display_config);
    // Touch SPI device with touch-specific configuration (not used in this example)
    let _touch_spi = SpiDeviceWithConfig::new(spi_bus, Output::new(touch_cs, Level::High), touch_config);
    
    // Initialize control pins
    let dcx = Output::new(dcx, Level::Low);  // Start in command mode
    let rst = Output::new(rst, Level::Low);  // Start in reset state
    
    // Enable LCD backlight
    let _bl = Output::new(bl, Level::High);
    
    // Create display interface abstraction
    let di = SPIInterface::new(display_spi, dcx);
    
    // Initialize the ST7789 display
    let mut display = Builder::new(ST7789, di)
        .display_size(240, 240)          // Physical display is 320x240, but we use 240x240
        .reset_pin(rst)                  // Use the reset pin we configured
        .orientation(Orientation::new().rotate(Rotation::Deg0)) // No rotation
        .init(&mut Delay)                // Initialize with delay implementation
        .unwrap();
    
    // Clear display to black
    display.clear(Rgb565::BLACK).unwrap();
    
    let raw_image_rp = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/rp.raw"), 240);
    let rp = Image::new(&raw_image_rp, Point::new(0, 0));

    let raw_image_1 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/1.raw"), 240);
    let im1 = Image::new(&raw_image_1, Point::new(0, 0));

    let raw_image_2 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/2.raw"), 240);
    let im2 = Image::new(&raw_image_2, Point::new(0, 0));

    let raw_image_3 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/3.raw"), 240);
    let im3 = Image::new(&raw_image_3, Point::new(0, 0));

    let raw_image_4 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/4.raw"), 240);
    let im4 = Image::new(&raw_image_4, Point::new(0, 0));

    let raw_image_5 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/5.raw"), 240);
    let im5 = Image::new(&raw_image_5, Point::new(0, 0));
    
    let raw_image_6 = ImageRawLE::<Rgb565>::new(include_bytes!("../assets/mode_6/6.raw"), 240);
    let im6 = Image::new(&raw_image_6, Point::new(0, 0));
    display.clear(Rgb565::BLACK).unwrap();
    let delay_interval = 500; // 500 ms delay between tests
    loop {
    rp.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;
    
    im1.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;

    im2.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;

    im3.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;

    im4.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;

    im5.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;

    im6.draw(&mut display).unwrap();
    Timer::after_millis(delay_interval).await;
        // // Test 1: Color accuracy verification
        // test_color_accuracy(&mut display);
        // // Timer::after(Duration::from_secs(3)).await; // 3-second pause between tests
        // Timer::after_millis(1000).await;
        
        // // Test 2: Geometric primitive drawing
        // test_geometric_primitives(&mut display);
        // // Timer::after(Duration::from_secs(3)).await;
        // Timer::after_millis(1000).await;

        
        // // Test 3: Text rendering capabilities
        // test_text_rendering(&mut display);
        // // Timer::after(Duration::from_secs(3)).await;
        // Timer::after_millis(100).await;

        
        // // Test 4: Animation smoothness verification
        // test_animation(&mut display).await;
        
        // // Test 5: Image rendering test
        // test_image_rendering(&mut display, &ferris);
        // // Timer::after(Duration::from_secs(3)).await;
        // Timer::after_millis(100).await;

        
        // Test 6: Screen fill performance measurement
        // test_screen_fill(&mut display).await;
    }
}
```
# 注意事项
- 你的屏幕驱动可能跟驱动的RGB映射不同，跟ST7789芯片相关
  - 一个解决办法就是用脚本生成不同模式的*.raw文件，然后一个一个试
    - 这也是比较快的一个办法，如果你看不懂驱动和芯片手册的话
- 工程里的converter.py文件干的就是这个事情
  - 使用的时候仅需在工程下创建一个imgs文件，里面存放你的原始文件，最好是先裁剪到你的屏幕尺寸大小
  - 处理好图片之后，直接运行脚本即可，若提示需要依赖未安装，先执行
```bash
pip install pillow
```
- 你的屏幕驱动（ST7789）可能跟我用的也不一样
  - 若是无法显示预设颜色，先尝试生成好的八种模式