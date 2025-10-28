use anyhow::{Result, bail};
use clap::Parser;
use rppal::gpio::{Gpio, InputPin, OutputPin, Level};
use std::net::UdpSocket;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Trigger generator: runs only when Start button is pressed.
/// Sends a timestamp to Jetson (UDP) on every pulse and on START/STOP.
/// A green LED lights up when the trigger is active.
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// BCM GPIO pin for trigger output (default 18 = pin 12)
    #[arg(long, default_value_t = 18)]
    trig_pin: u8,

    /// BCM pin for START button (to GND, internal pull-up)
    #[arg(long, default_value_t = 23)]
    start_pin: u8,

    /// BCM pin for STOP button (to GND, internal pull-up)
    #[arg(long, default_value_t = 24)]
    stop_pin: u8,

    /// BCM pin for green status LED
    #[arg(long, default_value_t = 25)]
    led_pin: u8,

    /// Frequency in Hz
    #[arg(long, default_value_t = 30.0)]
    fps: f64,

    /// Pulse width in microseconds
    #[arg(long, default_value_t = 100u64)]
    pulse_us: u64,

    /// Jetson IP address to send timestamps to (UDP)
    #[arg(long, default_value = "192.168.1.42")]
    jetson_ip: String,

    /// Jetson UDP port
    #[arg(long, default_value_t = 9000)]
    jetson_port: u16,
}

fn busy_sleep(dur: Duration) {
    let start = Instant::now();
    while Instant::now().duration_since(start) < dur {
        std::hint::spin_loop();
    }
}

fn now_unix_us() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros()
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.fps <= 0.0 || args.fps > 5000.0 {
        bail!("fps must be in (0,5000]");
    }
    let period = Duration::from_secs_f64(1.0 / args.fps);
    let high = Duration::from_micros(args.pulse_us);
    if high >= period {
        bail!("pulse_us must be < period");
    }

    // GPIO setup
    let gpio = Gpio::new()?;
    let mut trig: OutputPin = gpio.get(args.trig_pin)?.into_output_low();
    let mut led:  OutputPin = gpio.get(args.led_pin )?.into_output_low();
    let start_btn: InputPin = gpio.get(args.start_pin)?.into_input_pullup();
    let stop_btn : InputPin = gpio.get(args.stop_pin )?.into_input_pullup();

    // UDP setup
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect(format!("{}:{}", args.jetson_ip, args.jetson_port))?;
    println!("UDP connected to {}:{}", args.jetson_ip, args.jetson_port);

    // Ctrl-C handler
    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
        }).unwrap();
    }

    let mut trigger_on = false;
    let mut seq: u64 = 0;

    println!(
        "Ready. START on BCM{}, STOP on BCM{}. Trigger pin BCM{}, LED on BCM{}.",
        args.start_pin, args.stop_pin, args.trig_pin, args.led_pin
    );

    let send_event = |sock: &UdpSocket, etype: &str, seq: u64| {
        let ts = now_unix_us();
        let _ = sock.send(format!("{ts},{etype},{seq}\n").as_bytes());
    };

    while running.load(Ordering::SeqCst) {
        // --- Buttons ---
        if start_btn.read() == Level::Low && !trigger_on {
            trigger_on = true;
            led.set_high(); // LED ON
            println!("▶ Trigger STARTED");
            send_event(&socket, "START", seq);
            thread::sleep(Duration::from_millis(200)); // debounce
        }
        if stop_btn.read() == Level::Low && trigger_on {
            trigger_on = false;
            trig.set_low();
            led.set_low(); // LED OFF
            println!("■ Trigger STOPPED");
            send_event(&socket, "STOP", seq);
            thread::sleep(Duration::from_millis(200));
        }

        // --- Trigger pulses ---
        if trigger_on {
            let next = Instant::now() + period;
            trig.set_high();
            busy_sleep(high);
            trig.set_low();

            seq = seq.wrapping_add(1);
            send_event(&socket, "PULSE", seq);

            let now = Instant::now();
            if now < next {
                thread::sleep(next - now);
            }
        } else {
            thread::sleep(Duration::from_millis(50));
        }
    }

    trig.set_low();
    led.set_low();
    println!("Exiting…");
    let _ = socket.send(format!("{},EXIT,{}\n", now_unix_us(), seq).as_bytes());
    Ok(())
}
