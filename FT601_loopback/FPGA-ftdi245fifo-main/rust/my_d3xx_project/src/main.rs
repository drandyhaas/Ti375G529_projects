// src/main.rs
use d3xx::list_devices;

fn main() {
    match list_devices() {
        Ok(devices) => {
            if devices.is_empty() {
                println!("No D3XX devices found.");
            } else {
                println!("Found D3XX devices:");
                for (i, device) in devices.iter().enumerate() {
                    println!("  Device {}: {:?}", i, device);
                }
            }
        }
        Err(e) => {
            eprintln!("Error listing devices: {}", e);
        }
    }
}
