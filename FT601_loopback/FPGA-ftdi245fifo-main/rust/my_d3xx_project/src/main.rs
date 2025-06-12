use std::io::{Read, Write};
use d3xx::{list_devices, Pipe};

use std::time::Instant;

fn main() {

    // Scan for connected devices.
    let all_devices = list_devices().expect("failed to list devices");

    // Open the first device found.
    let device = all_devices[0].open().expect("failed to open device");

    let start = Instant::now(); // Start the timer
    //for read_iteration in 1..100 {

    // Convert to big-endian byte array
    let num_bytes_to_read: u32 = 1000_000_000;
    let big_endian_bytes = num_bytes_to_read.to_be_bytes();
    let data_to_write: [u8; 4] = big_endian_bytes;
    println!("\nAttempting to write 4 bytes: {:?}", data_to_write);

    let bytes_written = device.pipe(Pipe::Out0).write(&data_to_write).unwrap();

    // The write_pipe function returns the number of bytes successfully written.
    if bytes_written == data_to_write.len() {
        println!("Successfully wrote {} bytes.", bytes_written);
    } else {
        // This could happen if the device's buffer is full and a timeout occurs.
        println!(
            "Warning: Wrote {} bytes, but expected to write {}.",
            bytes_written,
            data_to_write.len()
        );
    }

    // --- Step 6: Read 10,000,000 Bytes in Chunks ---
    const TOTAL_BYTES_TO_READ: usize = 1000_000_000;
    // Using a large buffer on the stack can cause a stack overflow.
    // It's safer to allocate on the heap with a Vec and read in manageable chunks.
    let mut read_buffer = Vec::with_capacity(TOTAL_BYTES_TO_READ);
    let mut total_bytes_read = 0;
    println!("\nAttempting to read {} bytes...", TOTAL_BYTES_TO_READ);

    // Loop to read data in chunks until the target amount is reached or a timeout occurs.
    // The d3xx driver itself handles chunking at a lower level, but this application-level
    // loop ensures we get the total amount we expect.
    while total_bytes_read < TOTAL_BYTES_TO_READ {
        // Create a temporary buffer for the next chunk of data.
        // We try to read up to the remaining amount, with a reasonable max chunk size (e.g. 64KB).
        let chunk_size = std::cmp::min(65536*1000, TOTAL_BYTES_TO_READ - total_bytes_read);
        let mut chunk = vec![0; chunk_size];

        match device.pipe(Pipe::In0).read(&mut chunk) {
            Ok(bytes_in_chunk) => {
                if bytes_in_chunk == 0 {
                    // This typically means the read timed out. The device may have no more data.
                    println!("\nRead operation finished early (timeout or end of data).");
                    break;
                }
                // Add the read bytes to our main buffer.
                read_buffer.extend_from_slice(&chunk[..bytes_in_chunk]);
                total_bytes_read += bytes_in_chunk;
            }
            Err(e) => {
                // An unrecoverable error occurred.
                eprintln!("\nError reading from pipe after {} bytes: {}", total_bytes_read, e);
                // We'll break and process what we have.
                break;
            }
        };
    }
    println!("Total bytes read: {}", total_bytes_read);

    // It's often useful to print a small portion of the read data to verify it.
    //if read_iteration==0 {
    if total_bytes_read > 0 {
        // We'll print the first 16 bytes, or fewer if we didn't read that many.
        let preview_len = std::cmp::min(total_bytes_read, 16);
        println!("Data preview (first {} bytes):", preview_len);
        // Format the output as hex values.
        for (i, byte) in read_buffer.iter().take(preview_len).enumerate() {
            print!("{:02X} ", byte);
            if (i + 1) % 8 == 0 {
                println!(); // Newline every 8 bytes for readability
            }
        }
        println!();
    } else {
        println!("No data was read from the device. This could be expected or indicate an issue.");
    }
    //}

    //}

    let duration = start.elapsed(); // Get the elapsed time
    println!("Time taken: {:?}, {}ms, {} MB/s", duration, duration.as_millis(), 1000000.0/(duration.as_millis() as f32)); // Print the duration
    println!("\nDemonstration complete.");
}
