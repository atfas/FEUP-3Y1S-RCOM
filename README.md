# RCOM — Serial Port Data Transfer

Data link protocol in C for transferring files between two computers over an RS-232 serial port. Built for **RCOM (Redes de Computadores / Computer Networks)** at FEUP, L.EIC 3rd year, 1st semester (2025/2026). Group project.

The program runs as a transmitter (`tx`) or a receiver (`rx`). The transmitter reads a file and sends it frame by frame, and the receiver rebuilds it on the other side, even when the cable is unplugged or noise is injected during the transfer.

## What we implemented

### Link layer ([`src/link_layer.c`](src/link_layer.c))

- **`llopen`**: connection setup with a `SET` / `UA` handshake.
- **`llwrite`**: builds information frames (`FLAG | A | C | BCC1 | data | BCC2 | FLAG`) with byte stuffing (`0x7E` and `0x7D` escaped with `0x7D` and `XOR 0x20`) and sends them using **Stop-and-Wait** with alternating sequence numbers.
- **`llread`**: state machine that validates the header (`BCC1`), destuffs the payload, checks `BCC2`, and replies with `RR` (accepted) or `REJ` (rejected, asks for a retransmission). Duplicate frames are acknowledged and discarded.
- **`llclose`**: connection teardown with a `DISC` / `DISC` / `UA` exchange.
- **Timeouts and retransmissions**: a `SIGALRM` timer resends a frame when no answer arrives, up to a configured number of tries (3 tries, 4-second timeout by default).

### Application layer ([`src/application_layer.c`](src/application_layer.c))

- Sends a **START** control packet with the file size and file name (TLV-encoded), then the file split into **data packets** of up to 910 bytes, then an **END** control packet.
- The receiver writes the data packets to the output file until it gets the END packet.

## Project structure

| Path | Contents |
|---|---|
| `src/` | Link layer and application layer implementation, serial port helpers and `main.c` |
| `cable/` | Virtual cable program used to test without real serial hardware (provided, unchanged) |
| `bin/` | Compiled binaries |
| `Makefile` | Builds the project and runs the transmitter, receiver and cable (provided, unchanged) |
| `penguin.gif` | Sample file to send through the serial port |

## How to run

The virtual cable needs [`socat`](http://www.dest-unreach.org/socat/) installed.

1. Build the application and the virtual cable:

   ```bash
   make
   ```

2. Start the virtual cable:

   ```bash
   sudo make run_cable
   ```

   (or `sudo ./bin/cable`)

3. Test the protocol without disconnections or noise.

   In one terminal, start the receiver:

   ```bash
   make run_rx
   ```

   (or `./bin/main /dev/ttyS11 9600 rx penguin-received.gif`)

   In another terminal, start the transmitter:

   ```bash
   make run_tx
   ```

   (or `./bin/main /dev/ttyS10 9600 tx penguin.gif`)

   Then check that the received file matches the original:

   ```bash
   make check_files
   ```

   (or `diff -s penguin.gif penguin-received.gif`)

4. Test the protocol with disconnections and noise: run the receiver and transmitter again, then quickly switch to the cable program's console and press `0` to unplug the cable, `2` to add noise and `1` to go back to normal. The received file should still match the original.

Supported baud rates: 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200.
