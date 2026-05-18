# RASPBERRY PI 4B FADEC

## Compatible Boards and Firmware
**ADS131M04 4-Channel 24-bit ADC DAQ Board**
**Version:** v0.5 (2026-05-18)  
**MCU:** Raspberry Pi Pico 2 (RP2350)  
**ADC:** Texas Instruments ADS131M04

## ADC Board Real-Time Data Port (RTDP) Implementation
The **Real-Time Data Port** is a dedicated low-latency path that lets an external master device poll the **latest ADC sample set** from the Pico while the main acquisition loop continues logging independently on Core 0.

In the current setup, the **Pico acts as the SPI slave** and the **Raspberry Pi 4B FADEC acts as the SPI master**. The Pi watches `DATA_RDY`, asserts `CSn`, waits briefly for the slave/transceiver path to settle, and then clocks out one RTDP frame.

### Key Features
- Runs RTDP service on **Core 1** so the main sampling/logging loop on Core 0 is left alone.
- Uses **ping-pong DMA buffers** so the newest sample set can be handed to the SPI slave path with minimal CPU work.
- Exposes the **freshest 4-channel ADC sample set** rather than the full logging buffer.
- Data format is **13 bytes per frame**: `0xEB` sync header + `4 × 24-bit signed` channel values, big-endian.
- Uses timeout protection so the Pico can recover if the master does not respond.
- Drains stale SPI state between polls and de-initialises SPI if the master disappears.
- Enabled/disabled via virtual register 7 (`vregister[7]` = RTDP timeout / advertising window in µs).

### Hardware Pins & Required Circuitry
| GPIO | Pin Name       | Direction | Function                              | Notes |
|------|----------------|-----------|---------------------------------------|-------|
| 17   | SPI0_CSn       | Input     | Chip Select (active low)              | Pulled up on Pico2 |
| 18   | SPI0_SCK       | Input     | Serial Clock                          | Pulled up on Pico2 |
| 19   | SPI0_TX (MISO) | Output    | Data output to external master        | RTDP serial data |
| 27   | DATA_RDY       | Output    | New data available signal             | Goes high when fresh RTDP frame is ready |
| 20   | RTDP_DE        | Output    | RS-485 Driver Enable                  | High = transmit |
| 21   | RTDP_REn       | Output    | RS-485 Receiver Enable (active low)   | Low = receiver enabled |

### FADEC External Hardware
- **ISL83491 transceiver**
  - Pico GPIO19 -> DI (driver input)
  - Transceiver A/B differential pair -> FADEC master
  - DE -> GPIO20
  - /RE -> GPIO21

**SPI Mode:** Mode 3 (CPOL=1, CPHA=1)  
**Pico SPI0 slave configured for:** up to 20 MHz in firmware  
**Pi FADEC currently using:** 10 MHz

### RTDP Frame Format

Each RTDP response is exactly **13 bytes**:

| Byte(s) | Meaning |
|---------|---------|
| 0 | Sync header = `0xEB` |
| 1..3 | Channel 0, signed 24-bit, big-endian |
| 4..6 | Channel 1, signed 24-bit, big-endian |
| 7..9 | Channel 2, signed 24-bit, big-endian |
| 10..12 | Channel 3, signed 24-bit, big-endian |

The Pi validates byte 0 against `0xEB`. Frames with any other first byte are treated as misaligned or invalid and are dropped.

### How the RTDP Works

**Core 0 (sampling loop):**
- Every new ADC sample set is stored in the main circular logging buffer.
- If RTDP is enabled and Core 1 is idle, Core 0 packs the newest sample set into the next RTDP ping-pong buffer.
- The packed buffer format is:
  - byte 0 = `0xEB`
  - bytes 1..12 = 4 signed 24-bit ADC values, big-endian
- Core 0 then pushes the ready buffer index to Core 1 via the inter-core FIFO.

**Core 1 (RTDP service):**
1. Waits for a buffer index from Core 0.
2. (Re)initialises SPI0 as a **Mode 3 slave** if needed.
3. Drains any stale SPI RX FIFO state from previous traffic.
4. Preloads byte 0 (`0xEB`) into the SPI TX FIFO before advertising readiness.
5. Configures TX DMA for the remaining bytes (1..12).
6. Configures RX DMA to drain the receive side so the full-duplex SPI peripheral does not stall.
7. Asserts **DATA_RDY**.
8. Waits for the external master to pull **CSn** low.
9. Enables the ISL83491 driver only after **CSn** goes low, so the shared RS-485 bus is not driven early.
10. Waits for the full transfer and for **CSn** to return high.
11. Drains residual SPI state, disables the transceiver, clears **DATA_RDY**, and returns to idle.

If the external master does **not** respond within the timeout (`vregister[7]`), the Pico aborts the transfer, cleans up the SPI state, and waits for the next sample.

### Timing Notes

- The first byte is the most timing-sensitive part of the transfer.
- The Pico now stages `0xEB` into the SPI TX FIFO before `DATA_RDY` is asserted so the header is already present when clocking begins.
- The Pi FADEC code currently inserts about **2 µs** between asserting **CSn** and starting SCK.
- That CS-setup delay is intended to cover both:
  - SPI slave first-byte setup
  - ISL83491 driver-enable latency
- In practice, keeping at least **1–2 µs** of CS-setup delay is recommended for reliable frame alignment.

### What RTDP Is And Is Not

RTDP is the **live-view / latest-sample path**. It is used when the external Pi wants the most recent ADC values quickly and repeatedly.

RTDP is **not** the main bulk-recording path. The normal logging loop still writes to SRAM independently, and RTDP simply exposes the newest sample set in parallel.

### Example External Master Sequence

```c
// Pseudo-code for external device
while (true) {
    if (DATA_RDY == HIGH) {
        pull CSn LOW;
        delay_us(2);            // allow slave + transceiver path to settle
        uint8_t rx[13];
        spi_transfer(rx, 13);   // dummy TX bytes are ignored by the slave
        release CSn;

        if (rx[0] == 0xEB) {
            process_live_data(rx);  // decode 4 × signed 24-bit big-endian values
        }
    }
}
```
