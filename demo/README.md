# STM32F303 discovery kit demo by TJ

An attempt to create an interpretation of the OOTB STM32F303 discovery kit demo firmware in Rust.

The USER button has two types of operation, long press => change mode, and short press => change aspect of current mode

I am using this to learn Rust while also having a go at embedded programming on Cortex-M.
The Rust Embedded book, which also has a companion book specifically on the STM32F3xx discovery kit.
Another useful resource was the RTIC v2 book. This project is based on RTIC v2.

 * https://docs.rust-embedded.org/book/intro/index.html
 * https://docs.rust-embedded.org/discovery/f3discovery/index.html
 * https://rtic.rs/2/book/en/preface.html

## Modes / Demos

 * Spin1, Spin3, Spin5, Spin7
 * Magnetometer compass
 * Accelerometer indicator
 
### SpinX

Spin 1, 3, 5 or 7 LEDs. USER button short press will cycle through 15 speeds in one direction, and then 15 speeds in the other direction

### Magneto

Light up LED pointing north. USER button short presses currently do nothing.

NB: To speed sensor calibration turn the device around once, when in this mode.

### Accelero 

Light up LED in direction of sharp movement/acceleration for a short period. USER button short presses currently do nothing.
