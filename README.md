### Overview
This will generate an rtic project for an stm32f103c8 microcontroller. Adjustments to the desired toolchain or microcontroller will require changes to the configuration.

For RTIC documentation, Go to the [book](https://rtic.rs/0.5/book/en/) and the [docs](https://docs.rs/cortex-m-rtic/0.5.5/rtic/)

### Requirements:

Download the correct target via `rustup  ` (follow this guide if you dont have rustup: https://doc.rust-lang.org/book/ch01-01-installation.html )
Do this by running `rustup target add thumbv7m-none-eabi`

Install [`cargo-flash`](https://github.com/probe-rs/cargo-flash), [`cargo-embed`](https://github.com/probe-rs/cargo-embed), and [`cargo-generate`](https://github.com/ashleygwilliams/cargo-generate).

`Cargo` will handle the rest (you might have to install the rust `nightly` toolchain. Just look up how to do that)

### How to generate:
Run the following command to generate a project:
`cargo generate --git https://github.com/BBScholar/STM32F103-RTIC-template.git --name my-project`

### How to deploy
Run `cargo embed --release` to deploy the binary to the microcontroller
