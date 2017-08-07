# `cortex-m-quickstart`

> A template for building applications for ARM Cortex-M microcontrollers

# [Documentation](https://docs.rs/cortex-m-quickstart)

# License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

## Instructions
- Disable incremental compilation. It doesn't work for embedded development.
You'll hit nonsensical linker errors if you use it.

``` text
$ unset CARGO_INCREMENTAL
```

- Build the application

 ``` text
# NOTE this command requires `arm-none-eabi-ld` to be in $PATH
$ xargo build --release
```

- Flash the program

``` text
# Launch OpenOCD on a terminal
$ openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg
```

``` text
# Start a debug session in another terminal
$ arm-none-eabi-gdb target/..
```

**NOTE** As of nightly-2017-05-14 or so and cortex-m-quickstart v0.1.6 you
can simply run `cargo run` or `cargo run --example $example` to build the
program, and immediately start a debug session. IOW, it lets you omit the
`arm-none-eabi-gdb` command.

``` text
$ cargo run --example hello
> # drops you into a GDB session
```
