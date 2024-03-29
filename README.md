# Open Servo Tester

DIY hobby servo tester based on the
[ESP32-C3-LCDKit](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c3/esp32-c3-lcdkit/user_guide.html)
dev board.

## Hardware

### Modifications to LCDKit

This project uses the ESP32-C3-LCDKit as its base hardware. It repurposes IO4 (the kit's IR LED output pin) as the servo
signal line. A 5V regulator, battery and servo connectors, and 3D-printed case complete the hardware.

The hardware and software are still a work in progress; instructions and images to follow once the software is complete.

## Software

### Building

```
rustup target add riscv32imc-unknown-none-elf
cargo build --release
```

### Build, Flash, and Monitor Logs

You can build and flash in one command to make development easier:

```
cargo espflash flash --release --monitor
```

Optionaly remove `--monitor` to skip following the console logs.

### Build Performance Considerations

Note that the build instructions above build in release mode. You can build using the default debug mode as well, but
the result is extremely slow (so slow that the application will appear not to run at all).

### Credits and Thanks

Thanks to [@georgik](https://github.com/georgik) for [ESP32 Spooky Maze
Game](https://github.com/georgik/esp32-spooky-maze-game), which I used as starter code for this project.
