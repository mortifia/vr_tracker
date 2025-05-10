rustup toolchain install nightly --component rust-src
rustup target add riscv32imac-unknown-none-elf # For ESP32-C6 and ESP32-H2
rustup override set nightly 
cargo install espup
espup install

cargo install espflash ldproxy cargo-espflash


```powershell
    irm https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.ps1 | iex
```


rustup component add llvm-tools-preview








# command

- cargo-embed → pour flasher et logger via RTT
- cargo-flash → pour flasher simplement
- probe-rs → interface CLI bas-niveau
- probe-rs-debugger (intégré dans probe-rs)