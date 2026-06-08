fn main() {
    // Linker script is specified via Cargo config rustflags
    println!("cargo:rerun-if-changed=link.lds");
    println!("cargo:rerun-if-changed=src/asm/startup.s");
}
