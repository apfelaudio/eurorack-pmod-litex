use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

/// Put the linker script somewhere the linker can find it.
fn main() {
    let out_dir = env::var("OUT_DIR").expect("No out dir");
    let dest_path = Path::new(&out_dir);

    println!("cargo:rustc-link-search=native=../libvult/build");
    println!("cargo:rustc-link-lib=static=vult");
    println!("cargo:rerun-if-changed=../libvult/build/dsp.hpp");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("../libvult/build/dsp.hpp")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .use_core()
        .allowlist_file("../libvult/build/dsp.hpp")
        .allowlist_file("../libvult/runtime/vultin.h")
        .clang_arg("-I../libvult/runtime")
        .clang_arg(
            concat!("-I", env!("BUILD_DIR"), "/software/libc"))
        .clang_arg("-I../../deps/pythondata-software-picolibc/pythondata_software_picolibc/data/newlib/libc/include")
        .clang_arg("--target=riscv32-unknown-none-elf")
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("libvult_bindings.rs"))
        .expect("Couldn't write bindings!");

    let mut f = File::create(&dest_path.join("memory.x")).expect("Could not create file");
    f.write_all(include_bytes!("memory.x"))
        .expect("Could not write file");

    let mut f = File::create(&dest_path.join("regions.ld")).expect("Could not create file");
    f.write_all(include_bytes!(concat!(
        env!("BUILD_DIR"),
        "/software/include/generated/regions.ld"
    )))
    .expect("Could not write file");

    println!("cargo:rustc-link-search={}", dest_path.display());

    println!("cargo:rerun-if-changed=regions.ld");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
