use std::{env, path::{PathBuf}};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("Missing OUT_DIR"));

    println!(concat!("cargo:rustc-link-search=", env!("BUILD_DIR"), "/software/libc/"));
    println!(concat!("cargo:rustc-link-search=", env!("BUILD_DIR"), "/software/libbase/"));
    println!(concat!("cargo:rustc-link-search=", env!("BUILD_DIR"), "/software/libfatfs/"));
    println!(concat!("cargo:rustc-link-search=", env!("BUILD_DIR"), "/software/liblitesdcard/"));
    println!("cargo:rustc-link-lib=c");
    println!("cargo:rustc-link-lib=base");
    println!("cargo:rustc-link-lib=litesdcard");
    println!("cargo:rustc-link-lib=fatfs");

    let stdlib_litex_include_paths = vec![
        concat!(env!("BUILD_DIR"),
                "/software/include"),
        concat!(env!("BUILD_DIR"),
                "/../../deps/litex/litex/soc/cores/cpu/vexriscv"),
        concat!(env!("BUILD_DIR"),
                "/../../deps/litex/litex/soc/software/include"),
    ];

    let clang_args = vec![
        "--target=riscv32-unknown-none-elf",
        "-fvisibility=default",
        "-fshort-enums",
    ];

    let clang_litex_include_args: Vec<String> =
        stdlib_litex_include_paths.iter().map(|s| String::from("-I") + &String::from(*s)).collect();

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let bindings = bindgen::Builder::default()
        .header(concat!(env!("BUILD_DIR"), "/../../deps/litex/litex/soc/software/liblitesdcard/sdcard.h"))
        .header(concat!(env!("BUILD_DIR"), "/../../deps/litex/litex/soc/software/libfatfs/ff.h"))
        .rustified_enum(".*")
        .clang_arg(&format!("-I{}", &out_dir.display()))
        .derive_default(true)
        .layout_tests(false)
        .use_core()
        .clang_args(&clang_args)
        .clang_args(&clang_litex_include_args)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Can't write bindings!");
}
