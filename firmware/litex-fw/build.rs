use std::env;
use std::fs::File;
use std::io::{Write, BufReader};
use std::path::Path;
use xml::reader::{EventReader, XmlEvent};

fn main() {
    // LiteX build path for this board.
    let build_dir = env::var("BUILD_DIR").expect("No build dir");

    // Rust compilation output path.
    let out_dir = env::var("OUT_DIR").expect("No out dir");

    let dest_path = Path::new(&out_dir);
    let mut f = File::create(&dest_path.join("litex_svd_extensions.rs")).expect("Could not create file");

    // Extract some parameters from the .svf that svd2rust doesn't support.
    let svd_path = concat!(env!("BUILD_DIR"), "/csr.svd");
    let file = File::open(svd_path).expect("Failed to open file");
    let file_reader = BufReader::new(file);
    let parser = EventReader::new(file_reader);

    for event in parser {
        match event {
            Ok(XmlEvent::StartElement { name, attributes, .. }) => {
                if name.local_name == "constant" {
                    let mut found_name: Option<String> = None;
                    let mut found_value: Option<u32> = None;
                    for attr in attributes {
                        if attr.name.local_name == "name" {
                            found_name = Some(attr.value)
                        } else if attr.name.local_name == "value" {
                            found_value = attr.value.parse().ok()
                        }
                    }
                    match (found_name, found_value) {
                        (Some(n), Some(v)) => {
                            f.write(format!("const {} = {}usize;\n", n, v).as_bytes());
                        }
                        _ => {}
                    }
                } else if name.local_name == "memoryRegion" {
                    for attr in attributes {
                        println!("Attribute: {}={}", attr.name, attr.value);
                    }
                }
            }
            Err(e) => {
                panic!("SVD parse error: {}", e);
                break;
            }
            _ => {}
        }
    }



    // Put the linker script somewhere the linker can find it.
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
