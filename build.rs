use std::fs;

fn main() {
    let service_paths: Vec<_> = glob::glob("services/KRPC.SpaceCenter.json")
        .expect("the kRPC service schema pattern must be valid")
        .filter_map(Result::ok)
        .collect();

    for path in &service_paths {
        println!("cargo:rerun-if-changed={}", path.display());
    }

    fs::create_dir_all("src/services").expect("could not create generated service directory");
    krpc_mars_terraformer::run(&service_paths, "src/services")
        .expect("could not generate Rust kRPC service bindings");
}
