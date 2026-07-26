use std::error::Error;

use ksp_krpc::KspClient;

fn main() -> Result<(), Box<dyn Error>> {
    let mut ksp = KspClient::connect("KSP Rust vessel info")?;
    let vessel = ksp.active_vessel_info()?;

    println!("Active vessel: {}", vessel.name);
    println!("Type: {}", vessel.vessel_type);
    println!("Situation: {}", vessel.situation);
    println!("Mass: {:.1} kg", vessel.mass_kg);
    println!(
        "Mission elapsed time: {:.1} s",
        vessel.mission_elapsed_time_s
    );

    Ok(())
}
