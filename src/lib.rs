//! High-level Rust access to a locally running Kerbal Space Program kRPC server.
//!
//! The generated [`services`] module mirrors kRPC's low-level service definitions.
//! Most mission code should instead begin with [`KspClient`] and add focused methods
//! here as it needs more rocket behavior.

pub mod services;

use std::error::Error;

use services::space_center;

/// The locally configured kRPC RPC endpoint.
pub const DEFAULT_RPC_ADDRESS: &str = "127.0.0.1:50000";

/// A small, application-focused view of the vessel currently selected in KSP.
#[derive(Debug, Clone)]
pub struct VesselInfo {
    /// The vessel's in-game display name.
    pub name: String,
    /// The type kRPC assigns to the vessel, such as `Ship` or `Probe`.
    pub vessel_type: String,
    /// The current flight state, such as `PreLaunch`, `Flying`, or `Orbiting`.
    pub situation: String,
    /// Total vessel mass in kilograms.
    pub mass_kg: f64,
    /// Time in seconds since the vessel launched.
    pub mission_elapsed_time_s: f64,
}

/// A connection to the kRPC server that backs higher-level rocket operations.
pub struct KspClient {
    rpc: krpc_mars::RPCClient,
}

impl KspClient {
    /// Connect to the local kRPC server using the default RPC endpoint.
    pub fn connect(client_name: &str) -> Result<Self, Box<dyn Error>> {
        Self::connect_to(client_name, DEFAULT_RPC_ADDRESS)
    }

    /// Connect to a kRPC RPC endpoint, useful when kRPC uses a non-default address.
    pub fn connect_to(client_name: &str, address: &str) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            rpc: krpc_mars::RPCClient::connect(client_name, address)?,
        })
    }

    /// Read basic telemetry for the vessel selected in the KSP flight scene.
    pub fn active_vessel_info(&mut self) -> Result<VesselInfo, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;

        let (name, vessel_type, situation, mass_kg, mission_elapsed_time_s) = krpc_mars::batch_call!(
            &mut self.rpc,
            (
                &vessel.get_name(),
                &vessel.get_type(),
                &vessel.get_situation(),
                &vessel.get_mass(),
                &vessel.get_met(),
            )
        )?;

        Ok(VesselInfo {
            name: name?,
            vessel_type: format!("{:?}", vessel_type?),
            situation: format!("{:?}", situation?),
            mass_kg: mass_kg?.into(),
            mission_elapsed_time_s: mission_elapsed_time_s?,
        })
    }
}
