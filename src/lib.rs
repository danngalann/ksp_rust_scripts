//! High-level Rust access to a locally running Kerbal Space Program kRPC server.
//!
//! The generated [`services`] module mirrors kRPC's low-level service definitions.
//! Most mission code should instead begin with [`KspClient`] and add focused methods
//! here as it needs more rocket behavior.

pub mod services;

use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use services::space_center;

/// The locally configured kRPC RPC endpoint.
pub const DEFAULT_RPC_ADDRESS: &str = "127.0.0.1:50000";

/// Standard gravity, in meters per second squared, used to convert between
/// specific impulse (seconds) and exhaust velocity, and as the reference for
/// "g" units of acceleration. This is the conventional constant (9.80665 m/s^2)
/// used throughout KSP's own delta-v/TWR readouts, not any body's actual surface
/// gravity.
pub const STANDARD_GRAVITY_MS2: f64 = 9.80665;

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

/// The launch body's name and atmosphere height above mean sea level.
#[derive(Debug, Clone)]
pub struct BodyInfo {
    pub name: String,
    /// Zero for an airless body; otherwise the atmosphere depth in meters.
    pub atmosphere_depth_m: f64,
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

    /// Read the body currently orbited by the active vessel.
    pub fn current_body_info(&mut self) -> Result<BodyInfo, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;
        let body = self.rpc.mk_call(&orbit.get_body())?;
        let (name, atmosphere_depth_m) = krpc_mars::batch_call!(
            &mut self.rpc,
            (&body.get_name(), &body.get_atmosphere_depth())
        )?;

        Ok(BodyInfo {
            name: name?,
            atmosphere_depth_m: atmosphere_depth_m?,
        })
    }

    /// Engage the autopilot pointing straight up and set launch throttle.
    ///
    /// Staging is deliberately separate. Mission code must use an explicit
    /// [`PropulsionStagePlan`] so a failed ignition cannot repeatedly advance
    /// through unrelated decoupler, fairing, payload, or recovery stages.
    pub fn prepare_vertical_ascent(&mut self, throttle: f32) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;
        let surface_frame = self.rpc.mk_call(&vessel.get_surface_reference_frame())?;

        self.rpc.mk_call(&auto_pilot.set_reference_frame(surface_frame))?;
        self.rpc.mk_call(&auto_pilot.set_target_direction((1.0, 0.0, 0.0)))?;
        self.rpc.mk_call(&auto_pilot.set_engaged(true))?;

        self.rpc.mk_call(&control.set_throttle(throttle))?;

        Ok(())
    }


    /// Read the vessel's current altitude above the surface or sea level, in meters.
    pub fn surface_altitude_m(&mut self) -> Result<f64, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let surface_frame = self.rpc.mk_call(&vessel.get_surface_reference_frame())?;
        let flight = self.rpc.mk_call(&vessel.flight(surface_frame))?;

        Ok(self.rpc.mk_call(&flight.get_surface_altitude())?)
    }

    /// Read vertical and horizontal surface-relative speed, in meters per second.
    pub fn surface_speeds_ms(&mut self) -> Result<(f64, f64), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;
        let body = self.rpc.mk_call(&orbit.get_body())?;
        let body_frame = self.rpc.mk_call(&body.get_reference_frame())?;
        let flight = self.rpc.mk_call(&vessel.flight(body_frame))?;

        let vertical_speed_ms = self.rpc.mk_call(&flight.get_vertical_speed())?;
        let horizontal_speed_ms = self.rpc.mk_call(&flight.get_horizontal_speed())?;

        Ok((vertical_speed_ms, horizontal_speed_ms))
    }

    /// Read aerodynamic dynamic pressure, in pascals.
    pub fn dynamic_pressure_pa(&mut self) -> Result<f32, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let surface_frame = self.rpc.mk_call(&vessel.get_surface_reference_frame())?;
        let flight = self.rpc.mk_call(&vessel.flight(surface_frame))?;

        Ok(self.rpc.mk_call(&flight.get_dynamic_pressure())?)
    }

    /// Read the vessel's total currently available thrust, in Newtons.
    pub fn available_thrust_n(&mut self) -> Result<f32, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;

        Ok(self.rpc.mk_call(&vessel.get_available_thrust())?)
    }

    /// Activate the next stage. Equivalent to pressing space bar in-game.
    pub fn stage(&mut self) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;

        self.rpc.mk_call(&control.activate_next_stage())?;

        Ok(())
    }

    /// KSP's internal stage cursor. The next Space press activates one less.
    pub fn current_stage(&mut self) -> Result<i32, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;

        Ok(self.rpc.mk_call(&control.get_current_stage())?)
    }

    /// Maximum engine thrust assigned to the next pending UI stage, in newtons.
    ///
    /// `Vessel.stage_at` only exposes stock delta-v activation stages and can
    /// reject a valid mixed stage, such as engines activated alongside launch
    /// clamps. KSP's internal `current_stage` is one greater than the stage
    /// activated by the next Space press, so inspect engine parts assigned to
    /// `current_stage - 1` and sum `Engine.max_thrust`. Maximum thrust describes
    /// capacity without requiring inactive fuel-flow state to be initialized.
    pub fn next_stage_max_thrust_n(&mut self) -> Result<(i32, f32), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;
        let parts = self.rpc.mk_call(&vessel.get_parts())?;
        let current_stage = self.rpc.mk_call(&control.get_current_stage())?;
        let pending_stage = pending_stage_number(current_stage).ok_or(
            "KSP reports no pending stage; throttle cut without staging.",
        )?;
        let engines = self.rpc.mk_call(&parts.get_engines())?;
        let mut stage_max_thrust_n = 0.0;

        for engine in engines {
            let part = self.rpc.mk_call(&engine.get_part())?;
            let (engine_stage, max_thrust_n) = krpc_mars::batch_call!(
                &mut self.rpc,
                (&part.get_stage(), &engine.get_max_thrust())
            )?;

            if engine_stage? == pending_stage {
                stage_max_thrust_n += max_thrust_n?;
            }
        }

        Ok((pending_stage, stage_max_thrust_n))
    }

    /// Classify active engine state independently from the vessel throttle.
    pub fn active_propulsion_state(
        &mut self,
        min_thrust_n: f32,
    ) -> Result<ActivePropulsionState, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let parts = self.rpc.mk_call(&vessel.get_parts())?;
        let engines = self.rpc.mk_call(&parts.get_engines())?;
        let mut active_engine_found = false;

        for engine in engines {
            let (active, has_fuel, flameout, available_thrust_n) = krpc_mars::batch_call!(
                &mut self.rpc,
                (
                    &engine.get_active(),
                    &engine.get_has_fuel(),
                    &engine.get_flameout(),
                    &engine.get_available_thrust(),
                )
            )?;

            if active? {
                active_engine_found = true;
                if has_fuel? && !flameout? && available_thrust_n? >= min_thrust_n {
                    return Ok(ActivePropulsionState::Usable);
                }
            }
        }

        if active_engine_found {
            Ok(ActivePropulsionState::Spent)
        } else {
            Ok(ActivePropulsionState::NoActiveEngines)
        }
    }

    /// Set the active vessel's main throttle, from 0.0 through 1.0.
    pub fn set_throttle(&mut self, throttle: f32) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;

        self.rpc.mk_call(&control.set_throttle(throttle))?;

        Ok(())
    }

    /// Command the autopilot toward a pitch (degrees above horizon) and heading (compass degrees).
    ///
    /// Assumes the autopilot is already engaged with a reference frame set, as
    /// `prepare_vertical_ascent` does.
    pub fn set_target_pitch_and_heading(&mut self, pitch: f32, heading: f32) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;

        self.rpc.mk_call(&auto_pilot.target_pitch_and_heading(pitch, heading))?;

        Ok(())
    }

    /// Read the vessel's current apoapsis altitude above sea level, in meters.
    pub fn apoapsis_altitude_m(&mut self) -> Result<f64, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;

        Ok(self.rpc.mk_call(&orbit.get_apoapsis_altitude())?)
    }

    /// Read the vessel's current periapsis altitude above sea level, in meters.
    pub fn periapsis_altitude_m(&mut self) -> Result<f64, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;

        Ok(self.rpc.mk_call(&orbit.get_periapsis_altitude())?)
    }

    /// Seconds until the vessel reaches its orbit's apoapsis.
    pub fn time_to_apoapsis_s(&mut self) -> Result<f64, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;

        Ok(self.rpc.mk_call(&orbit.get_time_to_apoapsis())?)
    }

    /// The current universal time, in seconds.
    pub fn current_ut_s(&mut self) -> Result<f64, Box<dyn Error>> {
        Ok(self.rpc.mk_call(&space_center::get_ut())?)
    }

    /// Whether regular "on-rails" time warp can currently be used at all.
    ///
    /// KSP disallows on-rails warp (falling back to slower physical warp, or no
    /// warp) in various situations, most commonly while the active vessel is
    /// still inside an atmosphere. Check this before calling [`KspClient::warp_to`]
    /// if you want to avoid a call that - since physical warp is always capped
    /// at 1x here - would otherwise just block in real time without actually
    /// speeding anything up.
    pub fn can_rails_warp(&mut self) -> Result<bool, Box<dyn Error>> {
        const MIN_RAILS_WARP_FACTOR: i32 = 1;

        Ok(self.rpc.mk_call(&space_center::can_rails_warp_at(MIN_RAILS_WARP_FACTOR))?)
    }

    /// Warp forward in time to the given universal time. Blocks until reached.
    ///
    /// Always caps physical time warp at its normal (1x, i.e. effectively no
    /// acceleration) rate, so it can never actually speed up time via physical
    /// warp - KSP switches to physical warp automatically whenever the active
    /// vessel is traveling through an atmosphere, and physical warp can throw a
    /// vessel around or desync from the autopilot in ways on-rails warp can't.
    /// Regular "on-rails" warp is left uncapped. Only call this while coasting
    /// with no attitude or throttle changes needed, since the call blocks for
    /// the whole warp.
    pub fn warp_to(&mut self, ut: f64) -> Result<(), Box<dyn Error>> {
        const MAX_RAILS_RATE: f32 = 100_000.0;
        const MAX_PHYSICS_RATE: f32 = 1.0;

        Ok(self
            .rpc
            .mk_call(&space_center::warp_to(ut, MAX_RAILS_RATE, MAX_PHYSICS_RATE))?)
    }

    /// Read the vessel's current total mass, in kilograms.
    pub fn mass_kg(&mut self) -> Result<f64, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;

        Ok(self.rpc.mk_call(&vessel.get_mass())?.into())
    }

    /// Read the vessel's current average specific impulse of active engines, in seconds.
    pub fn specific_impulse_s(&mut self) -> Result<f32, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;

        Ok(self.rpc.mk_call(&vessel.get_specific_impulse())?)
    }

    /// Create a maneuver node at the next apoapsis that circularizes the current orbit.
    /// Returns a handle to the created node.
    pub fn plan_circularization_node(&mut self) -> Result<space_center::Node, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;
        let orbit = self.rpc.mk_call(&vessel.get_orbit())?;
        let body = self.rpc.mk_call(&orbit.get_body())?;

        let apoapsis_radius_m = self.rpc.mk_call(&orbit.get_apoapsis())?;
        let semi_major_axis_m = self.rpc.mk_call(&orbit.get_semi_major_axis())?;
        let gravitational_parameter = self.rpc.mk_call(&body.get_gravitational_parameter())?;
        let time_to_apoapsis_s = self.rpc.mk_call(&orbit.get_time_to_apoapsis())?;
        let current_ut = self.rpc.mk_call(&space_center::get_ut())?;

        let delta_v = circularization_delta_v_ms(gravitational_parameter, apoapsis_radius_m, semi_major_axis_m);
        let node_ut = current_ut + time_to_apoapsis_s;

        Ok(self.rpc.mk_call(&control.add_node(node_ut, delta_v as f32, 0.0, 0.0))?)
    }

    /// Remaining delta-v of a maneuver node, in meters per second. Decreases as it burns.
    pub fn node_remaining_delta_v(&mut self, node: space_center::Node) -> Result<f64, Box<dyn Error>> {
        Ok(self.rpc.mk_call(&node.get_remaining_delta_v())?)
    }

    /// Remaining burn direction and magnitude for a maneuver node, in the vessel's
    /// surface reference frame (the same frame the autopilot is already using).
    pub fn node_remaining_burn_vector(
        &mut self,
        node: space_center::Node,
    ) -> Result<(f64, f64, f64), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let surface_frame = self.rpc.mk_call(&vessel.get_surface_reference_frame())?;

        Ok(self.rpc.mk_call(&node.remaining_burn_vector(surface_frame))?)
    }

    /// Signed remaining prograde delta-v in the node's orbital frame, in m/s.
    ///
    /// The orbital frame's y-axis is prograde. A circularization node requests
    /// positive y; crossing through zero therefore identifies an overshoot even
    /// if the scalar remaining delta-v has begun increasing again.
    pub fn node_remaining_prograde_ms(
        &mut self,
        node: space_center::Node,
    ) -> Result<f64, Box<dyn Error>> {
        let orbital_frame = self.rpc.mk_call(&node.get_orbital_reference_frame())?;
        let (_, prograde_ms, _) = self.rpc.mk_call(&node.remaining_burn_vector(orbital_frame))?;

        Ok(prograde_ms)
    }

    /// Seconds until the maneuver node occurs.
    pub fn node_time_to(&mut self, node: space_center::Node) -> Result<f64, Box<dyn Error>> {
        Ok(self.rpc.mk_call(&node.get_time_to())?)
    }

    /// Point the autopilot toward a raw direction vector, in its current reference frame.
    pub fn set_target_direction(&mut self, direction: (f64, f64, f64)) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;

        Ok(self.rpc.mk_call(&auto_pilot.set_target_direction(direction))?)
    }

    /// Angular error between the commanded autopilot direction and vessel, in degrees.
    pub fn autopilot_error_deg(&mut self) -> Result<f32, Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;

        Ok(self.rpc.mk_call(&auto_pilot.get_error())?)
    }

    /// Remove a maneuver node once it's no longer needed.
    pub fn remove_node(&mut self, node: space_center::Node) -> Result<(), Box<dyn Error>> {
        Ok(self.rpc.mk_call(&node.remove())?)
    }

    /// Cut throttle and release autopilot control, used when an ascent must be aborted.
    pub fn abort_ascent(&mut self) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;

        self.rpc.mk_call(&control.set_throttle(0.0))?;
        self.rpc.mk_call(&auto_pilot.set_engaged(false))?;

        Ok(())
    }
}

/// A bounded list of propulsion-stage activations allowed by the mission.
///
/// Unlike a generic thrust-loss trigger, this plan checks engine state before
/// staging, verifies that KSP predicts usable thrust for the next activation
/// stage, and confirms that activation restores actual thrust. The activation
/// count includes the initial launch stage.
pub struct PropulsionStagePlan {
    remaining_activations: u32,
    min_thrust_n: f32,
    confirm_duration: Duration,
    ignition_timeout: Duration,
    thrust_loss_since: Option<Instant>,
    observed_stage_cursor: Option<i32>,
    manual_stage_change_observed: bool,
}

impl PropulsionStagePlan {
    pub fn new(
        propulsion_stage_count: u32,
        min_thrust_n: f32,
        confirm_duration: Duration,
        ignition_timeout: Duration,
    ) -> Self {
        Self {
            remaining_activations: propulsion_stage_count,
            min_thrust_n,
            confirm_duration,
            ignition_timeout,
            thrust_loss_since: None,
            observed_stage_cursor: None,
            manual_stage_change_observed: false,
        }
    }

    /// Activate one configured propulsion stage and wait for usable thrust.
    pub fn activate_next(&mut self, ksp: &mut KspClient) -> Result<(), Box<dyn Error>> {
        let (pending_stage, next_stage_max_thrust_n) = ksp.next_stage_max_thrust_n()?;
        let next_stage_has_engine_capacity = stage_has_engine_capacity(
            next_stage_max_thrust_n,
            self.min_thrust_n,
        );
        match staging_decision(self.remaining_activations, next_stage_has_engine_capacity) {
            StagingDecision::Activate => {}
            StagingDecision::AbortNoConfiguredStage => {
                ksp.set_throttle(0.0)?;
                return Err("No configured propulsion stage remains; throttle cut before staging.".into());
            }
            StagingDecision::AbortNonPropulsiveStage => {
                ksp.set_throttle(0.0)?;
                return Err(format!(
                    "Engines assigned to pending stage {pending_stage} have only {next_stage_max_thrust_n:.1} N of maximum thrust; throttle cut without staging."
                )
                .into());
            }
        }

        // Stage with liquid-engine throttle closed so a new upper stage cannot
        // inherit a burn command before mission code has rechecked attitude and
        // acceleration. `available_thrust` still reports full-throttle capability.
        ksp.set_throttle(0.0)?;
        ksp.stage()?;
        self.remaining_activations -= 1;
        self.observed_stage_cursor = Some(ksp.current_stage()?);
        self.manual_stage_change_observed = false;
        let ignition_started = Instant::now();

        while ignition_started.elapsed() < self.ignition_timeout {
            if ksp.available_thrust_n()? >= self.min_thrust_n {
                self.thrust_loss_since = None;
                return Ok(());
            }
            thread::sleep(Duration::from_millis(100));
        }

        ksp.set_throttle(0.0)?;
        Err("The configured propulsion stage did not produce usable thrust before the ignition timeout; throttle cut.".into())
    }

    /// Activate the next configured propulsion stage after a manual stage press.
    ///
    /// This is safe to call while orienting for a maneuver: it responds only to
    /// an observed stage-cursor change followed by no active engines. It does
    /// not perform ordinary spent-stage automation during a coast.
    pub fn reconcile_manual_stage_transition(
        &mut self,
        ksp: &mut KspClient,
        available_thrust_n: f32,
    ) -> Result<bool, Box<dyn Error>> {
        self.observe_stage_cursor(ksp)?;

        if available_thrust_n >= self.min_thrust_n {
            self.manual_stage_change_observed = false;
            return Ok(false);
        }

        if self.manual_stage_change_observed
            && ksp.active_propulsion_state(self.min_thrust_n)?
                == ActivePropulsionState::NoActiveEngines
        {
            self.activate_next(ksp)?;
            return Ok(true);
        }

        Ok(false)
    }

    fn observe_stage_cursor(&mut self, ksp: &mut KspClient) -> Result<(), Box<dyn Error>> {
        let current_stage = ksp.current_stage()?;
        if self
            .observed_stage_cursor
            .is_some_and(|previous_stage| previous_stage != current_stage)
        {
            self.manual_stage_change_observed = true;
        }
        self.observed_stage_cursor = Some(current_stage);

        Ok(())
    }

    /// Stage only after active engine telemetry confirms sustained exhaustion.
    /// Returns `true` when a configured propulsion stage was activated.
    pub fn check_for_spent_stage(
        &mut self,
        ksp: &mut KspClient,
        available_thrust_n: f32,
    ) -> Result<bool, Box<dyn Error>> {
        self.observe_stage_cursor(ksp)?;

        if available_thrust_n >= self.min_thrust_n {
            self.thrust_loss_since = None;
            self.manual_stage_change_observed = false;
            return Ok(false);
        }

        let loss_started = *self.thrust_loss_since.get_or_insert_with(Instant::now);

        if loss_started.elapsed() < self.confirm_duration {
            return Ok(false);
        }

        let propulsion_state = ksp.active_propulsion_state(self.min_thrust_n)?;
        match thrust_loss_staging_decision(
            propulsion_state,
            self.manual_stage_change_observed,
        ) {
            ThrustLossStagingDecision::ActivateNextPropulsionStage => {}
            ThrustLossStagingDecision::AbortUnexpectedThrustLoss => {
                self.thrust_loss_since = None;
                ksp.set_throttle(0.0)?;
                return Err("Sustained thrust loss was neither confirmed engine exhaustion nor a manual stage transition; throttle cut without staging.".into());
            }
        }

        self.activate_next(ksp)?;
        self.thrust_loss_since = None;

        Ok(true)
    }
}

/// Result of checking whether an automatic propulsion-stage activation is safe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StagingDecision {
    Activate,
    AbortNoConfiguredStage,
    AbortNonPropulsiveStage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActivePropulsionState {
    Usable,
    Spent,
    NoActiveEngines,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrustLossStagingDecision {
    ActivateNextPropulsionStage,
    AbortUnexpectedThrustLoss,
}

pub fn thrust_loss_staging_decision(
    propulsion_state: ActivePropulsionState,
    manual_stage_change_observed: bool,
) -> ThrustLossStagingDecision {
    match propulsion_state {
        ActivePropulsionState::Spent => {
            ThrustLossStagingDecision::ActivateNextPropulsionStage
        }
        ActivePropulsionState::NoActiveEngines if manual_stage_change_observed => {
            ThrustLossStagingDecision::ActivateNextPropulsionStage
        }
        ActivePropulsionState::Usable | ActivePropulsionState::NoActiveEngines => {
            ThrustLossStagingDecision::AbortUnexpectedThrustLoss
        }
    }
}

pub fn stage_has_engine_capacity(stage_max_thrust_n: f32, min_thrust_n: f32) -> bool {
    stage_max_thrust_n >= min_thrust_n
}

/// Convert KSP's internal stage cursor to the stage activated by the next press.
pub fn pending_stage_number(current_stage: i32) -> Option<i32> {
    current_stage.checked_sub(1).filter(|stage| *stage >= 0)
}

pub fn staging_decision(
    remaining_activations: u32,
    next_stage_has_engine_capacity: bool,
) -> StagingDecision {
    if remaining_activations == 0 {
        StagingDecision::AbortNoConfiguredStage
    } else if !next_stage_has_engine_capacity {
        StagingDecision::AbortNonPropulsiveStage
    } else {
        StagingDecision::Activate
    }
}

/// Why a maneuver burn should stop on the current control tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BurnCompletion {
    InTolerance,
    Overshot,
}

/// Classify maneuver progress using both scalar precision and signed prograde error.
pub fn burn_completion(
    remaining_delta_v_ms: f64,
    remaining_prograde_ms: f64,
    tolerance_ms: f64,
) -> Option<BurnCompletion> {
    if remaining_delta_v_ms <= tolerance_ms {
        Some(BurnCompletion::InTolerance)
    } else if remaining_prograde_ms <= 0.0 {
        Some(BurnCompletion::Overshot)
    } else {
        None
    }
}

/// Mission-level classification after the circularization node is removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrbitOutcome {
    Successful,
    SafeButOffTarget,
    Unsafe,
}

/// Classify altitudes measured above mean sea level, all in meters.
pub fn classify_orbit(
    apoapsis_altitude_m: f64,
    periapsis_altitude_m: f64,
    target_altitude_m: f64,
    atmosphere_depth_m: f64,
    safety_margin_m: f64,
    target_tolerance_m: f64,
) -> OrbitOutcome {
    if periapsis_altitude_m <= atmosphere_depth_m + safety_margin_m {
        return OrbitOutcome::Unsafe;
    }

    let apoapsis_on_target =
        (apoapsis_altitude_m - target_altitude_m).abs() <= target_tolerance_m;
    let periapsis_on_target =
        (periapsis_altitude_m - target_altitude_m).abs() <= target_tolerance_m;

    if apoapsis_on_target && periapsis_on_target {
        OrbitOutcome::Successful
    } else {
        OrbitOutcome::SafeButOffTarget
    }
}

/// Stateful max-Q limiter with hysteresis to avoid rapid throttle oscillation.
#[derive(Debug, Default)]
pub struct DynamicPressureLimiter {
    limiting: bool,
}

impl DynamicPressureLimiter {
    /// Return a throttle ceiling between 0 and 1 for the current pressure.
    ///
    /// Once pressure exceeds `max_dynamic_pressure_pa`, limiting remains active
    /// until it falls below `release_ratio * max_dynamic_pressure_pa`. While
    /// active, at least a 20% reduction is commanded; higher pressure tightens
    /// the cap proportionally.
    pub fn throttle_limit(
        &mut self,
        dynamic_pressure_pa: f32,
        max_dynamic_pressure_pa: f32,
        release_ratio: f32,
    ) -> f32 {
        if self.limiting {
            if dynamic_pressure_pa < max_dynamic_pressure_pa * release_ratio {
                self.limiting = false;
            }
        } else if dynamic_pressure_pa > max_dynamic_pressure_pa {
            self.limiting = true;
        }

        if self.limiting {
            (max_dynamic_pressure_pa / dynamic_pressure_pa.max(1.0)).clamp(0.1, 0.8)
        } else {
            1.0
        }
    }

    pub fn is_limiting(&self) -> bool {
        self.limiting
    }
}

/// A safety net that force-cuts the vessel's throttle when dropped.
///
/// Rust runs a value's [`Drop`] implementation automatically when it goes out of
/// scope, whether that happens because a function returned normally, returned an
/// error via `?`, or panicked. Holding one of these alive for the duration of a
/// launch script guarantees the throttle gets cut on every exit path, instead of
/// relying on every early return to remember to do it. kRPC does not zero the
/// throttle when a client disconnects, so without this an interrupted script can
/// leave a vessel burning until it runs out of fuel.
pub struct ThrottleGuard;

impl Drop for ThrottleGuard {
    fn drop(&mut self) {
        // Reconnect with a fresh client: `main`'s client may already be borrowed,
        // moved, or (in a crash scenario) in an unknown state. Errors are ignored
        // here because a `Drop` implementation cannot propagate a `Result` - this
        // is a best-effort last resort, not the primary safety mechanism.
        if let Ok(mut ksp) = KspClient::connect("KSP Rust throttle safety net") {
            let _ = ksp.set_throttle(0.0);
        }
    }
}

/// Smooth baseline gravity-turn pitch, in degrees above the horizon.
///
/// Uses smoothstep interpolation so the commanded turn rate starts and ends
/// gently instead of changing at a constant rate all the way through the turn.
pub fn pitch_for_altitude(altitude_m: f64, start_altitude_m: f64, end_altitude_m: f64) -> f32 {
    if altitude_m <= start_altitude_m {
        return 90.0;
    }

    if altitude_m >= end_altitude_m {
        return 0.0;
    }

    let progress = (altitude_m - start_altitude_m) / (end_altitude_m - start_altitude_m);
    let smooth_progress = progress * progress * (3.0 - 2.0 * progress);

    (90.0 * (1.0 - smooth_progress)) as f32
}

/// Adaptive gravity-turn pitch based on the smooth altitude schedule and live flight state.
///
/// Apoapsis feedback raises the nose when orbital altitude is developing more
/// slowly than the baseline schedule and lowers it when developing too quickly.
/// The final command remains within `max_angle_of_attack_deg` of the surface-relative
/// velocity direction, limiting drag and aerodynamic stress during atmospheric flight.
pub fn adaptive_ascent_pitch(
    altitude_m: f64,
    start_altitude_m: f64,
    end_altitude_m: f64,
    apoapsis_altitude_m: f64,
    target_apoapsis_m: f64,
    vertical_speed_ms: f64,
    horizontal_speed_ms: f64,
    max_angle_of_attack_deg: f64,
) -> f32 {
    let baseline_pitch_deg =
        pitch_for_altitude(altitude_m, start_altitude_m, end_altitude_m) as f64;

    if altitude_m <= start_altitude_m {
        return 90.0;
    }

    let altitude_progress =
        ((altitude_m - start_altitude_m) / (end_altitude_m - start_altitude_m)).clamp(0.0, 1.0);
    let expected_apoapsis_progress =
        altitude_progress * altitude_progress * (3.0 - 2.0 * altitude_progress);
    let actual_apoapsis_progress = (apoapsis_altitude_m / target_apoapsis_m).clamp(0.0, 1.0);
    let apoapsis_correction_deg =
        ((expected_apoapsis_progress - actual_apoapsis_progress) * 15.0).clamp(-10.0, 10.0);
    let corrected_pitch_deg = (baseline_pitch_deg + apoapsis_correction_deg).clamp(0.0, 90.0);

    let surface_speed_ms = vertical_speed_ms.hypot(horizontal_speed_ms);
    if surface_speed_ms < 1.0 {
        return corrected_pitch_deg as f32;
    }

    let flight_path_pitch_deg = vertical_speed_ms
        .max(0.0)
        .atan2(horizontal_speed_ms.max(0.0))
        .to_degrees();
    let minimum_pitch_deg = (flight_path_pitch_deg - max_angle_of_attack_deg).max(0.0);
    let maximum_pitch_deg = (flight_path_pitch_deg + max_angle_of_attack_deg).min(90.0);

    corrected_pitch_deg.clamp(minimum_pitch_deg, maximum_pitch_deg) as f32
}

/// Delta-v, in meters per second, for a prograde burn at apoapsis that circularizes
/// an orbit with the given apoapsis radius, semi-major axis (both from the body's
/// center, in meters), and the body's gravitational parameter (μ, in m^3/s^2).
pub fn circularization_delta_v_ms(
    gravitational_parameter: f64,
    apoapsis_radius_m: f64,
    semi_major_axis_m: f64,
) -> f64 {
    let v_circular = (gravitational_parameter / apoapsis_radius_m).sqrt();
    let v_current =
        (gravitational_parameter * (2.0 / apoapsis_radius_m - 1.0 / semi_major_axis_m)).sqrt();

    v_circular - v_current
}

/// Throttle fraction, clamped between 0.0 and `max_throttle`, that keeps
/// acceleration at or below `max_acceleration_ms2` given the vessel's current
/// mass and its fully-throttled available thrust.
///
/// Assumes thrust scales linearly with throttle, which is a reasonable
/// approximation for most rocket engines away from their spool-up transient.
pub fn throttle_for_max_acceleration(
    available_thrust_n: f32,
    mass_kg: f64,
    max_acceleration_ms2: f64,
    max_throttle: f32,
) -> f32 {
    if available_thrust_n <= 0.0 {
        return 0.0;
    }

    let accel_limited_throttle = (max_acceleration_ms2 * mass_kg / available_thrust_n as f64) as f32;

    accel_limited_throttle.clamp(0.0, max_throttle)
}

/// Estimated duration, in seconds, of a burn of `delta_v_ms` performed with the
/// given specific impulse (seconds), available thrust (Newtons), and starting
/// mass (kilograms).
///
/// Derived from the Tsiolkovsky rocket equation: for exhaust velocity
/// $v_e = I_{sp} g_0$, a burn of $\Delta v$ leaves final mass
/// $m_1 = m_0 / e^{\Delta v / v_e}$. Dividing the burned mass by the mass flow
/// rate ($\dot m = F / v_e$) gives the burn time. Assumes constant thrust and
/// specific impulse for the whole burn, which is reasonable for a short
/// circularization burn but not for one that meaningfully drains the stage.
pub fn burn_duration_s(
    delta_v_ms: f64,
    specific_impulse_s: f64,
    available_thrust_n: f64,
    initial_mass_kg: f64,
) -> f64 {
    let effective_exhaust_velocity_ms = specific_impulse_s * STANDARD_GRAVITY_MS2;
    let final_mass_kg = initial_mass_kg / (delta_v_ms / effective_exhaust_velocity_ms).exp();
    let mass_flow_rate_kg_s = available_thrust_n / effective_exhaust_velocity_ms;

    (initial_mass_kg - final_mass_kg) / mass_flow_rate_kg_s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pitch_for_altitude_clamps_and_interpolates() {
        assert_eq!(pitch_for_altitude(0.0, 250.0, 45_000.0), 90.0);
        assert_eq!(pitch_for_altitude(250.0, 250.0, 45_000.0), 90.0);
        assert_eq!(pitch_for_altitude(45_000.0, 250.0, 45_000.0), 0.0);
        assert_eq!(pitch_for_altitude(100_000.0, 250.0, 45_000.0), 0.0);

        let midpoint = (250.0 + 45_000.0) / 2.0;
        let pitch = pitch_for_altitude(midpoint, 250.0, 45_000.0);

        assert!((pitch - 45.0).abs() < 0.01);
    }

    #[test]
    fn smooth_pitch_stays_more_vertical_early_and_turns_more_late() {
        let turn_range = 45_000.0 - 250.0;

        assert!(pitch_for_altitude(250.0 + turn_range * 0.25, 250.0, 45_000.0) > 67.5);
        assert!(pitch_for_altitude(250.0 + turn_range * 0.75, 250.0, 45_000.0) < 22.5);
    }

    #[test]
    fn adaptive_pitch_is_bounded_near_velocity_direction() {
        let pitch = adaptive_ascent_pitch(
            20_000.0, 250.0, 45_000.0, 30_000.0, 80_000.0, 300.0, 300.0, 5.0,
        );

        assert!((40.0..=50.0).contains(&pitch));
    }

    #[test]
    fn adaptive_pitch_stays_vertical_before_turn_start() {
        let pitch = adaptive_ascent_pitch(
            100.0, 250.0, 45_000.0, 500.0, 80_000.0, 10.0, 0.0, 5.0,
        );

        assert_eq!(pitch, 90.0);
    }

    #[test]
    fn adaptive_pitch_uses_baseline_when_speed_is_unavailable() {
        let pitch = adaptive_ascent_pitch(
            800.0, 250.0, 45_000.0, 1_500.0, 80_000.0, 0.0, 0.0, 5.0,
        );

        assert!(pitch > 85.0);
    }

    #[test]
    fn circularization_delta_v_is_zero_for_already_circular_orbit() {
        let gravitational_parameter = 3.5316e12;
        let radius_m = 670_000.0 + 80_000.0;

        let delta_v = circularization_delta_v_ms(
            gravitational_parameter,
            radius_m,
            radius_m,
        );

        assert!(delta_v.abs() < 0.01);
    }

    #[test]
    fn throttle_for_max_acceleration_limits_and_clamps() {
        // 10,000 N available, 1,000 kg -> full throttle gives 10 m/s^2.
        // Limiting to 4 m/s^2 should cap throttle at 0.4.
        let throttle = throttle_for_max_acceleration(10_000.0, 1_000.0, 4.0, 1.0);
        assert!((throttle - 0.4).abs() < 0.001);

        // A generous acceleration limit should still respect max_throttle.
        let throttle = throttle_for_max_acceleration(10_000.0, 1_000.0, 100.0, 0.85);
        assert!((throttle - 0.85).abs() < 0.001);

        // No available thrust means no meaningful throttle to command.
        assert_eq!(throttle_for_max_acceleration(0.0, 1_000.0, 4.0, 1.0), 0.0);
    }

    #[test]
    fn burn_duration_matches_rocket_equation() {
        // A 500 kg vessel with a 300s Isp engine producing 5,000 N of thrust.
        let duration = burn_duration_s(50.0, 300.0, 5_000.0, 500.0);

        // Mass flow rate = thrust / (Isp * g0) ~= 1.698 kg/s.
        // Burned mass = 500 * (1 - e^(-50 / (300 * g0))) ~= 8.43 kg.
        // Expected duration ~= 5.0 s.
        assert!((duration - 5.0).abs() < 0.1);
    }

    #[test]
    fn staging_requires_budget_and_a_propulsive_next_stage() {
        assert_eq!(staging_decision(1, true), StagingDecision::Activate);
        assert_eq!(
            staging_decision(0, true),
            StagingDecision::AbortNoConfiguredStage
        );
        assert_eq!(
            staging_decision(1, false),
            StagingDecision::AbortNonPropulsiveStage
        );
        assert!(!stage_has_engine_capacity(0.0, 1.0));
        assert!(stage_has_engine_capacity(825_000.0, 1.0));
        assert_eq!(pending_stage_number(4), Some(3));
        assert_eq!(pending_stage_number(1), Some(0));
        assert_eq!(pending_stage_number(0), None);
    }

    #[test]
    fn manual_interstage_allows_next_propulsion_ignition() {
        assert_eq!(
            thrust_loss_staging_decision(ActivePropulsionState::NoActiveEngines, true),
            ThrustLossStagingDecision::ActivateNextPropulsionStage
        );
        assert_eq!(
            thrust_loss_staging_decision(ActivePropulsionState::NoActiveEngines, false),
            ThrustLossStagingDecision::AbortUnexpectedThrustLoss
        );
        assert_eq!(
            thrust_loss_staging_decision(ActivePropulsionState::Spent, false),
            ThrustLossStagingDecision::ActivateNextPropulsionStage
        );
    }

    #[test]
    fn burn_completion_detects_tolerance_and_signed_overshoot() {
        assert_eq!(
            burn_completion(0.4, 0.4, 0.5),
            Some(BurnCompletion::InTolerance)
        );
        assert_eq!(
            burn_completion(1.2, -0.1, 0.5),
            Some(BurnCompletion::Overshot)
        );
        assert_eq!(burn_completion(1.2, 1.0, 0.5), None);
    }

    #[test]
    fn orbit_classification_distinguishes_target_safe_and_unsafe() {
        assert_eq!(
            classify_orbit(80_000.0, 79_000.0, 80_000.0, 70_000.0, 5_000.0, 2_000.0),
            OrbitOutcome::Successful
        );
        assert_eq!(
            classify_orbit(90_000.0, 78_000.0, 80_000.0, 70_000.0, 5_000.0, 2_000.0),
            OrbitOutcome::SafeButOffTarget
        );
        assert_eq!(
            classify_orbit(80_000.0, 74_000.0, 80_000.0, 70_000.0, 5_000.0, 2_000.0),
            OrbitOutcome::Unsafe
        );
    }

    #[test]
    fn dynamic_pressure_limiter_uses_hysteresis() {
        let mut limiter = DynamicPressureLimiter::default();

        assert_eq!(limiter.throttle_limit(24_000.0, 25_000.0, 0.9), 1.0);
        assert!(limiter.throttle_limit(26_000.0, 25_000.0, 0.9) <= 0.8);
        assert!(limiter.is_limiting());
        assert!(limiter.throttle_limit(23_000.0, 25_000.0, 0.9) <= 0.8);
        assert_eq!(limiter.throttle_limit(22_000.0, 25_000.0, 0.9), 1.0);
        assert!(!limiter.is_limiting());
    }
}