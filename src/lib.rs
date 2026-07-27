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

    /// Engage the autopilot pointing straight up, throttle up, and stage until
    /// the engines are actually producing thrust.
    ///
    /// A single `activate_next_stage()` call isn't always enough: some craft
    /// have a decoupler, fairing, or launch-clamp-only stage sitting above the
    /// stage that actually ignites the engines, so thrust can still read zero
    /// right after staging once. But re-staging isn't always the right fix
    /// either: kRPC's own docs on `Control.throttle` note that reading it back
    /// right after a set can still return the previous value for one physics
    /// tick, and a revert-to-launch can also leave the throttle reset from a
    /// previous attempt. If the throttle itself is zero, no engine will ever
    /// show thrust no matter how many stages get activated, so check and fix
    /// that first - staging again in that situation would activate whatever
    /// comes after the engine in the sequence (e.g. a decoupler) before the
    /// engine ever got a real chance to run.
    pub fn begin_vertical_ascent(&mut self, throttle: f32) -> Result<(), Box<dyn Error>> {
        let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
        let control = self.rpc.mk_call(&vessel.get_control())?;
        let auto_pilot = self.rpc.mk_call(&vessel.get_auto_pilot())?;
        let surface_frame = self.rpc.mk_call(&vessel.get_surface_reference_frame())?;

        self.rpc.mk_call(&auto_pilot.set_reference_frame(surface_frame))?;
        self.rpc.mk_call(&auto_pilot.set_target_direction((1.0, 0.0, 0.0)))?;
        self.rpc.mk_call(&auto_pilot.set_engaged(true))?;

        self.rpc.mk_call(&control.set_throttle(throttle))?;
        self.rpc.mk_call(&control.activate_next_stage())?;

        const MAX_IGNITION_RECOVERY_ATTEMPTS: u32 = 5;
        const IGNITION_SETTLE_TIME: Duration = Duration::from_millis(500);

        for _ in 0..MAX_IGNITION_RECOVERY_ATTEMPTS {
            thread::sleep(IGNITION_SETTLE_TIME);

            if self.rpc.mk_call(&vessel.get_available_thrust())? > 0.0 {
                return Ok(());
            }

            if self.rpc.mk_call(&control.get_throttle())? <= 0.0 {
                self.rpc.mk_call(&control.set_throttle(throttle))?;
            } else {
                self.rpc.mk_call(&control.activate_next_stage())?;
            }
        }

        Err("No stage produced thrust after several activations; check the vessel's staging.".into())
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
    /// `begin_vertical_ascent` does.
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

/// Automatically stages when commanded thrust is lost for a sustained period.
///
/// Only call [`StagingMonitor::check`] while the script actually wants thrust
/// (for example during a powered ascent or burn). It treats near-zero available
/// thrust as "this stage is spent", which would misfire during an intentional
/// coast with the throttle already at zero.
pub struct StagingMonitor {
    min_thrust_n: f32,
    confirm_duration: Duration,
    thrust_loss_since: Option<Instant>,
}

impl StagingMonitor {
    /// `min_thrust_n` is the available-thrust threshold, in Newtons, below which
    /// the current stage is considered spent. `confirm_duration` is how long that
    /// condition must persist before staging, to avoid reacting to a brief engine
    /// spool-up or a single noisy telemetry read.
    pub fn new(min_thrust_n: f32, confirm_duration: Duration) -> Self {
        Self {
            min_thrust_n,
            confirm_duration,
            thrust_loss_since: None,
        }
    }

    /// Check the current available thrust and stage if it has been too low for
    /// too long. Returns `true` if staging was triggered on this call.
    pub fn check(
        &mut self,
        ksp: &mut KspClient,
        available_thrust_n: f32,
    ) -> Result<bool, Box<dyn Error>> {
        if available_thrust_n >= self.min_thrust_n {
            self.thrust_loss_since = None;
            return Ok(false);
        }

        let loss_started = *self.thrust_loss_since.get_or_insert_with(Instant::now);

        if loss_started.elapsed() < self.confirm_duration {
            return Ok(false);
        }

        ksp.stage()?;
        self.thrust_loss_since = None;

        Ok(true)
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
}