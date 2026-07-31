use std::{
    error::Error,
    io::{Error as IoError, ErrorKind},
};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use ksp_krpc::{
    adaptive_ascent_pitch, burn_completion, burn_duration_s, classify_orbit,
    throttle_for_max_acceleration, BurnCompletion, DynamicPressureLimiter, KspClient,
    OrbitOutcome, PropulsionStagePlan, ThrottleGuard, STANDARD_GRAVITY_MS2,
};

/// Command-line configuration for the launch sequence.
#[derive(Debug, Parser)]
#[command(about = "Launch the active KSP vessel through a vertical ascent phase.")]
struct LaunchSettings {
    /// Fraction of available throttle to use during ascent, from 0.0 through 1.0.
    #[arg(long, default_value_t = 1.0)]
    throttle: f32,

    /// Height above the launch site before beginning the gravity turn, in meters.
    #[arg(long, default_value_t = 250.0)]
    vertical_ascent_altitude_m: f64,

    /// Requested apoapsis altitude in kilometers, used by a future ascent phase.
    #[arg(long, default_value_t = 80.0)]
    target_apoapsis_km: f64,

    /// Maximum time to wait for the vertical-clearance altitude, in seconds.
    #[arg(long, default_value_t = 60.0)]
    ascent_timeout_s: f64,

    /// Altitude at which the gravity turn's pitch reaches minimum (horizontal), in meters.
    #[arg(long, default_value_t = 45_000.0)]
    gravity_turn_end_altitude_m: f64,

    /// Maximum acceleration to command during powered flight, in multiples of
    /// standard gravity (9.80665 m/s^2). Limits throttle after staging into a
    /// lighter or higher-thrust stage, to avoid violent attitude excursions.
    #[arg(long, default_value_t = 3.0)]
    max_acceleration_g: f64,

    /// Number of propulsion-stage activations permitted, including launch ignition.
    #[arg(long, default_value_t = 2)]
    propulsion_stage_count: u32,

    /// Maximum seconds to wait for thrust after activating a propulsion stage.
    #[arg(long, default_value_t = 3.0)]
    stage_ignition_timeout_s: f64,

    /// Maximum autopilot direction error allowed before a maneuver burn, in degrees.
    #[arg(long, default_value_t = 2.0)]
    pointing_tolerance_deg: f32,

    /// Time the pointing error must remain within tolerance before burning, in seconds.
    #[arg(long, default_value_t = 1.0)]
    pointing_settle_time_s: f64,

    /// Minimum real-time orientation window before the calculated maneuver burn start, in seconds.
    #[arg(long, default_value_t = 20.0)]
    maneuver_orientation_reserve_s: f64,

    /// Required clearance above the current body's atmosphere, in kilometers.
    #[arg(long, default_value_t = 5.0)]
    orbit_safety_margin_km: f64,

    /// Allowed final apoapsis and periapsis error from the target, in kilometers.
    #[arg(long, default_value_t = 5.0)]
    orbit_target_tolerance_km: f64,

    /// Maximum aerodynamic dynamic pressure during ascent, in pascals.
    #[arg(long, default_value_t = 25_000.0)]
    max_dynamic_pressure_pa: f32,

    /// Fraction of max-Q below which the dynamic-pressure throttle limit releases.
    #[arg(long, default_value_t = 0.9)]
    max_q_release_ratio: f32,

    /// Launch compass heading in degrees. The Kerbin eastward default is 90 degrees.
    #[arg(long, default_value_t = 90.0)]
    launch_heading_deg: f32,
}

impl LaunchSettings {
    fn validate(&self) -> Result<(), IoError> {
        if !(0.0..=1.0).contains(&self.throttle) {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--throttle must be between 0.0 and 1.0",
            ));
        }

        if self.vertical_ascent_altitude_m <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--vertical-ascent-altitude-m must be greater than zero",
            ));
        }

        if self.target_apoapsis_km <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--apoapsis-km must be greater than zero",
            ));
        }

        if self.ascent_timeout_s <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--ascent-timeout-s must be greater than zero",
            ));
        }

        if self.gravity_turn_end_altitude_m <= self.vertical_ascent_altitude_m {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--gravity-turn-end-altitude-m must be greater than --vertical-ascent-altitude-m",
            ));
        }

        if self.max_acceleration_g <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--max-acceleration-g must be greater than zero",
            ));
        }

        if self.propulsion_stage_count == 0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--propulsion-stage-count must be at least one",
            ));
        }

        if self.stage_ignition_timeout_s <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--stage-ignition-timeout-s must be greater than zero",
            ));
        }

        if self.pointing_tolerance_deg <= 0.0 || self.pointing_settle_time_s <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "pointing tolerance and settle time must be greater than zero",
            ));
        }

        if self.maneuver_orientation_reserve_s < self.pointing_settle_time_s {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--maneuver-orientation-reserve-s must be at least --pointing-settle-time-s",
            ));
        }

        if self.orbit_safety_margin_km < 0.0 || self.orbit_target_tolerance_km <= 0.0 {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "orbit safety margin must be non-negative and target tolerance must be positive",
            ));
        }

        if self.max_dynamic_pressure_pa <= 0.0
            || !(0.0..1.0).contains(&self.max_q_release_ratio)
        {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "max dynamic pressure must be positive and release ratio must be between 0 and 1",
            ));
        }

        if !(0.0..360.0).contains(&self.launch_heading_deg) {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                "--launch-heading-deg must be at least 0 and less than 360",
            ));
        }

        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    // Phase 1: validate the requested mission profile and capture preflight state.
    let settings = LaunchSettings::parse();
    settings.validate()?;

    let mut ksp = KspClient::connect("KSP Rust launch to orbit")?;
    let vessel = ksp.active_vessel_info()?;
    let body = ksp.current_body_info()?;
    let target_apoapsis_m = settings.target_apoapsis_km * 1000.0;
    let orbit_safety_margin_m = settings.orbit_safety_margin_km * 1000.0;
    let minimum_safe_orbit_m = body.atmosphere_depth_m + orbit_safety_margin_m;

    if target_apoapsis_m <= minimum_safe_orbit_m {
        return Err(IoError::new(
            ErrorKind::InvalidInput,
            format!(
                "Target orbit {:.1} km does not clear {}'s {:.1} km atmosphere plus {:.1} km safety margin.",
                settings.target_apoapsis_km,
                body.name,
                body.atmosphere_depth_m / 1000.0,
                settings.orbit_safety_margin_km,
            ),
        )
        .into());
    }

    println!("Preparing to launch: {}", vessel.name);
    println!("Situation: {}", vessel.situation);
    println!("Mass: {:.1} kg", vessel.mass_kg);
    println!("Throttle: {:.0}%", settings.throttle * 100.0);
    println!(
        "Vertical ascent target: {:.0} m",
        settings.vertical_ascent_altitude_m
    );
    println!("Target apoapsis: {:.0} km", settings.target_apoapsis_km);
    println!("Ascent timeout: {:.0} s", settings.ascent_timeout_s);
    println!("Max acceleration: {:.1} g", settings.max_acceleration_g);
    println!("Launch body: {}", body.name);
    println!("Launch heading: {:.1} deg", settings.launch_heading_deg);
    println!("Configured propulsion stages: {}", settings.propulsion_stage_count);
    println!(
        "Maneuver orientation reserve: {:.1} s",
        settings.maneuver_orientation_reserve_s
    );
    if body.atmosphere_depth_m > 0.0 {
        println!("Maximum dynamic pressure: {:.0} Pa", settings.max_dynamic_pressure_pa);
    } else {
        println!("Airless body: dynamic-pressure control disabled.");
    }

    // Cutting throttle here happens on every exit from `main` from this point on
    // (normal completion, an early `return`, or a propagated `?` error), so a
    // crash mid-burn can no longer leave the vessel's throttle stuck open.
    let _throttle_guard = ThrottleGuard;

    const MIN_THRUST_N: f32 = 1.0;
    const STAGE_CONFIRM_S: f64 = 0.5;
    const MISSED_NODE_MARGIN_S: f64 = 30.0;
    const NODE_PLANNING_LEAD_S: f64 = 90.0;
    const CIRCULARIZATION_TOLERANCE_MS: f64 = 0.5;
    const FINE_BURN_TARGET_MS: f64 = 0.25;
    const BURN_CONTROL_INTERVAL_S: f64 = 0.05;
    const MAX_ANGLE_OF_ATTACK_DEG: f64 = 5.0;

    let max_acceleration_ms2 = settings.max_acceleration_g * STANDARD_GRAVITY_MS2;
    let mut staging_plan = PropulsionStagePlan::new(
        settings.propulsion_stage_count,
        MIN_THRUST_N,
        Duration::from_secs_f64(STAGE_CONFIRM_S),
        Duration::from_secs_f64(settings.stage_ignition_timeout_s),
    );

    ksp.prepare_vertical_ascent(settings.throttle)?;
    staging_plan.activate_next(&mut ksp)?;

    let ascent_start = Instant::now();
    let mut gravity_turn_started = false;
    let mut max_q_limiter = DynamicPressureLimiter::default();

    // Phase 2: powered ascent. The controller shapes the gravity turn using
    // altitude, apoapsis progress, and current surface-relative velocity.
    loop {
        let altitude_m = ksp.surface_altitude_m()?;
        let available_thrust_n = ksp.available_thrust_n()?;
        let apoapsis_altitude_m = ksp.apoapsis_altitude_m()?;
        let (vertical_speed_ms, horizontal_speed_ms) = ksp.surface_speeds_ms()?;
        let dynamic_pressure_pa = if body.atmosphere_depth_m > 0.0 {
            ksp.dynamic_pressure_pa()?
        } else {
            0.0
        };
        let pitch = adaptive_ascent_pitch(
            altitude_m,
            settings.vertical_ascent_altitude_m,
            settings.gravity_turn_end_altitude_m,
            apoapsis_altitude_m,
            target_apoapsis_m,
            vertical_speed_ms,
            horizontal_speed_ms,
            MAX_ANGLE_OF_ATTACK_DEG,
        );

        println!(
            "Altitude: {:.1} m, apoapsis: {:.1} m, pitch: {:.1} deg, vertical speed: {:.1} m/s, horizontal speed: {:.1} m/s, thrust: {:.1} N, dynamic pressure: {:.0} Pa",
            altitude_m,
            apoapsis_altitude_m,
            pitch,
            vertical_speed_ms,
            horizontal_speed_ms,
            available_thrust_n,
            dynamic_pressure_pa,
        );

        if apoapsis_altitude_m >= target_apoapsis_m {
            ksp.set_throttle(0.0)?;
            println!("Target apoapsis reached; throttle cut. Coasting toward apoapsis.");
            break;
        }

        if staging_plan.check_for_spent_stage(&mut ksp, available_thrust_n)? {
            println!("Active engines exhausted; configured propulsion stage activated and verified.");
            continue;
        }

        if !gravity_turn_started && altitude_m >= settings.vertical_ascent_altitude_m {
            println!("Vertical ascent altitude reached; beginning gravity turn.");
            gravity_turn_started = true;
        }

        ksp.set_target_pitch_and_heading(pitch, settings.launch_heading_deg)?;

        let mass_kg = ksp.mass_kg()?;
        let acceleration_limit = throttle_for_max_acceleration(
            available_thrust_n,
            mass_kg,
            max_acceleration_ms2,
            settings.throttle,
        );
        let dynamic_pressure_limit = if body.atmosphere_depth_m > 0.0 {
            max_q_limiter.throttle_limit(
                dynamic_pressure_pa,
                settings.max_dynamic_pressure_pa,
                settings.max_q_release_ratio,
            )
        } else {
            1.0
        };
        let throttle = settings
            .throttle
            .min(acceleration_limit)
            .min(dynamic_pressure_limit);
        let active_constraint = if dynamic_pressure_limit <= acceleration_limit
            && dynamic_pressure_limit < settings.throttle
        {
            "dynamic pressure"
        } else if acceleration_limit < settings.throttle {
            "acceleration"
        } else {
            "user throttle"
        };
        println!(
            "Throttle command: {:.0}% (constraint: {})",
            throttle * 100.0,
            active_constraint
        );
        ksp.set_throttle(throttle)?;

        if !gravity_turn_started && ascent_start.elapsed().as_secs_f64() > settings.ascent_timeout_s {
            println!("Ascent timeout exceeded before reaching vertical clearance; aborting.");
            ksp.abort_ascent()?;
            return Err(IoError::new(
                ErrorKind::TimedOut,
                "Ascent timeout exceeded before vertical clearance.",
            )
            .into());
        }

        thread::sleep(Duration::from_millis(500));
    }

    // Phase 3: coast close to the real apoapsis before planning the circularization
    // node. Atmospheric drag can keep lowering the apoapsis while coasting; the
    // closer the node is planned to the actual apoapsis, the less that drift
    // matters by the time the burn happens. There's nothing to actively control
    // during this coast, so warp through it instead of polling in real time.
    // On-rails warp usually isn't available yet at this point (still inside the
    // atmosphere even though the apoapsis itself is above it), so keep retrying
    // until it becomes available instead of a single warp attempt that would
    // otherwise just block in real time without speeding anything up.
    println!("Coasting toward apoapsis before planning circularization.");
    loop {
        let time_to_apoapsis_s = ksp.time_to_apoapsis_s()?;

        if time_to_apoapsis_s <= NODE_PLANNING_LEAD_S {
            break;
        }

        if ksp.can_rails_warp()? {
            let warp_target_ut = ksp.current_ut_s()? + (time_to_apoapsis_s - NODE_PLANNING_LEAD_S);
            println!("Warping toward apoapsis.");
            ksp.warp_to(warp_target_ut)?;
        } else {
            thread::sleep(Duration::from_secs(1));
        }
    }

    println!("Planning circularization burn.");
    let node = ksp.plan_circularization_node()?;
    let planned_delta_v = ksp.node_remaining_delta_v(node)?;
    println!("Planned circularization burn: {:.1} m/s", planned_delta_v);

    // Phase 4: face the maneuver vector and continually refine the burn timing.
    println!("Orienting along the burn vector.");
    let mut burn_start_lead_s;
    let mut aligned_since: Option<Instant> = None;

    loop {
        let burn_vector = ksp.node_remaining_burn_vector(node)?;
        ksp.set_target_direction(burn_vector)?;
        let pointing_error_deg = ksp.autopilot_error_deg()?;
        if pointing_error_deg <= settings.pointing_tolerance_deg {
            aligned_since.get_or_insert_with(Instant::now);
        } else {
            aligned_since = None;
        }
        let pointing_settled = aligned_since.is_some_and(|since| {
            since.elapsed().as_secs_f64() >= settings.pointing_settle_time_s
        });

        // Recompute the burn-duration estimate every tick, instead of once
        // right after planning. Mass, available thrust, and specific impulse
        // all change if a stage separates during this coast/orient phase after
        // a manual stage press, and
        // a stale estimate would start the burn at the wrong time.
        let mass_kg = ksp.mass_kg()?;
        let available_thrust_n = ksp.available_thrust_n()?;
        if staging_plan.reconcile_manual_stage_transition(&mut ksp, available_thrust_n)? {
            println!(
                "Manual interstage transition detected; configured orbital propulsion stage activated and verified."
            );
            aligned_since = None;
            continue;
        }
        let specific_impulse_s = ksp.specific_impulse_s()?;

        // Use the throttle-limited (accel-capped) thrust the burn will actually
        // run at, not the raw available thrust - otherwise the estimate is too
        // optimistic whenever --max-acceleration-g caps the throttle below 1.0,
        // which throws off both when the burn starts and the missed-node check.
        let accel_throttle =
            throttle_for_max_acceleration(available_thrust_n, mass_kg, max_acceleration_ms2, 1.0);
        let effective_thrust_n = available_thrust_n * accel_throttle;

        let estimated_burn_s = burn_duration_s(
            planned_delta_v,
            specific_impulse_s as f64,
            effective_thrust_n as f64,
            mass_kg,
        );
        // Start the burn early enough that its estimated midpoint lands on the
        // node's planned time, instead of only starting to burn once the node time
        // arrives (which would finish the burn late and leave a slightly wrong orbit).
        burn_start_lead_s = (estimated_burn_s / 2.0).max(5.0);
        // The burn is expected to still be running up to burn_start_lead_s past
        // the node time by design (it's centered on the node), so the missed-node
        // tolerance has to scale with that instead of being a flat constant, or
        // any burn whose half-duration exceeds the constant would always trip it.
        let missed_node_tolerance_s = burn_start_lead_s + MISSED_NODE_MARGIN_S;

        let time_to_node_s = ksp.node_time_to(node)?;
        println!(
            "Time to node: {:.1} s, burn duration: {:.1} s, pointing error: {:.2} deg, settled: {}",
            time_to_node_s, estimated_burn_s, pointing_error_deg, pointing_settled
        );

        if time_to_node_s < -missed_node_tolerance_s {
            ksp.set_throttle(0.0)?;
            ksp.remove_node(node)?;
            return Err(IoError::new(
                ErrorKind::TimedOut,
                "Missed the circularization burn window while orienting; aborting.",
            )
            .into());
        }

        if time_to_node_s <= burn_start_lead_s && pointing_settled {
            break;
        }

        // Leave a real-time orientation window after warp. kRPC's autopilot
        // already adapts its control loop to the vessel; this reserve simply
        // prevents time warp from leaving it only a couple of seconds to turn.
        if time_to_node_s > burn_start_lead_s + settings.maneuver_orientation_reserve_s
            && ksp.can_rails_warp()?
        {
            let warp_target_ut =
                ksp.current_ut_s()? + (time_to_node_s - burn_start_lead_s - settings.maneuver_orientation_reserve_s);
            println!("Warping toward circularization orientation window.");
            ksp.warp_to(warp_target_ut)?;
            aligned_since = None;
            continue;
        }

        thread::sleep(Duration::from_millis(500));
    }

    // Phase 5: execute the maneuver, reducing throttle near completion for
    // accurate delivery of the final few meters per second of delta-v.
    println!("Executing circularization burn.");
    // Reuse the burn-start lead time from just before the burn as the basis for
    // how long past the node the burn is allowed to keep running - it already
    // reflects the accel-limited throttle, so a long, low-TWR burn gets a
    // correspondingly generous window instead of a flat, easily-tripped one.
    let missed_node_tolerance_s = burn_start_lead_s + MISSED_NODE_MARGIN_S;
    let mut burn_aligned_since = aligned_since;

    loop {
        let remaining_delta_v = ksp.node_remaining_delta_v(node)?;
        let remaining_prograde_ms = ksp.node_remaining_prograde_ms(node)?;
        println!(
            "Remaining delta-v: {:.2} m/s, signed prograde: {:.2} m/s",
            remaining_delta_v, remaining_prograde_ms
        );

        if let Some(completion) = burn_completion(
            remaining_delta_v,
            remaining_prograde_ms,
            CIRCULARIZATION_TOLERANCE_MS,
        ) {
            ksp.set_throttle(0.0)?;
            match completion {
                BurnCompletion::InTolerance => {
                    println!("Burn stopped inside the delta-v tolerance.")
                }
                BurnCompletion::Overshot => {
                    println!("Burn stopped immediately after signed delta-v crossed zero.")
                }
            }
            break;
        }

        let available_thrust_n = ksp.available_thrust_n()?;

        if staging_plan.reconcile_manual_stage_transition(&mut ksp, available_thrust_n)? {
            println!(
                "Manual interstage transition detected during burn; configured orbital propulsion stage activated and verified."
            );
            burn_aligned_since = None;
            continue;
        }

        if staging_plan.check_for_spent_stage(&mut ksp, available_thrust_n)? {
            println!("Active engines exhausted during burn; configured propulsion stage activated and verified.");
            burn_aligned_since = None;
            continue;
        }

        let burn_vector = ksp.node_remaining_burn_vector(node)?;
        ksp.set_target_direction(burn_vector)?;
        let pointing_error_deg = ksp.autopilot_error_deg()?;
        if pointing_error_deg <= settings.pointing_tolerance_deg {
            burn_aligned_since.get_or_insert_with(Instant::now);
        } else {
            burn_aligned_since = None;
        }
        let pointing_settled = burn_aligned_since.is_some_and(|since| {
            since.elapsed().as_secs_f64() >= settings.pointing_settle_time_s
        });

        if !pointing_settled {
            ksp.set_throttle(0.0)?;
            println!(
                "Burn paused for alignment; pointing error: {:.2} deg",
                pointing_error_deg
            );
            if ksp.node_time_to(node)? < -missed_node_tolerance_s {
                ksp.remove_node(node)?;
                return Err(IoError::new(
                    ErrorKind::TimedOut,
                    "Circularization burn missed its maneuver window while realigning; aborting.",
                )
                .into());
            }
            thread::sleep(Duration::from_secs_f64(BURN_CONTROL_INTERVAL_S));
            continue;
        }

        let mass_kg = ksp.mass_kg()?;
        let acceleration_limited_throttle =
            throttle_for_max_acceleration(available_thrust_n, mass_kg, max_acceleration_ms2, 1.0);
        // As the node approaches, leave a 0.25 m/s cushion inside the allowed
        // 0.5 m/s tolerance. This makes an in-flight kRPC update or an imperfect
        // acceleration estimate far less likely to carry the vessel past the node.
        let safe_delta_v_ms = (remaining_delta_v - FINE_BURN_TARGET_MS).max(0.0);
        let precision_throttle = (safe_delta_v_ms * mass_kg
            / (available_thrust_n as f64 * BURN_CONTROL_INTERVAL_S)) as f32;
        let throttle = acceleration_limited_throttle.min(precision_throttle).clamp(0.0, 1.0);
        ksp.set_throttle(throttle)?;

        if ksp.node_time_to(node)? < -missed_node_tolerance_s {
            ksp.set_throttle(0.0)?;
            ksp.remove_node(node)?;
            return Err(IoError::new(
                ErrorKind::TimedOut,
                "Circularization burn missed its maneuver window; aborting.",
            )
            .into());
        }

        thread::sleep(Duration::from_secs_f64(BURN_CONTROL_INTERVAL_S));
    }

    ksp.set_throttle(0.0)?;
    ksp.remove_node(node)?;

    let final_apoapsis_m = ksp.apoapsis_altitude_m()?;
    let final_periapsis_m = ksp.periapsis_altitude_m()?;
    let orbit_outcome = classify_orbit(
        final_apoapsis_m,
        final_periapsis_m,
        target_apoapsis_m,
        body.atmosphere_depth_m,
        orbit_safety_margin_m,
        settings.orbit_target_tolerance_km * 1000.0,
    );
    println!(
        "Final apoapsis: {:.1} km, periapsis: {:.1} km",
        final_apoapsis_m / 1000.0,
        final_periapsis_m / 1000.0,
    );

    match orbit_outcome {
        OrbitOutcome::Successful => println!("Result: successful approximately circular orbit."),
        OrbitOutcome::SafeButOffTarget => println!(
            "Result: safe orbit, but apoapsis or periapsis is outside the configured target tolerance."
        ),
        OrbitOutcome::Unsafe => {
            println!("Result: unsafe or suborbital trajectory.");
            return Err(IoError::other(
                "Circularization did not produce an orbit above the atmospheric safety margin.",
            )
            .into());
        }
    }

    Ok(())
}