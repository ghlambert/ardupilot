/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_guided_althold.pde - init and run calls for guided althold flight
 * mode
 */

#define GUIDED_ALTHOLD_TARGET_MASK_ATTITUDE_IGNORE (1<<0)
#define GUIDED_ALTHOLD_TARGET_MASK_YAW_RATE_IGNORE (1<<1)

// Variable used to switch between user/remote commands
static uint16_t guided_althold_target_mask;

// Target roll angle in centi-degrees
static float guided_althold_target_roll;

// Target pitch angle in centi-degrees
static float guided_althold_target_pitch;

// Target yaw rate in centi-degrees/sec
static float guided_althold_target_yaw_rate;

// guided_althold_init - initialise guided althold controller
static bool guided_althold_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialize altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // Reset remote setpoint to begin
    guided_althold_target_mask = GUIDED_ALTHOLD_TARGET_MASK_ATTITUDE_IGNORE 
        | GUIDED_ALTHOLD_TARGET_MASK_YAW_RATE_IGNORE;
    guided_althold_target_roll = 0.0;
    guided_althold_target_pitch = 0.0;
    guided_althold_target_yaw_rate = 0.0;

    return true;
}

// guided_althold_run - runs the main guided althold controller
// should be called at 100hz or more
static void guided_althold_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed)
    {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    if(!(guided_althold_target_mask 
        & GUIDED_ALTHOLD_TARGET_MASK_ATTITUDE_IGNORE))
    {
        // Use remote setpoint instead of pilot input
        target_roll = guided_althold_target_roll;
        // Roll is always ignored for now
        //target_pitch = guided_althold_target_pitch;
    }

    if(guided_althold_target_mask & GUIDED_ALTHOLD_TARGET_MASK_YAW_RATE_IGNORE)
    {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }
    else
    {
        // Use remote setpoint
        target_yaw_rate = guided_althold_target_yaw_rate;
    }

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off
    if (ap.land_complete && target_climb_rate > 0)
    {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete)
    {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
    }
    else
    {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(target_roll, target_pitch, target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop

        // call throttle controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)
        {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// Set target attitude in radians
static void guided_althold_set_target_attitude(float roll, float pitch)
{
    // Convert from radians to centi-degrees
    guided_althold_target_roll = (18000/M_PI_F)*roll;
    guided_althold_target_pitch = (18000/M_PI_F)*pitch;
    guided_althold_target_mask &= ~GUIDED_ALTHOLD_TARGET_MASK_ATTITUDE_IGNORE;
}

static void guided_althold_unset_target_attitude()
{
    guided_althold_target_mask |= GUIDED_ALTHOLD_TARGET_MASK_ATTITUDE_IGNORE;
}

// Set target yaw rate in radians/sec
static void guided_althold_set_target_yaw_rate(float yaw_rate)
{
    // Convert from radians to centi-degrees
    guided_althold_target_yaw_rate = (18000/M_PI_F)*yaw_rate;
    guided_althold_target_mask &= ~GUIDED_ALTHOLD_TARGET_MASK_YAW_RATE_IGNORE;
}

static void guided_althold_unset_target_yaw_rate()
{
    guided_althold_target_mask |= GUIDED_ALTHOLD_TARGET_MASK_YAW_RATE_IGNORE;
}

