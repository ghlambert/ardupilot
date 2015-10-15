/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_guided_stabilize.pde - init and run calls for guided stabilize flight
 * mode
 */

// Target roll angle in radians
static float guided_stabilize_target_roll;

// Target pitch angle in radians
static float guided_stabilize_target_pitch;

// Target yaw rate in radians
static float guided_stabilize_target_yaw_rate;

// guided_stabilize_init - initialise guided stabilize controller
static bool guided_stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // Reset remote setpoint to begin
    guided_stabilize_target_roll = 0;
    guided_stabilize_target_pitch = 0;
    guided_stabilize_target_yaw_rate = 0;

    // stabilize should never be made to fail
    return true;
}

// guided_stabilize_run - runs the main guided stabilize controller
// should be called at 100hz or more
static void guided_stabilize_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    //get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    target_roll = guided_stabilize_target_roll;
    target_pitch = guided_stabilize_target_pitch;

    // get pilot's desired yaw rate
    //target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    target_yaw_rate = guided_stabilize_target_yaw_rate;

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}

static void guided_stabilize_set_target(float roll, float pitch, float yaw_rate)
{
    // Convert from radians to centi-degrees
    guided_stabilize_target_roll = (18000/M_PI_F)*roll;
    guided_stabilize_target_pitch = (18000/M_PI_F)*pitch;
    guided_stabilize_target_yaw_rate = (18000/M_PI_F)*yaw_rate;
}

