/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_guided_stabilize.pde - init and run calls for guided stabilize flight mode
 */

#define GUIDED_STABILIZE_TARGET_MASK_ATTITUDE_IGNORE (1<<0)
#define GUIDED_STABILIZE_TARGET_MASK_YAW_RATE_IGNORE (1<<1)

// Variable used to switch between user/remote commands
static uint16_t guided_stabilize_target_mask;

// Target roll angle in radians
static float guided_stabilize_target_roll;

// Target pitch angle in radians
static float guided_stabilize_target_pitch;

// Target yaw rate in radians/sec
static float guided_stabilize_target_yaw_rate;

// guided_stabilize_init - initialise stabilize controller
bool Copter::guided_stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // Reset remote setpoint to begin
    guided_stabilize_target_mask = GUIDED_STABILIZE_TARGET_MASK_ATTITUDE_IGNORE
        | GUIDED_STABILIZE_TARGET_MASK_YAW_RATE_IGNORE;
    guided_stabilize_target_roll = 0;
    guided_stabilize_target_pitch = 0;
    guided_stabilize_target_yaw_rate = 0;

    // stabilize should never be made to fail
    return true;
}

// guided_stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::guided_stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || ap.throttle_zero) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // slow start if landed
        if (ap.land_complete) {
            motors.slow_start(true);
        }
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    if(guided_stabilize_target_mask & GUIDED_STABILIZE_TARGET_MASK_ATTITUDE_IGNORE)
    {
        // convert pilot input to lean angles
        // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
    }
    else
    {
        // Use remote setpoint
        target_roll = guided_stabilize_target_roll;
        target_pitch = guided_stabilize_target_pitch;
    }

    if(guided_stabilize_target_mask & GUIDED_STABILIZE_TARGET_MASK_YAW_RATE_IGNORE)
    {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }
    else
    {
        // Use remote setpoint
        target_yaw_rate = guided_stabilize_target_yaw_rate;
    }

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

void Copter::guided_stabilize_set_target_attitude(float roll, float pitch)
{
    // Convert from radians to centi-degrees
    guided_stabilize_target_roll = (18000/M_PI_F)*roll;
    guided_stabilize_target_pitch = (18000/M_PI_F)*pitch;
    guided_stabilize_target_mask &= ~GUIDED_STABILIZE_TARGET_MASK_ATTITUDE_IGNORE;
}

void Copter::guided_stabilize_unset_target_attitude()
{
    guided_stabilize_target_mask |= GUIDED_STABILIZE_TARGET_MASK_ATTITUDE_IGNORE;
}

void Copter::guided_stabilize_set_target_yaw_rate(float yaw_rate)
{
    // Convert from radians/sec to centi-degrees/sec
    guided_stabilize_target_yaw_rate = (18000/M_PI_F)*yaw_rate;
    guided_stabilize_target_mask &= ~GUIDED_STABILIZE_TARGET_MASK_YAW_RATE_IGNORE;
}

void Copter::guided_stabilize_unset_target_yaw_rate()
{
    guided_stabilize_target_mask |= GUIDED_STABILIZE_TARGET_MASK_YAW_RATE_IGNORE;
}
