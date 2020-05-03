#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init() {
    BaseController::Init();

    // variables needed for integral control
    integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
    // Load params from simulator parameter system
    ParamsHandle config = SimpleConfig::GetInstance();

    // Load parameters (default to 0)
    kpPosXY = config->Get(_config + ".kpPosXY", 0);
    kpPosZ = config->Get(_config + ".kpPosZ", 0);
    KiPosZ = config->Get(_config + ".KiPosZ", 0);

    kpVelXY = config->Get(_config + ".kpVelXY", 0);
    kpVelZ = config->Get(_config + ".kpVelZ", 0);

    kpBank = config->Get(_config + ".kpBank", 0);
    kpYaw = config->Get(_config + ".kpYaw", 0);

    kpPQR = config->Get(_config + ".kpPQR", V3F());

    maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
    maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
    maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
    maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

    maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

    minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
    maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);

    maxTorque = config->Get(_config + ".maxTorque", 1);
#else
    // load params from PX4 parameter system
    //TODO
    param_get(param_find("MC_PITCH_P"), &Kp_bank);
    param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd) {
    // Convert a desired 3-axis moment and collective thrust command to
    //   individual motor thrust commands
    // INPUTS:
    //   collThrustCmd: desired collective thrust [N]
    //   momentCmd: desired rotation moment about each axis [N m]
    // OUTPUT:
    //   set class member variable cmd (class variable for graphing) where
    //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

    // HINTS:
    // - you can access parts of momentCmd via e.g. momentCmd.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // Perpendicular distance (d_perp, l):
    // The rotors are assembled L units away from the center of mass,
    // at a 45° angle to the body frame coordinate system.
    // Thus, the perpendicular distance l = L * cos(45°).
    // Since cos(45°) = 1/√2, we have
    const auto d_perp = L * static_cast<float>(M_SQRT1_2);

    // The goal of this method is to take a total thrust F and
    // distribute it across all rotors, such that the specified rotational
    // moment is obtained.
    const auto F = collThrustCmd; // F_1 + F_1 + F_2 + F_3

#if 0

    // About the maths ...
    //
    // In general, we have
    //   tau = k_m * omega^2
    //   F = tau / l
    //
    // From the forums we've got
    //   kappa = k_f / k_m          <- TODO: verify this
    //
    // From the lectures, we have
    //   u_p_bar = tau_x / Ixx
    //   u_q_bar = tau_y / Iyy
    //   u_r_bar = tau_z / Izz
    //
    // and
    //   c_bar = F / k_f
    //   p_bar = (Ixx u_p_bar) / (k_f * l) = tau_x / (k_f * l)
    //   q_bar = (Iyy u_q_bar) / (k_f * l) = tau_y / (k_f * l)
    //   r_bar = (Izz u_z_bar) / (k_m)     = tau_z / k_m
    //
    // From the exercises we have
    //   omega_4^2 = 1/4 (c_bar + p_bar - q_bar - r_bar)
    //   omega_3^2 = 1/2 (c_bar - q_bar) - omega_4^2
    //   omega_2^2 = 1/2 (c_bar - p_bar) - omega_3^2
    //   omega_1^2 = c - omega_2^2 - omega_3^2 - omega_4^2
    //
    // Expanding the equations gives
    //   omega_1^2 = 1/4 (c_bar + p_bar + q_bar + r_bar)
    //   omega_2^2 = 1/4 (c_bar - p_bar + q_bar - r_bar)
    //   omega_3^2 = 1/4 (c_bar - p_bar - q_bar + r_bar)
    //   omega_4^2 = 1/4 (c_bar - p_bar + q_bar - r_bar)
    //
    // Since tau = k_m * omega^2 and F = tau / l, we have
    //
    //   | F1 |           k_m    | 1  1  1  1 |   | F     /  k_f      |
    //   | F2 | = 0.25 * ----- * | 1 -1  1 -1 | * | tau_x / (k_f * l) |
    //   | F3 |            l     | 1 -1 -1  1 |   | tau_y / (k_f * l) |
    //   | F4 |                  | 1 -1  1 -1 |   | tau_z /  k_m      |
    //
    // By realizing that 1/kappa = k_m / k_f, this gives
    //
    //   | F1 |          1   | 1  1  1  1 |   | F     /  kappa      |
    //   | F2 | = 0.25 * - * | 1 -1  1 -1 | * | tau_x / (kappa * l) |
    //   | F3 |          l   | 1 -1 -1  1 |   | tau_y / (kappa * l) |
    //   | F4 |              | 1 -1  1 -1 |   | tau_z               |

    const auto F_bar = F / kappa;
    const auto tau_x_bar = momentCmd.x / (kappa * d_perp);
    const auto tau_y_bar = momentCmd.y / (kappa * d_perp);
    const auto tau_z_bar = -momentCmd.z / d_perp;

    const auto factor = 0.25 / d_perp;
    const auto F1 = factor * (F_bar + tau_x_bar + tau_y_bar + tau_z_bar);
    const auto F2 = factor * (F_bar - tau_x_bar - tau_y_bar + tau_z_bar);
    const auto F3 = factor * (F_bar - tau_x_bar + tau_y_bar - tau_z_bar);
    const auto F4 = factor * (F_bar + tau_x_bar - tau_y_bar - tau_z_bar);

    // Note that the order of rotors is different from the lectures,
    // which is why the F3 and F4 values were swapped below.
    cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
    cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
    cmd.desiredThrustsN[2] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust); // rear left
    cmd.desiredThrustsN[3] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust); // rear right

#else

    // Okay, so the above is apparently incorrect.
    // After consulting with the internet, the following seems to be the "correct"
    // solution. I do have trouble getting there though.
    // In the above approach, we have
    //
    //   c_bar = F / k_f
    //   p_bar = (Ixx u_p_bar) / (k_f * l) = tau_x / (k_f * l)
    //   q_bar = (Iyy u_q_bar) / (k_f * l) = tau_y / (k_f * l)
    //   r_bar = (Izz u_z_bar) / (k_m)     = tau_z / k_m
    //
    // Clearly, if we multiply all values with k_f, we end up with
    //
    //   c_bar' = F
    //   p_bar' = tau_x / l
    //   q_bar' = tau_y / l
    //   r_bar' = tau_z * kf / k_m
    //
    // Now r_bar' is a problem, because it already contains Kappa.
    //
    // I mean, sure. If kappa is torque per force, then we could divide
    // a torque by kappa and get a thrust.

    const auto c_bar = F;
    const auto p_bar =  momentCmd.x / d_perp;
    const auto q_bar =  momentCmd.y / d_perp;
    const auto r_bar = -momentCmd.z / kappa;

    const auto F1 = 0.25 * (c_bar + p_bar + q_bar + r_bar);
    const auto F2 = 0.25 * (c_bar - p_bar + q_bar - r_bar);
    const auto F3 = 0.25 * (c_bar + p_bar - q_bar - r_bar);
    const auto F4 = 0.25 * (c_bar - p_bar - q_bar + r_bar);

    cmd.desiredThrustsN[0] = F1; // front left
    cmd.desiredThrustsN[1] = F2; // front right
    cmd.desiredThrustsN[2] = F3; // rear left
    cmd.desiredThrustsN[3] = F4; // rear right

    // It appears we don't have to concern ourselves with the details
    // of limiting the thrust range.
    // cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
    // cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
    // cmd.desiredThrustsN[2] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust); // rear left
    // cmd.desiredThrustsN[3] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust); // rear right

#endif

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr) {
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes

    // HINTS:
    //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
    //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
    //  - you'll also need the gain parameter kpPQR (it's a V3F)

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    const auto rateError = pqrCmd - pqr;

    const V3F moi { Ixx, Iyy, Izz };
    auto momentCmd = moi * (kpPQR * rateError);

#if FUN_AND_PROFIT

    const auto torque = momentCmd.mag();
    if (torque > maxTorque) {
        momentCmd = momentCmd*maxTorque/torque;
    }

#endif

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd) {
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the roll/pitch gain kpBank
    //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

    V3F pqrCmd { 0.0, 0.0, 0.0 };
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    if (collThrustCmd <= 0) {
        return pqrCmd;
    }

    // In NED, down is the new up.
    const auto acceleration = -collThrustCmd / mass;

    // Target roll and pitch rates.
    const auto b_x_c_target = CONSTRAIN(accelCmd.x / acceleration, -maxTiltAngle, maxTiltAngle);
    const auto b_y_c_target = CONSTRAIN(accelCmd.y / acceleration, -maxTiltAngle, maxTiltAngle);

    // Actual roll and pitch values.
    const auto b_x = R(0,2);
    const auto b_y = R(1,2);

    // P controller for roll and pitch rates.
    const auto b_x_commanded_dot = kpBank * (b_x_c_target - b_x);
    const auto b_y_commanded_dot = kpBank * (b_y_c_target - b_y);

    // Convert to rates in the body frame.
    pqrCmd.x = (R(1,0) * b_x_commanded_dot - R(0,0) * b_y_commanded_dot) / R(2,2);
    pqrCmd.y = (R(1,1) * b_x_commanded_dot - R(0,1) * b_y_commanded_dot) / R(2,2);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude,
                                   float accelZCmd, float dt) {
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the gain parameters kpPosZ and kpVelZ
    //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
    //  - make sure to return a force, not an acceleration
    //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////



    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF) {
    // Calculate a desired horizontal acceleration based on
    //  desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmdFF: feed-forward acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    // HINTS:
    //  - use the gain parameters kpPosXY and kpVelXY
    //  - make sure you limit the maximum horizontal velocity and acceleration
    //    to maxSpeedXY and maxAccelXY

    // make sure we don't have any incoming z-component
    accelCmdFF.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    // we initialize the returned desired acceleration to the feed-forward value.
    // Make sure to _add_, not simply replace, the result of your controller
    // to this variable
    V3F accelCmd = accelCmdFF;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////



    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw) {
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    // HINTS:
    //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
    //  - use the yaw control gain parameter kpYaw

    float yawRateCmd = 0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime) {
    curTrajPoint = GetNextTrajectoryPoint(simTime);

    float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt,
                                          curTrajPoint.accel.z, dt);

    // reserve some thrust margin for angle control
    float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
    collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f,
                              (maxMotorThrust - thrustMargin) * 4.f);

    V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel,
                                        curTrajPoint.accel);

    V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
    desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

    V3F desMoment = BodyRateControl(desOmega, estOmega);

    return GenerateMotorCommands(collThrustCmd, desMoment);
}
