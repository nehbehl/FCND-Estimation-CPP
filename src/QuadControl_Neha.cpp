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

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
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
#else
  // load params from PX4 parameter system
  //TODO

  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
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

  float f_l, f_r, r_l, r_r;
  float l = L / sqrtf(2.f);
  f_l = collThrustCmd / 4.f;
  f_r = momentCmd.x / (l*4.f) ;
  r_l = momentCmd.y / (l*4.f) ;
  r_r = momentCmd.z / (kappa*4.f);
  cmd.desiredThrustsN[0] = f_l + f_r + r_l - r_r; // front left
  cmd.desiredThrustsN[1] = f_l - f_r + r_l + r_r; // front right
  cmd.desiredThrustsN[2] = f_l + f_r - r_l + r_r; // rear left
  cmd.desiredThrustsN[3] = f_l - f_r - r_l - r_r; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
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

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F Insta;
  Insta.x = Ixx;
  Insta.y = Iyy;
  Insta.z = Izz;
  momentCmd = Insta * kpPQR * ( pqrCmd - pqr );

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
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

  V3F pqr_cmd;
  Mat3x3F Rm = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    if ( collThrustCmd > 0 ) {
    float c =  - collThrustCmd / mass;
    float b_x, b_y;
    float b_x_d, b_y_d;
    b_x = CONSTRAIN((accelCmd.x / c), -sin(maxTiltAngle), sin(maxTiltAngle));
    b_y = CONSTRAIN((accelCmd.y / c), -sin(maxTiltAngle), sin(maxTiltAngle));
    b_x_d = kpBank * (b_x - Rm(0,2));
    b_y_d = kpBank * (b_y - Rm(1,2));

    pqr_cmd.x = (Rm(1,0) * b_x_d - Rm(0,0) * b_y_d) / Rm(2,2);
    pqr_cmd.y = (Rm(1,1) * b_x_d - Rm(0,1) * b_y_d) / Rm(2,2);
  } else {
    pqr_cmd.x = 0.0;
    pqr_cmd.y = 0.0;
  }
  
  pqr_cmd.z = 0;

    

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqr_cmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
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

  Mat3x3F Rm = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float z_e = posZCmd - posZ;
  float p_t = kpPosZ * z_e;
  
  float z_d_e = velZCmd - velZ;
  integratedAltitudeError += z_e * dt;

  
  float d_t = kpVelZ * z_d_e + velZ;
  float i_t = KiPosZ * integratedAltitudeError;
  float b_z = Rm(2,2);

  float u_1_b = p_t + d_t + i_t + accelZCmd;

  float a = ( u_1_b - CONST_GRAVITY ) / b_z;

  thrust = - mass * CONSTRAIN(a, - maxAscentRate / dt, maxAscentRate / dt);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
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
  
  V3F kp_pos;
  kp_pos.x = kpPosXY;
  kp_pos.y = kpPosXY;
  kp_pos.z = 0.f;

  V3F kp_vel;
  kp_vel.x = kpVelXY;
  kp_vel.y = kpVelXY;
  kp_vel.z = 0.f;

  V3F cap_vel_cmd;
  if ( velCmd.mag() > maxSpeedXY ) {
    cap_vel_cmd = velCmd.norm() * maxSpeedXY;
  } else {
    cap_vel_cmd = velCmd;
  }
  
    accelCmd = kpPosZ * ( posCmd - pos ) + kpVelZ * ( cap_vel_cmd - vel ) + accelCmd;
  
  if ( accelCmd.mag() > maxAccelXY ) {
    accelCmd = accelCmd.norm() * maxAccelXY;
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


  float yaw_c2pi = 0;
  if ( yawCmd > 0 ) {
    yaw_c2pi = fmodf(yawCmd, 2 * F_PI);
  } else {
    yaw_c2pi = -fmodf(-yawCmd, 2 * F_PI);
  }
  float err = yaw_c2pi - yaw;
  if ( err > F_PI ) {
    err -= 2 * F_PI;
  } if ( err < -F_PI ) {
    err += 2 * F_PI;
  }
  yawRateCmd = kpYaw * err;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
