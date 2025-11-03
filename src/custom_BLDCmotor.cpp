#include "custom_BLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,_HIGH_IMPEDANCE,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,_HIGH_IMPEDANCE,-1} 
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,1,1},
    {-1,_HIGH_IMPEDANCE,1},
    {-1,-1,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,-1,-1},
    {1,_HIGH_IMPEDANCE,-1},
    {1,1,-1} 
};

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance
new_BLDCMotor::new_BLDCMotor(int pp, float _R, float _KV, float _KT,  float _inductance)
: FOCMotor()
{
  // save pole pairs number
  pole_pairs = pp;
  // save phase resistance number
  phase_resistance = _R;
  // save back emf constant KV = 1/KV
  // 1/sqrt(2) - rms value
  KV_rating = NOT_SET;
  if (_isset(_KV))
    KV_rating = _KV;
  // save phase inductance
  phase_inductance = _inductance;

  KT_rating = NOT_SET;
    if (_isset(_KT))
        KT_rating = _KT;

  // torque control type is voltage by default
  torque_controller = TorqueControlType::voltage;
}


/**
	Link the driver which controls the motor
*/
void new_BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// init hardware pins
int new_BLDCMotor::init() {
  if (!driver || !driver->initialized) {
    motor_status = FOCMotorStatus::motor_init_failed;
    SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
    return 0;
  }
  motor_status = FOCMotorStatus::motor_initializing;
  SIMPLEFOC_DEBUG("MOT: Init");

  // sanity check for the voltage limit configuration
  if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
  // constrain voltage for sensor alignment
  if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

  // update the controller limits
  if(current_sense){
    // current control loop controls voltage
    PID_current_q.limit = voltage_limit;
    PID_current_d.limit = voltage_limit;
  }
  if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
    // velocity control loop controls current
    PID_velocity.limit = current_limit;
  }else{
    // velocity control loop controls the voltage
    PID_velocity.limit = voltage_limit;
  }
  P_angle.limit = velocity_limit;

  // if using open loop control, set a CW as the default direction if not already set
  // only if no sensor is used
  if(!sensor){
    if ((controller==MotionControlType::angle_openloop
      ||controller==MotionControlType::velocity_openloop)
      && (sensor_direction == Direction::UNKNOWN)) {
        sensor_direction = Direction::CW;
    }
  }

  _delay(500);
  // enable motor
  SIMPLEFOC_DEBUG("MOT: Enable driver.");
  enable();
  _delay(500);
  motor_status = FOCMotorStatus::motor_uncalibrated;
  return 1;
}


// disable motor driver
void new_BLDCMotor::disable()
{
  // disable the current sense
  if(current_sense) current_sense->disable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void new_BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // enable the current sense
  if(current_sense) current_sense->enable();
  // reset the pids
  PID_velocity.reset();
  P_angle.reset();
  PID_current_q.reset();
  PID_current_d.reset();
  // motor status update
  enabled = 1;
}

/**
  FOC functions
*/
// FOC initialization function
int  new_BLDCMotor::initFOC() {
  int exit_flag = 1;

  motor_status = FOCMotorStatus::motor_calibrating;

  // align motor if necessary
  // alignment necessary for encoders!
  // sensor and motor alignment - can be skipped
  // by setting motor.sensor_direction and motor.zero_electric_angle
  if(sensor){
    exit_flag *= alignSensor();
    // added the shaft_angle update
    sensor->update();
    shaft_angle = shaftAngle();

    // aligning the current sensor - can be skipped
    // checks if driver phases are the same as current sense phases
    // and checks the direction of measuremnt.
    if(exit_flag){
      if(current_sense){ 
        if (!current_sense->initialized) {
          motor_status = FOCMotorStatus::motor_calib_failed;
          SIMPLEFOC_DEBUG("MOT: Init FOC error, current sense not initialized");
          exit_flag = 0;
        }else{
          exit_flag *= alignCurrentSense();
        }
      }
      else { SIMPLEFOC_DEBUG("MOT: No current sense."); }
    }

  } else {
    SIMPLEFOC_DEBUG("MOT: No sensor.");
    if ((controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop)){
      exit_flag = 1;    
      SIMPLEFOC_DEBUG("MOT: Openloop only!");
    }else{
      exit_flag = 0; // no FOC without sensor
    }
  }

  if(exit_flag){
    SIMPLEFOC_DEBUG("MOT: Ready.");
    motor_status = FOCMotorStatus::motor_ready;
  }else{
    SIMPLEFOC_DEBUG("MOT: Init FOC failed.");
    motor_status = FOCMotorStatus::motor_calib_failed;
    disable();
  }

  return exit_flag;
}

// Calibrate the motor and current sense phases
int new_BLDCMotor::alignCurrentSense() {
  int exit_flag = 1; // success

  SIMPLEFOC_DEBUG("MOT: Align current sense.");

  // align current sense and the driver
  exit_flag = current_sense->driverAlign(voltage_sensor_align, modulation_centered);
  if(!exit_flag){
    // error in current sense - phase either not measured or bad connection
    SIMPLEFOC_DEBUG("MOT: Align error!");
    exit_flag = 0;
  }else{
    // output the alignment status flag
    SIMPLEFOC_DEBUG("MOT: Success: ", exit_flag);
  }

  return exit_flag > 0;
}

// Encoder alignment to electrical 0 angle
int new_BLDCMotor::alignSensor() {
  int exit_flag = 1; //success
  SIMPLEFOC_DEBUG("MOT: Align sensor.");

  // check if sensor needs zero search
  if(sensor->needsSearch()) exit_flag = absoluteZeroSearch();
  // stop init if not found index
  if(!exit_flag) return exit_flag;

  // v2.3.3 fix for R_AVR_7_PCREL against symbol" bug for AVR boards
  // TODO figure out why this works
  float voltage_align = voltage_sensor_align;

  // if unknown natural direction
  if(sensor_direction==Direction::UNKNOWN){

    // find natural direction
    // move one electrical revolution forward
    for (int i = 0; i <=500; i++ ) {
      float angle = _3PI_2 + _2PI * i / 500.0f;
      setPhaseVoltage(voltage_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    // take and angle in the middle
    sensor->update();
    float mid_angle = sensor->getAngle();
    // move one electrical revolution backwards
    for (int i = 500; i >=0; i-- ) {
      float angle = _3PI_2 + _2PI * i / 500.0f ;
      setPhaseVoltage(voltage_align, 0,  angle);
	    sensor->update();
      _delay(2);
    }
    sensor->update();
    float end_angle = sensor->getAngle();
    // setPhaseVoltage(0, 0, 0);
    _delay(200);
    // determine the direction the sensor moved
    float moved =  fabs(mid_angle - end_angle);
    if (moved<MIN_ANGLE_DETECT_MOVEMENT) { // minimum angle to detect movement
      SIMPLEFOC_DEBUG("MOT: Failed to notice movement");
      return 0; // failed calibration
    } else if (mid_angle < end_angle) {
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CCW");
      sensor_direction = Direction::CCW;
    } else{
      SIMPLEFOC_DEBUG("MOT: sensor_direction==CW");
      sensor_direction = Direction::CW;
    }
    // check pole pair number
    pp_check_result = !(fabs(moved*pole_pairs - _2PI) > 0.5f); // 0.5f is arbitrary number it can be lower or higher!
    if( pp_check_result==false ) {
      SIMPLEFOC_DEBUG("MOT: PP check: fail - estimated pp: ", _2PI/moved);
    } else {
      SIMPLEFOC_DEBUG("MOT: PP check: OK!");
    }

  } else { SIMPLEFOC_DEBUG("MOT: Skip dir calib."); }

  // zero electric angle not known
  if(!_isset(zero_electric_angle)){
    // align the electrical phases of the motor and sensor
    // set angle -90(270 = 3PI/2) degrees
    setPhaseVoltage(voltage_align, 0,  _3PI_2);
    _delay(700);
    // read the sensor
    sensor->update();
    // get the current zero electric angle
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    //zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
    _delay(20);
    SIMPLEFOC_DEBUG("MOT: Zero elec. angle: ", zero_electric_angle);
    // stop everything
    setPhaseVoltage(0, 0, 0);
    _delay(200);
  } else { SIMPLEFOC_DEBUG("MOT: Skip offset calib."); }
  return exit_flag;
}

// Encoder alignment the absolute zero angle
// - to the index
int new_BLDCMotor::absoluteZeroSearch() {
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_DEBUG("MOT: Index search...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if(monitor_port){
    if(sensor->needsSearch()) { SIMPLEFOC_DEBUG("MOT: Error: Not found!"); }
    else { SIMPLEFOC_DEBUG("MOT: Success!"); }
  }
  return !sensor->needsSearch();
}

// Iterative function looping FOC algorithm, setting Uq on the Motor
// The faster it can be run the better
void new_BLDCMotor::loopFOC() {
  // update sensor - do this even in open-loop mode, as user may be switching between modes and we could lose track
  //                 of full rotations otherwise.
  if (sensor) sensor->update();

  // if open-loop do nothing
  if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
  
  // if disabled do nothing
  if(!enabled) return;

  // Needs the update() to be called first
  // This function will not have numerical issues because it uses Sensor::getMechanicalAngle() 
  // which is in range 0-2PI
  electrical_angle = electricalAngle();
  switch (torque_controller) {
    case TorqueControlType::voltage:
      // no need to do anything really
      break;
    case TorqueControlType::dc_current:
      if(!current_sense) return;
      // read overall current magnitude
      current.q = current_sense->getDCCurrent(electrical_angle);
      // filter the value values
      current.q = LPF_current_q(current.q);
      // calculate the phase voltage
      voltage.q = PID_current_q(current_sp - current.q);
      // d voltage  - lag compensation
      if(_isset(phase_inductance)) voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      else voltage.d = 0;
      break;
    case TorqueControlType::foc_current:
      if(!current_sense) return;
      // read dq currents
      current = current_sense->getFOCCurrents(electrical_angle);
      // filter values
      current.q = LPF_current_q(current.q);
      current.d = LPF_current_d(current.d);
      // calculate the phase voltages
      voltage.q = PID_current_q(current_sp - current.q);
      voltage.d = PID_current_d(-current.d);
      // d voltage - lag compensation - TODO verify
      // if(_isset(phase_inductance)) voltage.d = _constrain( voltage.d - current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      break;
    default:
      // no torque control selected
      SIMPLEFOC_DEBUG("MOT: no torque control selected!");
      break;
  }

  // set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}


void new_BLDCMotor::move(float new_target) {

  // set internal target variable
  if(_isset(new_target)) target = new_target;
  
  // downsampling (optional)
  if(motion_cnt++ < motion_downsample) return;
  motion_cnt = 0;

  // shaft angle/velocity need the update() to be called first
  // get shaft angle
  // TODO sensor precision: the shaft_angle actually stores the complete position, including full rotations, as a float
  //                        For this reason it is NOT precise when the angles become large.
  //                        Additionally, the way LPF works on angle is a precision issue, and the angle-LPF is a problem
  //                        when switching to a 2-component representation.
  if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop ) 
    shaft_angle = shaftAngle(); // read value even if motor is disabled to keep the monitoring updated but not in openloop mode
  // get angular velocity  TODO the velocity reading probably also shouldn't happen in open loop modes?
  shaft_velocity = shaftVelocity(); // read value even if motor is disabled to keep the monitoring updated

  // if disabled do nothing
  if(!enabled) return;
  
  // calculate the back-emf voltage if KV_rating available U_bemf = vel*(1/KV)
  if (_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
  // estimate the motor current if phase resistance available and current_sense not available
  if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;

  // upgrade the current based voltage limit
  switch (controller) {
    case MotionControlType::torque:
      if(torque_controller == TorqueControlType::voltage){ // if voltage torque control
        if(!_isset(phase_resistance))  voltage.q = target;
        else  voltage.q =  target*phase_resistance + voltage_bemf;
        voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }else{
        current_sp = target; // if current/foc_current torque control
      }
      break;
    case MotionControlType::angle:
      // TODO sensor precision: this calculation is not numerically precise. The target value cannot express precise positions when
      //                        the angles are large. This results in not being able to command small changes at high position values.
      //                        to solve this, the delta-angle has to be calculated in a numerically precise way.
      // angle set point
      shaft_angle_sp = target;
      // calculate velocity set point
      shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
      shaft_velocity_sp = _constrain(shaft_velocity_sp,-velocity_limit, velocity_limit);
      // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
      // if torque controlled through voltage
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q =  _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity:
      // velocity set point - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control
      if(torque_controller == TorqueControlType::voltage){
        // use voltage if phase-resistance not provided
        if(!_isset(phase_resistance))  voltage.q = current_sp;
        else  voltage.q = _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
        // set d-component (lag compensation if known inductance)
        if(!_isset(phase_inductance)) voltage.d = 0;
        else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
      }
      break;
    case MotionControlType::velocity_openloop:
      // velocity control in open loop - sensor precision: this calculation is numerically precise.
      shaft_velocity_sp = target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
    case MotionControlType::angle_openloop:
      // angle control in open loop - 
      // TODO sensor precision: this calculation NOT numerically precise, and subject
      //                        to the same problems in small set-point changes at high angles 
      //                        as the closed loop version.
      shaft_angle_sp = target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
      break;
  }
}

void new_BLDCMotor::IMP_move(float new_target) {
    // Update target if provided
    if(new_target != NOT_SET) target = new_target;
    
    // Update sensor and FOC loop
    if(sensor) sensor->update();
    
    // Calculate time step
    unsigned long now_us = _micros();
    float dt = (now_us - open_loop_timestamp) * 1e-6f;
    if(dt <= 0 || dt > 0.5f) dt = 1e-3f;
    open_loop_timestamp = now_us;

    // Update current state
    shaft_angle = shaftAngle();
    shaft_velocity = shaftVelocity();
    
    // Impedance control parameters (outer loop)
    const float K_base = 0.05f;  // base stiffness
    const float B_base = 0.7f;   // base damping
    
    // Calculate position and velocity errors for impedance control
    float x_d = target;                // desired position
    float v_d = 0;                     // desired velocity (typically zero for position control)
    float position_error = x_d - shaft_angle;
    float velocity_error = v_d - shaft_velocity;
    
    // Apply low-pass filtering to errors (optional)
    position_error = pos_error_lpf(position_error);
    velocity_error = vel_error_lpf(velocity_error);
    
    // Calculate impedance torque - this is our "virtual" torque from impedance model
    float tau_imp = -K_base * position_error - B_base * velocity_error;
    tau_imp = _constrain(tau_imp/KT_rating, -current_limit, current_limit);
    
    // ---------- FROM HERE WE USE ORIGINAL BLDC MOTOR STRUCTURE ----------
    
    // Convert impedance torque to current setpoint
    current_sp = tau_imp;
    
    // Calculate back-EMF compensation
    if(_isset(KV_rating)) voltage_bemf = shaft_velocity/(KV_rating*_SQRT3)/_RPM_TO_RADS;
    
    // Convert current/torque setpoint to voltage (following BLDCMotor structure)
    if(torque_controller == TorqueControlType::voltage) {
        if(!_isset(phase_resistance)) {
            voltage.q = current_sp;
        } else {
            voltage.q = _constrain(current_sp * phase_resistance + voltage_bemf, 
                                   -voltage_limit, voltage_limit);
        }
        
        // Set d-axis voltage (field weakening or lag compensation)
        if(!_isset(phase_inductance)) {
            voltage.d = 0;
        } else {
            voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, 
                                  -voltage_limit, voltage_limit);
        }
        
        // Apply voltages through FOC
        setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
    } else {
        // For other torque control modes, just use the current setpoint
        // (this branch would be handled in loopFOC)
    }

    // Debug output (optional)
    if(monitor_port && (millis() % 100 == 0)) {
        Serial.print("Imp: "); Serial.print(tau_imp);
        Serial.print(" Vel: "); Serial.print(shaft_velocity);
        Serial.print(" Error: "); Serial.println(position_error);
    }
}

// void new_BLDCMotor::IMP_move(float new_target) {

//   float Kt = KT_rating;
//   // Only update target if explicitly set
//   if(new_target != NOT_SET) target = new_target;
  
//   // Update sensor and FOC loop
//   if(sensor) sensor->update();
    

//   // Calculate time step
//   unsigned long now_us = _micros();
//   float dt = (now_us - open_loop_timestamp) * 1e-6f;
//   if(dt <= 0 || dt > 0.5f) dt = 1e-3f; // sanity check
//   open_loop_timestamp = now_us;

//   // Update current state
//   shaft_angle = shaftAngle();
//   shaft_velocity = shaftVelocity();

//   // Read current position and velocity
//   float x = shaft_angle;  // actual position
//   float v = shaft_velocity;  // actual velocity
//   float x_d = target;     // desired position
//   float v_d = 0;          // desired velocity (can be added as parameter if needed)
//   // Impedance parameters
//   const float K_base = 0.000f;  // base stiffness
//   const float B_base = 0.7f;   // base damping
  
//   // Calculate impedance torque
//   float position_error = x_d - x;
//   position_error = pos_error_lpf(position_error); // Optional: Low-pass filter the position error
//   float velocity_error = v_d - v;
//   velocity_error = vel_error_lpf(velocity_error); // Optional: Low-pass filter the velocity error
//   float tau_imp = K_base * position_error + B_base * velocity_error; // Impedance torque
//   tau_imp = _constrain(tau_imp, -0.05f, 0.05f); // Constrain torque to reasonable limits
//   // Optional: Add gravity compensation if needed
//   // float tau_gravity = calculateGravityTorque();
  
//   // static float prev_tau = 0;
//   //   const float max_tau_rate = 2.0f; // Maximum torque change per second
//   //   float tau_rate = (tau_imp - prev_tau) / dt;
//   //   if(fabs(tau_rate) > max_tau_rate) {
//   //       tau_imp = prev_tau + _sign(tau_rate) * max_tau_rate * dt;
//   //   }
//   //   prev_tau = tau_imp;



//   // Weak position stabilizer - only active near target
//   const float POSITION_THRESHOLD = 0.3f;  // rad (~5.7 degrees)
//   const float VELOCITY_THRESHOLD = 0.5f;  // rad/s
  
//   // Calculate stabilizer gain that decreases with distance from target
//   float distance_factor = 0.0f;
//   if(fabs(position_error) < POSITION_THRESHOLD && fabs(velocity_error) < VELOCITY_THRESHOLD) {
//       // Smoothly blend stabilizer based on distance
//       distance_factor = (1.0f - (fabs(position_error)/POSITION_THRESHOLD)) * 
//                        (1.0f - (fabs(velocity_error)/VELOCITY_THRESHOLD));
//       distance_factor = distance_factor * distance_factor; // Square for smoother transition
//   }
//   // Very weak stabilizer gains
//   const float Kp_pos = 0.05f;  // Reduced from 0.5
//   const float Ki_pos = 0.001f; // Reduced from 0.005
  
//   // Update integral term with anti-windup
//   static float e_theta_int = 0;
//   // Only integrate when stabilizer is active
//   if(distance_factor > 0) {
//       e_theta_int += position_error * dt;
//       e_theta_int = _constrain(e_theta_int, -1.0f, 1.0f); // Reduced integral limits
//   } else {
//       e_theta_int = 0; // Reset when far from target
//   }
  
//   // Calculate stabilizer torque with distance-based fade-in
//   float tau_pos = distance_factor * (Kp_pos * position_error + Ki_pos * e_theta_int);

//   // if(fabs(position_error) < 0.001f && fabs(velocity_error) < 0.01f) {
//   //   theta_hold = x;
//   // }
  
//   // float e_theta = theta_hold - x;
//   // e_theta_int += e_theta * dt;
  
//   // // Anti-windup
//   // e_theta_int = _constrain(e_theta_int, -10.0f, 10.0f);
  
//   // float tau_pos = Kp_pos * e_theta + Ki_pos * e_theta_int;
  
//   // Combine impedance and stabilizer torques
//   float tau_cmd = tau_imp + tau_pos;
  
//   // Convert torque to voltage (similar to original FOC)
//   // Assuming voltage mode torque control
//   if(torque_controller == TorqueControlType::voltage) {
//     // Convert torque to voltage using motor parameters
//     if(!_isset(phase_resistance))
//       voltage.q = tau_cmd;  // direct voltage control
//     else
//       voltage.q = _constrain((tau_cmd/Kt) * phase_resistance + voltage_bemf, 
//                              -voltage_limit, voltage_limit);
                             
//     // Set the q-axis voltage using FOC
//     voltage.d = 0;
//     setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
//   }

//   // Optional debugging
//     if(monitor_port && (millis() % 100 == 0)) {
//         Serial.print("Imp: "); Serial.print(tau_imp);
//         Serial.print(" Pos: "); Serial.print(tau_pos);
//         Serial.print(" Error: "); Serial.println(position_error);
//     }
  
// }

void new_BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
      }else{
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
      }else{
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation
      _sincos(angle_el, &_sa, &_ca);

      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // Clarke transform
      Ua = Ualpha;
      Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
      Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

      center = driver->voltage_limit/2;
      if (foc_modulation == FOCModulationType::SpaceVectorPWM){
        // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        // Midpoint Clamp
        float Umin = min(Ua, min(Ub, Uc));
        float Umax = max(Ua, max(Ub, Uc));
        center -= (Umax+Umin) / 2;
      } 

      if (!modulation_centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }else{
        Ua += center;
        Ub += center;
        Uc += center;
      }

      break;

  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float new_BLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float new_BLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

