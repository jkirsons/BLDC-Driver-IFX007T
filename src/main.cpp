/*
Motor Used: Gartt 5010
Specifications: Motor KV: 300KV RPM/V
Motor Resistance (RM): 0.0946 Ω
Idle Current (Io/10V): 0.3A/10V
Max Continuous Current: 36A
Max Continuous Power: 850W
Weight: ≈193g
Lipo Cell: 6S―8S
Motor Diameter: 58.8mm
Motor Body Length: 27mm
Configuration: 12N14P
Overall Shaft Length: 33.5mm
Shaft Diameter: 4.0mm
Bolt holes spacing: 25mm
Bolt thread: M3×6
*/
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPIConfig_s MA702_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15, 
  .command_rw_bit = 0,
  .command_parity_bit = 0
};

MagneticSensorSPI sensor = MagneticSensorSPI(MA702_SPI, 5);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
//BLDCMotor motor = BLDCMotor(7, 0.0946F);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 27, 4, 33, 26, 14);

// velocity set point variable
float target_velocity = 0;
float enable = 1;
// instantiate the commander
Commander command = Commander(Serial);
void doCommander(char* cmd) { command.motor(&motor, cmd); }

unsigned long pidDelay = 0;
bool pids_set = false;

struct target {
  float P;
  float I;
  float D;
};
target motor_target = { .P = 0.1, .I = 10, .D = 0};

void setup() {
 
  // initialise magnetic sensor hardware
  SPI.begin(20, 19, 21, 5);
  sensor.init(&SPI);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 25000;  
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.motion_downsample = 5;

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.0;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;
  
  // default voltage_power_supply
  motor.voltage_limit = 6;
  //motor.velocity_limit = 2;
  //motor.voltage_sensor_align = 12;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.001;

  // use monitoring with serial 
  Serial.begin(115200);

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VOLT_Q | _MON_VEL | _MON_ANGLE;

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC(5.45, Direction::CCW);

  // don't go back to 0
  motor.target = motor.shaftAngle();

  command.add('M', doCommander, "Commander Command");  
  command.verbose = VerboseMode::user_friendly;
  command.decimal_places = 4;

  // current sense
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(32, INPUT);
  analogSetAttenuation(ADC_0db);

  _delay(1000);

  pidDelay = millis() + 10;
}

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
  
  // user communication
  command.run();

  // Smooth start the PIDs
  if(!pids_set && (millis() > pidDelay)) {
    motor.PID_velocity.P = motor_target.P;
    motor.PID_velocity.I = motor_target.I;
    motor.PID_velocity.D = motor_target.D;
    pids_set = true;
  }

}