// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//AUTO IS DEF FKD UP, DONT USE THIS WITHOUT CHECKING HOW TO AUTO
package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//import javax.lang.model.util.ElementScanner6;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.*;

//import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.PrintCommand;



public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
//defines motorcontrollers  
  XboxController gamepad = new XboxController(0);


  Encoder driveEncoder = new Encoder(0,1);

  WPI_TalonSRX L2_motor = new WPI_TalonSRX(5); // defines the motors 
  WPI_TalonSRX L1_motor = new WPI_TalonSRX(7);
  WPI_TalonSRX L3_Motor = new WPI_TalonSRX(6);

  WPI_TalonSRX R1_motor = new WPI_TalonSRX(3);
  WPI_TalonSRX R2_motor = new WPI_TalonSRX(4);
  WPI_TalonSRX R3_motor = new WPI_TalonSRX(2);

  WPI_TalonSRX LArm_motor = new WPI_TalonSRX(8);
  WPI_TalonSRX RArm_motor = new WPI_TalonSRX(9);

  WPI_VictorSPX LIntake_motor = new WPI_VictorSPX(10);
  WPI_VictorSPX RIntake_motor = new WPI_VictorSPX(11);

  
  MotorControllerGroup LeftDrive = new MotorControllerGroup(L2_motor, L3_Motor, L1_motor); //defines motor groups
  MotorControllerGroup RightDrive = new MotorControllerGroup(R1_motor, R2_motor, R3_motor); 

  MotorControllerGroup Lift = new MotorControllerGroup(LArm_motor, RArm_motor);

  MotorControllerGroup Intake = new MotorControllerGroup(LIntake_motor, RIntake_motor);



//drive
  DifferentialDrive drive = new DifferentialDrive(LeftDrive, RightDrive);

  //compressor
  Compressor pcmCompressor = new Compressor(0,PneumaticsModuleType.CTREPCM);

//solenoid one and two rename to left right
  DoubleSolenoid LeftSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid RightSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

// motion profiling and PID
/*PIDController pid = new PIDController(Constants.kP,Constants.kI,Constants.kD);

ProfiledPIDController LiftProfileController = new ProfiledPIDController(
  Constants.Arm_kP, Constants.Arm_kI, Constants.Arm_kD,
  new TrapezoidProfile.Constraints(5, 10)); // speed and acceleration

*/


  //end definition

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override


  
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings
    CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();

    RIntake_motor.setInverted(true);

    LArm_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

LArm_motor.setNeutralMode(NeutralMode.Brake);
RArm_motor.setNeutralMode(NeutralMode.Brake);
  }
  

public double LiftSpeed() {

  double LArmEncoderValue = LArm_motor.getSelectedSensorPosition(); // do we just want one encoder set up? arms should be locked togeter so it shouldnt matter.
  double KEncoderValueAtTop = 4000; //test me, 4000 is used a lot in the code
  double KAcceleration = .0003; //test me
  double KStopSpeed = 9; // test me //this value holds the lift motors in place

  // do we just want one encoder set up? arms should be locked togeter so it shouldnt matter. // instead of this do slave master for both motors
  //maybe set up min and max speeds using endspeed = outputspeed and changing lift speed to outputspeed
 LArmEncoderValue = LArm_motor.getSelectedSensorPosition(); 
 double endspeed = Math.sqrt(-KAcceleration*(LArmEncoderValue- KEncoderValueAtTop)); 
 double outputSpeed= 0;  
  
  outputSpeed = endspeed;

  if (outputSpeed > 1) {
    return 1;
  }
  else if (outputSpeed < -1) {
    return -1;
  } else{
    return outputSpeed;
  }


 
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  // gets angle of Dpad, 0 = up , 90 = right etc. -1 = no press(?)
  int dpad = gamepad.getPOV(0); 

//these might need to be getRawAxis
// raw axis arguments are 1 5 2 3 respectively
//remember to check the Lstick and Rstick
    double Lstick = gamepad.getRawAxis(1)*-1; //left stick Y axis // read from 0 to 1
    double Rstick = gamepad.getRawAxis(5);// right stick y axis
    double LT = gamepad.getRawAxis(2); // Left trigger, analog 
    double RT = gamepad.getRawAxis(3); // right trigger, analog
  //drive.tankDrive(Lstick, Rstick);


//checks dpad for toggles
if(dpad == 180)
 {
  if (Constants.DriveSpeedToggle == false) // bool is false, switch bool values
  {
      Constants.DriveSpeedToggle = true;
      Timer.delay(.1);
  } else if(Constants.DriveSpeedToggle == true) // bool is true, switch bool values
   {
    Constants.DriveSpeedToggle = false;
    Timer.delay(.1);
  }
} 

if(dpad == 90){
  if(Constants.DriveDirectionToggle == false)
  {
    Constants.DriveDirectionToggle = true;
    Timer.delay(.1);
  
}  else if(Constants.DriveDirectionToggle == true){
  Constants.DriveDirectionToggle = false;
  Timer.delay(.1);
}

}

// L3 and R3 at same time sets drive toggles etc to default 
if (gamepad.getRightStickButton() && gamepad.getLeftStickButton()){
Constants.DriveDirectionToggle = false;
Constants.DriveSpeedToggle = false;
System.out.println("drive is regular");
}

 //checks booleans for dpad toggles
  if(Constants.DriveSpeedToggle == true && Constants.DriveDirectionToggle == false ){ //checks boolean. if true, run if. if false, run else
    drive.tankDrive(Lstick/2, Rstick/2);
    
  } 
  else if(Constants.DriveDirectionToggle == true && Constants.DriveSpeedToggle == false){ //checks boolean. if true, run if. if false, run else
    drive.tankDrive(Lstick*-1, Rstick*-1);
    
  }
  else if(Constants.DriveDirectionToggle == true && Constants.DriveSpeedToggle == true ){ 
    drive.tankDrive(Lstick*-.5, Rstick*-.5);
  }
  else {
    drive.tankDrive(Lstick, Rstick);
  }


//lift controls
if( LT > 0){
Lift.set(LT);
}

else if(RT > 0){
  Lift.set(-RT);
}

else if(LT > 0 && gamepad.getLeftStickButton()){
//liftHigh()
Constants.LiftMaxHeightCheck = false;
while(Constants.LiftMaxHeightCheck == false ){
  Lift.set(LiftSpeed());
 
}
if (LArm_motor.getSelectedSensorPosition() == 4000){
  Constants.LiftMaxHeightCheck = true;
    }
}




//intake controls
if(gamepad.getLeftBumper()){
  Intake.set(.1);
}else if(gamepad.getRightBumper()){
  Intake.set(-.1);

}else {
  Intake.set(0);
}

//solenoids
//pcmCompressor.disable(); //delete me for compressor 
//do we need to set to reverse in init??

 if (gamepad.getAButton()){
   LeftSol.set(kForward);
   RightSol.set(kForward);
   System.out.println("forward");
   }

  if (gamepad.getBButton())
  {
    LeftSol.set(kReverse);
    RightSol.set(kReverse);
    System.out.println("backwards");
   }




  }//end teleop

  





  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
