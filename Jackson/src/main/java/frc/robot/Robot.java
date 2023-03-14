// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

//import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //private RobotContainer m_robotContainer;
  
    //limelight:
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tid = table.getEntry("tid");
    //drive motors
    WPI_TalonFX BR = new WPI_TalonFX(9);
    WPI_TalonFX BL = new WPI_TalonFX(2);
    WPI_TalonFX FR = new WPI_TalonFX(1);
    WPI_TalonFX FL = new WPI_TalonFX(3);
    //low wheels
    WPI_TalonFX LW = new WPI_TalonFX(11);
    //ArmMotor Motor
    WPI_TalonFX ArmMotor = new WPI_TalonFX(10);
    //Joysicks
    Joystick Joy = new Joystick(0);
    Joystick Bitterness = new Joystick(1);
    //mecanum drive
    MecanumDrive mecanum = new MecanumDrive(FL, BL, FR, BR);

    //sensors
    AHRS Alex = new AHRS(Port.kMXP);

    private String m_autoSelected;

    private static final String Middle_April = "Middle_April";
    private static final String Right_April = "Right_April";
    private static final String Left_April = "Left_April";

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    //joystick dead zone double values
    //double joyX;
    //double joyY;

    // joy button mapping 
    int axisX = 0;// axis 1
    int axisY = 1;// axis 2
    int axisZ = 2;// axis 3
    int rotX = 3; // axis 4
    int rotY = 4; // axis 5
    int rotZ = 5; // axis 6
    int slider = 6;// axis 7
    int trigger = 1;
    int Fire = 2;
    int A = 3;
    int B = 4;
    int C = 5;
    int pink = 6;
    int D = 7;
    int E = 8;
    int T1 = 9;
    int T2 = 10;
    int T3 = 11;
    int T4 = 12;
    int T5 = 13;
    int T6 = 14;
    int povUp = 16;
    int povRight = 17;
    int povDown = 18;
    int povLeft = 19;
    int thumbUp = 20;
    int thumbRight = 21;
    int thumbDown = 22;
    int thumbLeft = 23;
    int ModeG = 24;
    int ModeO = 25;
    int ModeR = 26;
    int i = 30;
    int button = 31;
    int scroll = 32;
    // bitterness button mapping
    int b = 2;
    int a = 1;
    int X = 3;
    int Y = 4;
    int RB = 6;
    int LB = 5;
    int Start = 8;
    int Back = 7;
    
    int fl = 0;
    
    double InMConverter = 0.0254;
    
    double Pitch;
    double DistX;
    
    //Vars for auto-balance
    double PitchHolder = 2;
    int SwayCounter = 0;
    
    double Target;
    double area;
    
    //public void mechs
    double pvm = 0;
    //Limit Switches
    AnalogInput UpperArmLimit = new AnalogInput(0);
    AnalogInput LowerArmLimit = new AnalogInput(1);
    


    //pheunamtics
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    //low wheels
    Solenoid LWS = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    Solenoid StickIn = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    Solenoid StickOut = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    Solenoid GrabberIn = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
    Solenoid GrabberOut = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    Solenoid ArmSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    //m_robotContainer = new RobotContainer();
    
    FL.setSafetyEnabled(false);
    FR.setSafetyEnabled(false);
    BL.setSafetyEnabled(false);
    BR.setSafetyEnabled(false);
    
    BR.setInverted(true);
    FR.setInverted(true);

    ArmMotor.setInverted(true);
    
    compressor.enableDigital();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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

    Pitch = Alex.getPitch();
    DistX = Alex.getDisplacementX();
    Target = tv.getDouble(0.0);
    area = ta.getDouble(0.0);

    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("Test", Middle_April);
    m_chooser.addOption("Nothing", Middle_April);
    m_chooser.addOption("Right_April", Right_April);
    
    SmartDashboard.putBoolean("Solenoid ArmMotor", ArmSolenoid.get());
    SmartDashboard.putBoolean("Solenoid LWS", LWS.get());
    SmartDashboard.putBoolean("GrabberIn", GrabberIn.get());
    SmartDashboard.putBoolean("GrabberOut", GrabberOut.get());
    SmartDashboard.putBoolean("grabberIn", StickIn.get());
    SmartDashboard.putBoolean("grabberOut", StickOut.get());
    SmartDashboard.putNumber("Pitch", Pitch);
    SmartDashboard.putNumber("SwayCounter", SwayCounter);
    SmartDashboard.putNumber("Alex Dispacement X", DistX);
    SmartDashboard.putNumber("Upper LimitSwitch", UpperArmLimit.getVoltage());
    SmartDashboard.putNumber("Lower Limit Switch", LowerArmLimit.getVoltage());
  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
    case Middle_April:
      if(fl == 0){
        ScoreHighCube();
      }else{
        fl = 1;
      }
      if(fl == 1){
        if(Target == 0){
          mecanum.driveCartesian(-0.7, 0, 0);
        }else if(Target == 1){
          fl = 2;
        }
      }else if(fl == 2){
        if(area > 0.7){
          mecanum.driveCartesian(-0.7, 0, 0);
        }else{
          fl = 3;
        }
      }else if(fl == 3){
          if(Pitch > 1.6 || Pitch < -1.6){
            int PitchInt = (int)Pitch;
            mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.15, 0, 0);
          }else if(Pitch < 0.5 || Pitch > -0.5){
            mecanum.driveCartesian(0, 0, 0);
            i ++;
          }
        }else if(i == 5){
          mecanum.driveCartesian(0, 0, 0);
        }
    break;

  case Right_April:
    //int sc = 0;
    if(fl == 0){
      ScoreHighCube();
    }else{
      fl = 1;
    }
    if(Alex.getDisplacementX() < 224*InMConverter && fl == 1){
      mecanum.driveCartesian(-0.5, 0.1, 0);
    }else{
      //sc = 1;
      fl = 2;
    }
    if(fl == 2 && Alex.getAngle() < 88){
      mecanum.driveCartesian(0, 0, 0.5);

    }else{
        fl = 3;
    }
    if(fl == 4){
      PickUpCube();
    }
  break;
  case Left_April:
  break;
  }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Alex.calibrate();
  }

  public void ScoreHighCube(){
    if(UpperArmLimit.getVoltage() < 0 && pvm == 0){
      ArmMotor.set(0.4);
    }else{
      ArmMotor.set(0);
      pvm = 1;
    }
    if(pvm == 1){
      ArmSolenoid.set(true);
    }else{
      pvm = 2;
    }
    if(pvm == 2){
      GrabberIn.set(true);
      GrabberOut.set(false);
    }else if(LowerArmLimit.getVoltage() < 0){
      ArmMotor.set(-0.4);
    }
    else{
      GrabberIn.set(false);
      GrabberOut.set(true);
      ArmSolenoid.set(false);
      ArmMotor.set(0);
    }
  }

  public void ScoreMidCube(){

  }
  public void ScoreLowCube(){

  }
  public void ScoreHighCone(){

  }
  public void ScoreMidCone(){

  }
  public void ScoreLowCone(){

  }
  public void PickUpCone(){

  }
  
  public void PickUpCube(){
    GrabberIn.set(true);
    GrabberOut.set(false);
  }
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mecanum.driveCartesian(-1*Joy.getRawAxis(axisY), Joy.getRawAxis(axisX), Joy.getRawAxis(rotZ));
    
    int PitchInt = (int)Pitch;

    // StickIn.set(true);
    // StickOut.set(true);
    // GrabberIn.set(true);
    // GrabberOut.set(true);

    // if(Bitterness.getRawButtonPressed(a)){
    //   GrabberOut.set(true);
    //   GrabberIn.set(false);
    // }else if(Bitterness.getRawButtonReleased(a)){
    //   GrabberIn.set(true);
    //   GrabberOut.set(false);
    // }
    //else if(Bitterness.getRawButtonPressed(Y)){
    //   StickIn.set(true);
    //   StickOut.set(false);
    // }else if(Bitterness.getRawButtonReleased(Y)){
    //   StickIn.set(false);
    //   StickOut.set(true);
    // }else if(Bitterness.getRawButton(LB)){
    //   ArmMotor.set(0.5);




      // i = 0;
      // if(i == 0){
      //   if(Pitch > 1.5 || Pitch < -1.5){
      //     int PitchInt = (int)Pitch;
      //     mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.15, 0, 0);
      //   }else if(Pitch < 0.5 || Pitch > -0.5){
      //     mecanum.driveCartesian(0, 0, 0);
      //     i ++;
      //   }
      // }else if(i == 5){
      //   mecanum.driveCartesian(0, 0, 0);
      // }

    //}
    if(Bitterness.getRawButton(b)){
      LWS.set(true);
    }else if(Bitterness.getRawButton(RB)){
      ArmMotor.set(-0.5);
    }
    else if(Bitterness.getRawButtonPressed(X)){
      ArmSolenoid.set(true);
    }
    else if (Bitterness.getRawButtonReleased(X)){
      ArmSolenoid.set(false);
    }
    else if(Joy.getRawButton(i)){
      LWS.set(true);
      if((Pitch > 1.6 || Pitch < -1.6) && SwayCounter < 4){
        LW.set(-1*Integer.signum(PitchInt)*0.16);
        mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.14, 0, 0);
        if((Pitch > 0 && PitchHolder < 0) || (Pitch < 0 && PitchHolder > 0)){
          PitchHolder = Pitch;
          SwayCounter++;
          Alex.resetDisplacement();
        }
      // }else if(Pitch < 0.8 || Pitch > -0.8){
      //   mecanum.driveCartesian(0, 0, 0);
      //   //LWS.set(true);
      //   LW.set(-1*Integer.signum(PitchInt)*0.15);
      } else if(SwayCounter >= 4){
        if(PitchHolder > 0 && Alex.getDisplacementX() > -0.15){
          mecanum.driveCartesian(-0.25, 0, 0);
        }else if(PitchHolder < 0 && Alex.getDisplacementX() < 0.15){
          mecanum.driveCartesian(0.25, 0, 0);
        // }else if(Pitch < 0.6 || Pitch > -0.6){
        } else {
          LW.set(0);
          mecanum.driveCartesian(0, 0, 0);
       }
      }
    

    }
    // if(Joy.getRawButton(A)){
    //   if(UpperArmLimit.getVoltage() < 0.1){
    //     ArmMotor.set(0.2);
    //   }else{
    //     ArmMotor.set(0);
    //   }
    // }else if(Joy.getRawButton(B)){
    //   if(LowerArmLimit.getVoltage() < 0.1){
    //     ArmMotor.set(-0.2);
    //   }else{
    //     ArmMotor.set(0);
    //   }
    // }else{
    //   ArmMotor.set(0);
    // }
      if(Joy.getRawButton(E)){
        //LWS.set(true);
        if(Pitch > 1.6 || Pitch < -1.6){
          //LW.set(-1*Integer.signum(PitchInt)*0.16);
          mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.12, 0, 0);
          LWS.set(false);
        }else if(Pitch < 0.8 || Pitch > -0.8){
          mecanum.driveCartesian(0, 0, 0);
          LWS.set(true);
          LW.set(-1*Integer.signum(PitchInt)*0.12);
        }else if(Pitch < 0.6 || Pitch > -0.6){
          LW.set(0);
          mecanum.driveCartesian(0, 0, 0);
        }
      }
      //competition version
      else if(Bitterness.getRawAxis(axisY) <  -0.5){
        if(UpperArmLimit.getVoltage() < 0.1){
          ArmMotor.set(0.2);
        }else{
          ArmMotor.set(0);
        }
      }else if(Bitterness.getRawAxis(axisY) > 0.5){
        if(LowerArmLimit.getVoltage() < 0.1){
          ArmMotor.set(-0.2);
        }else{
          ArmMotor.set(0);
        }
      }else{
        ArmMotor.set(0);
      }
      
      // else{
      //   mecanum.driveCartesian(0, 0, 0);
      //   ArmSolenoid.set(false);
      //   LWS.set(false);
      //   ArmMotor.set(0);
      // }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
