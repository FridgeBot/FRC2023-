// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

  private RobotContainer m_robotContainer;
  
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
    WPI_TalonSRX LW = new WPI_TalonSRX(7);
    //Arm Motor
    WPI_TalonSRX Arm = new WPI_TalonSRX(8);
    //Joysicks
    Joystick Joy = new Joystick(0);
    Joystick Bitterness = new Joystick(1);
    //mecanum drive
    MecanumDrive mecanum = new MecanumDrive(FL, BL, FR, BR);

    //sensors
    AHRS Alex = new AHRS(Port.kMXP);

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

    //Limit Switches
    AnalogInput MainArm = new AnalogInput(0);
    AnalogInput LoserArm = new AnalogInput(1);
    


    //pheunamtics
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    //low wheels
    Solenoid LWS = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    Solenoid grabber = new Solenoid(PneumaticsModuleType.REVPH, 5);
    Solenoid Grabber = new Solenoid(PneumaticsModuleType.REVPH, 2);
    Solenoid arm = new Solenoid(PneumaticsModuleType.CTREPCM, 6);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    FL.setSafetyEnabled(false);
    FR.setSafetyEnabled(false);
    BL.setSafetyEnabled(false);
    BR.setSafetyEnabled(false);
    
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
    mecanum.driveCartesian(-1*Joy.getRawAxis(axisY), Joy.getRawAxis(axisX), -1*Joy.getRawAxis(rotZ));

    if(Bitterness.getRawButton(Y)){
      grabber.set(true);
    }else if(Bitterness.getRawButton(A)){
      Grabber.set(true);
    }else if(Bitterness.getRawButton(X)){
      arm.set(true);
    }else if(Bitterness.getRawButton(B)){
      LWS.set(true);
    }
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
