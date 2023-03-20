// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String Calibration = "Calibration";
  private static final String AutoNavPath = "AutoNavPath";
  private static final String AutoNavPath2 = "AutoNavPath2";
  private static final String AutoNavPath3 = "AutoNavPath3";
  private String m_autoSelected;
  WPI_TalonSRX left = new WPI_TalonSRX(4);
  WPI_TalonSRX Left = new WPI_TalonSRX(3);
  WPI_TalonSRX Right = new WPI_TalonSRX(1);
  WPI_TalonSRX right = new WPI_TalonSRX(0);
  
  MecanumDrive mecanum = new MecanumDrive(Left, left, Right, right);
  Joystick joy = new Joystick(0);

  Solenoid intakeSolenoidIn = new Solenoid(4);
  Solenoid intakeSolenoidOut = new Solenoid(5);
  Solenoid climberSolenoid3 = new Solenoid(3);
  Solenoid climberSolenoid2 = new Solenoid(2);
  Solenoid kicker = new Solenoid(6);
  WPI_TalonSRX intakewheels = new WPI_TalonSRX(5);
  WPI_TalonSRX climber = new WPI_TalonSRX(8);
  AnalogInput tClimb = new AnalogInput(3);
  AnalogInput bClimb = new AnalogInput(2);
  CANSparkMax ShooterR = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax ShooterL = new CANSparkMax(1, MotorType.kBrushless);

  I2C.Port navPort = I2C.Port.kOnboard;
  AHRS navX = new AHRS(navPort);

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

  double shootSpeed = 1;
  boolean on = true;
  int stop = 0;

  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("Calibration", Calibration);
    m_chooser.addOption("AutoNavPath", AutoNavPath);
    m_chooser.addOption("AutoNavPath2", AutoNavPath2);
    m_chooser.addOption("AutoNavPath3", AutoNavPath3);
    //climber.configSelectedFeedbackSensor();
    Right.setSafetyEnabled(false);
    right.setSafetyEnabled(false);
    Left.setSafetyEnabled(false);
    left.setSafetyEnabled(false);
    mecanum.setSafetyEnabled(false);

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

    if(joy.getRawButton(T6)){
      navX.reset();
      navX.resetDisplacement();
    }
    SmartDashboard.putNumber("DisplaceX", navX.getDisplacementX());
    SmartDashboard.putNumber("DisplaceY", navX.getDisplacementY());
    SmartDashboard.putNumber("RotZ", navX.getYaw());

    SmartDashboard.putNumber("RawX", navX.getRawAccelX());
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
    navX.reset();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Timer.delay(0.02);
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
      case Calibration:
        if(navX.getDisplacementY() < 0.24){
          mecanum.driveCartesian(0, 0.3, 0);
        }else if(navX.getYaw() > -45){
          mecanum.driveCartesian(0, 0, -1);
        }else{
          mecanum.driveCartesian(0, 0, 0);
        }
        break;
      case AutoNavPath:
        double Dis [] = {63, 36.9, 32.4, 37.8, 32.4, 37.8, 63, 45, 37.8, 32.4, 28.8, 125.1, 40.5, 28.8, 117, 28.8, 138};
        double Ang [] = {-5.9, 16, 36.7, 98.7, 81.3, 98.7, 28.1, -18.6, -101.3, -94.8, -61, -47.8, -77.1, -82, -51, 22.5, -22.5};
        int pos = 0;
        for (int x = 0; x < 17; x ++){
          
          if(navX.getYaw() > Ang[x] && Ang[x] < 0 && pos == 0){
            mecanum.driveCartesian( 0, 0, (-1.0));
          }
          else if (navX.getYaw() < Ang[x] && Ang[x] > 0 && pos == 0){
            mecanum.driveCartesian( 0, 0, (1.0));
          }
          else{
            pos = 1;
          }
          if(navX.getDisplacementY() > Dis[x]/100 && pos == 1){
            mecanum.driveCartesian( 0, -0.3, 0);
          }else{
            pos = 0;
            navX.reset();
            navX.resetDisplacement();
          }
          
        }
        break;
      case AutoNavPath2:
        double Dis2 [] = {22.4, 56.6, 44.7, 70, 36.1, 44.7, 22.4, 20, 36.1, 36.1, 20, 22.4, 44.7, 22.4, 80, 44.7, 56.6, 22.4};
        double Ang2 [] = {-26.6, -18.4, 18.4, 18.4, 33.7, 29.7, -39.4, -26.6, -56.3, -67.4, -60.3, -26.6, -31.7, 26.6, 26.6, 14, 22.5, -22.5};
        int dir2 = 1;
        for (int x = 0; x < 18; x++){
          if (Ang2[x] < 0){dir2 = -1;}else{dir2 = 1;}

          if(Math.abs(navX.getYaw()) < Math.abs(Ang2[x])){
            mecanum.driveCartesian( 0, 0, (-1.0*dir2));
          }else if(navX.getDisplacementY() < Dis2[x]/100){
            mecanum.driveCartesian( 0, 0.3, 0);
          }else{
            mecanum.driveCartesian( 0, 0, 0);
            navX.reset();
            navX.resetDisplacement();
          }
        }
        break;
      case AutoNavPath3:
      double Dis3[] = {28.3, 41.2, 76.2, 50, 28.3, 90.6, 90.6, 28.3, 30, 28.3, 90.6, 41.2, 2.24};
      double Ang3[] = {-45, -30.9, 99.2, -3.1, -90, -41.9, 167.4, -38.7, -50.2, -45, -41.9, 159.7, -30};
      int dir3 = 1;
      for (int x = 0; x < 13; x++){
        if (Ang3[x] < 0){dir3 = -1;}else{dir3 = 1;}

        if(Math.abs(navX.getYaw()) < Math.abs(Ang3[x])){
          mecanum.driveCartesian( 0, 0, (-1.0*dir3));
        }else if(navX.getDisplacementY() < Dis3[x]/100){
          mecanum.driveCartesian( 0, 0.3, 0);
        }else{
          mecanum.driveCartesian( 0, 0, 0);
          navX.reset();
          navX.resetDisplacement();
        }
      }
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Timer.delay(0.02);
    mecanum.driveCartesian(joy.getX(), -1 * joy.getY(), joy.getRawAxis(5));
    shooterMech();
    intakeMech();
    climberMech();
  }
  public void shooterMech() {
    if (joy.getRawButton(T6)) {
      shootSpeed = 1;
      SmartDashboard.putBoolean("Shooter Speed", true);
    } else if (joy.getRawButton(T5)) {
      shootSpeed = 0.925;
      SmartDashboard.putBoolean("Shooter Speed", false);
    }

    if (joy.getRawButton(D)) {
      kicker.set(on);
    } else {
      kicker.set(false);
    }

    if(joy.getRawButton(Fire)){
      ShooterR.set(shootSpeed);
      ShooterL.set(-1*shootSpeed);
    }else{
      ShooterR.set(0);
      ShooterL.set(0);
    }
  }

    public void intakeMech() {
      if (joy.getRawButton(B)) {
        intakeSolenoidIn.set(on);
        intakeSolenoidOut.set(false);
      } else if (joy.getRawButton(A)) {
        intakeSolenoidIn.set(false);
        intakeSolenoidOut.set(on);
      } else {
        intakeSolenoidIn.set(false);
        intakeSolenoidOut.set(false);
      }
      if (joy.getRawButton(trigger) || joy.getRawButton(D)) {
        intakewheels.set(1);
      } else if (joy.getRawButton(pink)) {
        intakewheels.set(-1);
      } else {
        intakewheels.set(0);
      }
    }
    int armTog = 0;
    boolean arming = false;
    public void climberMech() {
      if(joy.getRawButton(E) && armTog == 0){
        armTog = 1;
      }else if(!joy.getRawButton(E) && armTog == 1){
        armTog = 2;
      }else if(joy.getRawButton(E) && armTog == 2){
        armTog = 3;
      }else if(!joy.getRawButton(E) && armTog == 3){
        armTog = 0;
      }
      if(joy.getRawButton(T1)){
        arming = true;
      }
      if ((armTog == 1 || armTog == 2) && arming) {
        climberSolenoid3.set(false);
        climberSolenoid2.set(true);
      } else {
        climberSolenoid3.set(true);
        climberSolenoid2.set(false);
      }
      if (joy.getRawButton(povUp) && tClimb.getVoltage() < 3) {
        climber.set(1);
      } else if (joy.getRawButton(povDown) && bClimb.getVoltage() < 3) {
        climber.set(-1);
      } else {
        climber.set(0);
      }
      SmartDashboard.putBoolean("Armed Climber Solenoid", arming);
      SmartDashboard.putNumber("top", tClimb.getVoltage());
    }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
}
