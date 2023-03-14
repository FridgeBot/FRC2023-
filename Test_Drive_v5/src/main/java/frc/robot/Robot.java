// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


// We really should put a list of used libraries somewhere Tanya -_-

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //limelight:
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tid = table.getEntry("tid");
  NetworkTableEntry fid = table.getEntry("Fid");
  
  private static final String Middle_Simple = "Middle_Simple";
  private static final String Middle_April = "Middle_April";
  private static final String Multi = "Multi";


  private String m_autoSelected;  


  VictorSP FL = new VictorSP(0);
  VictorSP FR = new VictorSP(1);
  VictorSP BR = new VictorSP(2);
  VictorSP BL = new VictorSP(3);

  Joystick Bitterness = new Joystick(1);
  WPI_TalonFX Falcon = new WPI_TalonFX(10);
  AHRS Alex = new AHRS(Port.kMXP);
  Timer timer = new Timer();
  
  MecanumDrive mecanum = new MecanumDrive(FL, BL, FR, BR);
  
  
  int A = 1;
  int Y = 4;
  int X = 3;
  int B = 2;
  int axisZ = 0;
  int LT = 2;
  int axisX = 4;
  int axisY = 5;
  int RB = 6;
  int LB = 5;
  int RJoy = 10;
  double x = 0.5;
  int i = 0;
  //conversion vars
  double MI = 1/39.37;
  
  double Ang;
  double Ya;
  double Pitch;
  double Angle;
  
  double TX;
  double y;
  double area;
  double Target;
  double Tod;
  //RIP Fod
  //flag vars
  int fl = 1;
  int ga = 0;
  int nc = 0;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture(0);

    BL.setInverted(true);
    BR.setInverted(true);
    FL.setInverted(true);
    FR.setInverted(true);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard
    m_robotContainer = new RobotContainer();
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

    //read values periodically
    TX = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    Target = tv.getDouble(0.0);
    Tod = tid.getDouble(0.0);

    Ang = Alex.getAngle();
    Ya = Alex.getYaw();
    Pitch = Alex.getPitch();
        //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", TX);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("Angle", Ang);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Tod:", Tod);


    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("Test", Middle_Simple);
    m_chooser.addOption("Middle_April", Middle_April);
    m_chooser.addOption("Multi", Multi);

    SmartDashboard.putNumber("Fl", fl);

    SmartDashboard.putNumber("NavX Y", Ya);
    SmartDashboard.putNumber("NavX Z", Ang);
    SmartDashboard.putNumber("Pitch:", Pitch);
    SmartDashboard.putNumber("Speed", FR.get());
    
    SmartDashboard.putNumber("tx", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("i", i);
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Target", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
  

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

    Alex.reset();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    Alex.reset();
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
      case Middle_Simple:
        if(fl == 0){
          if(Alex.getDisplacementX() < -60.72*MI){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else{
            mecanum.driveCartesian(0, 0, 0);
            fl = 1;
          }
        }else if(fl == 1){
          i = 0;
          if(i == 0){
            if(Pitch > 1.6 || Pitch < -1.6){
              int PitchInt = (int)Pitch;
              mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.2, 0, 0);
            }else if(Pitch < 0.5 || Pitch > -0.5){
              mecanum.driveCartesian(0, 0, 0);
              i ++;
            }
          }else if(i == 5){
            mecanum.driveCartesian(0, 0, 0);
          }
        }else{
          mecanum.driveCartesian(0, 0, 0);
          fl = 2;
        }
      break;
      case Middle_April:
        if(fl == 0){
          if(Target == 0){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else if(Target == 1){
            fl = 1;
          }
        }else if(fl ==1){
          if(area > 0.7){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else{
            fl = 2;
          }
        }else if(fl == 2){
          i = 0;
          if(i == 0){
            if(Pitch > 1.6 || Pitch < -1.6){
              int PitchInt = (int)Pitch;
              mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.225, 0, 0);
            }else if(Pitch < 0.5 || Pitch > -0.5){
              mecanum.driveCartesian(0, 0, 0);
              i ++;
            }
          }else if(i == 5){
            mecanum.driveCartesian(0, 0, 0);
          }
        }else{
          fl = 3;
        }
      break;
      case Multi:
      if(Target == 0 && nc == 0){
        mecanum.driveCartesian(-0.5, 0 , 0);
      }else if(Target == 1 && fl == 1){
        nc = 1;
        if(Tod == 2){
          if(area > 0.7 && fl == 1){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else{
            fl = 2;
          }
        }
      }else if(fl == 2){
        timer.start();
        if(timer.get() < 5){
          if(Pitch > 1.6 || Pitch < -1.6){
            int PitchInt = (int)Pitch;
            mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.225, 0, 0);
          }else{
            timer.stop();
          }
        }else if(timer.get() > 5){
          mecanum.driveCartesian(0, 0, 0);
          timer.stop();
        }
      }else{
        fl = 3;
      }



        // }else if(Tod == 1){
        //   if(ga == 0){
        //     if(area > 0.6){
        //       mecanum.driveCartesian(-0.5, 0, 0);
        //     }else{
        //       mecanum.driveCartesian(0, 0, 0);
        //       ga = 1;
        //     }
        //   }else if(ga == 1){
        //     System.out.println("Pick up");
        //     ga = 2;
        //   }else{
        //     mecanum.driveCartesian(0, 0, 0);
        //     //stop all other mechs
        //   }
        // }else if(Tod == 3){

        // }

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
  }

  public void GoBackwards(){
    mecanum.driveCartesian(-0.5, 0, 0);
  }

  public void GoForwards(){
    mecanum.driveCartesian(0.5, 0, 0);
  }

  public void GoLeft(){

    FR.set(x);
    FL.set(-x);
    BR.set(x);
    BL.set(-x);
}

public void GoRight(){

    FR.set(-x);
    FL.set(x);
    BR.set(-x);
    BL.set(x);
}

  public void Stop(){

      FR.set(0);
      FL.set(0);
      BR.set(0);
      BL.set(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    //Alex.reset();

    if(Bitterness.getRawButton(A)){
      GoBackwards();
    }else if(Bitterness.getRawButton(Y)){
      GoForwards();
    }else if(Bitterness.getRawButton(X)){
      GoLeft();
    }else if(Bitterness.getRawButton(B)){
      GoRight();
    }

    else if(Bitterness.getRawButton(LB)){
      i = 0;
      if(i == 0){
        if(Pitch > 1.6 || Pitch < -1.6){
          int PitchInt = (int)Pitch;
          mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.2, 0, 0);
        }else if(Pitch < 0.5 || Pitch > -0.5){
          mecanum.driveCartesian(0, 0, 0);
          i ++;
        }
      }else if(i == 5){
        mecanum.driveCartesian(0, 0, 0);
      }

    }
    else if(Bitterness.getRawButton(RB)){
      if (Target == 1){
        if(TX > 3){
          mecanum.driveCartesian(0, 0, 0.25);
        }else if(TX < -3){
          mecanum.driveCartesian(0, 0, -0.25);
        }else if(TX < 3 && TX > -3){
          if(area < 5){
            mecanum.driveCartesian( 1/area, 0, 0);
          }
        }
      }else if(Target == 0){
        mecanum.driveCartesian(0, 0, 0);
      }
    }else if(Bitterness.getRawButton(RJoy)){
      if(Alex.getAngle() < 180){
        mecanum.driveCartesian(0, 0, 0.5);
      }else{
        mecanum.driveCartesian(0, 0, 0);
      }
    }else {
      FR.set(0);
      FL.set(0);
      BR.set(0);
      BL.set(0);
    }
  }
  
  



  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

