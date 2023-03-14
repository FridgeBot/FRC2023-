// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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




  
  VictorSP FL = new VictorSP(2);
  VictorSP FR = new VictorSP(8);
  VictorSP BR = new VictorSP(9);
  VictorSP BL = new VictorSP(3);

  Joystick Bitterness = new Joystick(1);
  WPI_TalonFX Falcon = new WPI_TalonFX(10);
  AHRS Alex = new AHRS(Port.kMXP);

  int A = 1;
  int Y = 4;
  int X = 3;
  int B = 2;
  int axisZ = 0;
  int LT = 2;
  int axisX = 4;
  int axisY = 5;
  int RB = 6;

  double Ang;
  double Ya;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    BL.setInverted(true);
    BR.setInverted(true);
    FL.setInverted(true);
    FR.setInverted(true);


    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard

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

    Ang = Alex.getAngle();
    Ya = Alex.getYaw();
    SmartDashboard.putNumber("NavX Y", Ya);
    SmartDashboard.putNumber("NavX Z", Ang);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //Alex.reset();
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

  // public void GoBackwards(){

  //     FR.set(-1);
  //     FL.set(-1);
  //     BR.set(-1);
  //     BL.set(-1);
  // }

  // public void GoForwards(){

  //     FR.set(1);
  //     FL.set(1);
  //     BR.set(1);
  //     BL.set(1);
  // }

  // public void Stop(){

  //     FR.set(0);
  //     FL.set(0);
  //     BR.set(0);
  //     BL.set(0);
  // }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    //Alex.reset();

    if(Bitterness.getRawButton(A)){
      FR.set(-1);
      FL.set(-1);
      BR.set(-1);
      BL.set(-1);
    }else if(Bitterness.getRawButton(Y)){
      //GoForwards();
    }else if(Bitterness.getRawButton(X)){
      FR.set(1);
      FL.set(-1);
      BR.set(1);
      BL.set(-1);
    }else if(Bitterness.getRawButton(B)){
      FR.set(-1);
      FL.set(1);
      BR.set(-1);
      BL.set(1);
    }
    // else if(Bitterness.getRawButton(RB)){
    //   if(Alex.getRoll() > 0.5){
    //     GoBackwards();
    //   }else if(Alex.getRoll() < -0.5){
    //     GoForwards();
    //   }else{
    //     Stop();
    //   }
    // }


    // }else if(Bitterness.getRawAxis(axisX) != 0){
    //   Victor.set(Bitterness.getRawAxis(axisX));
    //   victor.set(-1*Bitterness.getRawAxis(axisX));
    // }else if(Bitterness.getRawAxis(axisY) != 0){
    //   Victor.set(Bitterness.getRawAxis(axisY));
    //   victor.set(Bitterness.getRawAxis(axisY));
    
    // else if(Bitterness.getRawButton(RB)){
    //   Falcon.set(1);
    // }
    else{
      FR.set(0);
      FL.set(0);
      BR.set(0);
      BL.set(0);
      //Falcon.set(0);
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
