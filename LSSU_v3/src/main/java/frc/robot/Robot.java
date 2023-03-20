// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
    //Grabber Motors
    WPI_TalonSRX LGrabber = new WPI_TalonSRX(7);
    WPI_TalonSRX RGrabber = new WPI_TalonSRX(6);
    //arm motors
    WPI_TalonFX ArmMotor = new WPI_TalonFX(10);
    //low wheels
    WPI_TalonFX LW = new WPI_TalonFX(11);
    //Joysicks
    Joystick Joy = new Joystick(0);
    Joystick Bitterness = new Joystick(1);
    //mecanum drive
    MecanumDrive mecanum = new MecanumDrive(FL, BL, FR, BR);

    //sensors
    AHRS Alex = new AHRS(Port.kMXP);

    private String m_autoSelected;

    private static final String Middle = "Middle";
    private static final String Side_April = "Side_April";
    private static final String Balance_Test = "Balance_Test";
    private static final String Time_Balance_Test = "Time_Balance_Test";
    private static final String Score_High_Cube = "ScoreHighCube";
    private static final String Middle_April_Extended = "Middle_April_Extended";
    private static final String Time_Side_April = "Time_Side_April";
    private static final String Side_No_Score = "Side_No_Score";
    private static final String Encoder_Back = "Encoder_Back";
    private static final String Middle_Simple = "Middle_Simple";

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    Timer auton_timer = new Timer();
    Timer score_timer = new Timer();
    Timer balance_timer = new Timer();

    //joystick dead zone double values
    //double joyX;
    //double joyY;
    // commeeeeent

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
    int LYAxis = 1;
    int RYAxis = 5;
    int b = 2;
    int a = 1;
    int X = 3;
    int Y = 4;
    int RB = 6;
    int LB = 5;
    int Start = 8;
    int Back = 7;
    int joydown = 10;
    
    int fl = 0;

    int balance_flag = 0;
    
    double InMConverter = 0.0254;
    double distEncode = 26465/18; //26,465 units per 18 inches
    
    double Pitch;
    double DistX;
    
    //Vars for auto-balance
    double PitchHolder = 2;
    int SwayCounter = 0;
    double Balance_Dist = 0;

    int PitchStep = 0;
    
    double Target;
    double TX;
    double area;

    int PitchInt = (int)Pitch;

    double bLeftPos;

    int pipes;
    
    
    //public void mechs
    double pvm = 0;
    //Limit Switches
    AnalogInput UpperArmLimit = new AnalogInput(1);
    AnalogInput LowerArmLimit = new AnalogInput(0);
    
    
    //double UpperArmLimitVar;
    //double LowerArmLimitVar;
    
    
    //pheunamtics
    Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    //low wheels
    Solenoid LWS = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.
      //m_robotContainer = new RobotContainer();
      
      CameraServer.startAutomaticCapture(0);

      //ArmMotor.setSelectedSensorPosition(0);
      
      FL.setSafetyEnabled(false);
      FR.setSafetyEnabled(false);
      BL.setSafetyEnabled(false);
      BR.setSafetyEnabled(false);
      LGrabber.setNeutralMode(NeutralMode.Brake);
      RGrabber.setNeutralMode(NeutralMode.Brake);


      
      
      BR.setInverted(true);
      FR.setInverted(true);
      
      ArmMotor.setInverted(true);

      RGrabber.setInverted(true);
      
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
      TX = tx.getDouble(0.0);
      area = ta.getDouble(0.0);
      
          
      //UpperArmLimitVar = UpperArmLimit.getVoltage();
      //LowerArmLimitVar = LowerArmLimit.getVoltage();

      SmartDashboard.putData("Auto choices", m_chooser);
      m_chooser.addOption("Middle_April", Middle);
      m_chooser.addOption("Side_April", Side_April);
      m_chooser.addOption("Balance_Test", Balance_Test);
      m_chooser.addOption("Time_Balance_Test", Time_Balance_Test);
      m_chooser.addOption("Score_Hight_Cube", Score_High_Cube);
      m_chooser.addOption("Middle_April_Extended", Middle_April_Extended);
      m_chooser.addOption("Time_Side_April", Time_Side_April);
      m_chooser.addOption("Side_No_Score", Side_No_Score);
      m_chooser.addOption("Encoder_Back", Encoder_Back);
      m_chooser.addOption("Middle_Simple", Middle_Simple);

      bLeftPos = BL.getSelectedSensorPosition(TalonFXFeedbackDevice.IntegratedSensor.value);
      
      SmartDashboard.putBoolean("Solenoid LWS", LWS.get());
      SmartDashboard.putNumber("Pitch", Pitch);
      SmartDashboard.putNumber("SwayCounter", SwayCounter);
      SmartDashboard.putNumber("Alex Dispacement X", DistX);
      SmartDashboard.putNumber("Upper Limit Switch", UpperArmLimit.getVoltage());
      SmartDashboard.putNumber("Lower Limit Switch", LowerArmLimit.getVoltage());
      SmartDashboard.putNumber("auton_timer", auton_timer.get());
      SmartDashboard.putNumber("pvm", pvm);
      SmartDashboard.putNumber("score_timer", score_timer.get());
      SmartDashboard.putNumber("balance speed", (-1*Integer.signum(PitchInt)*0.15)/((auton_timer.get())*0.1+2));
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("fl", fl);
      SmartDashboard.putNumber("BL encoder", bLeftPos);
      SmartDashboard.putNumber("DisplacementX", Alex.getDisplacementX());
      SmartDashboard.putNumber("DisplacementY", Alex.getDisplacementY());
      SmartDashboard.putNumber("DisplacementZ", Alex.getDisplacementZ());
      SmartDashboard.putNumber("Target", Target);
      SmartDashboard.putNumber("Arm Encoder", ArmMotor.getSelectedSensorPosition());
      
  }
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    pipes = 0;
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    BL.setSelectedSensorPosition(0);
    auton_timer.reset();
    auton_timer.stop();
    balance_timer.reset();
    balance_timer.stop();
    fl = 0;
    pvm = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipelines").setNumber(pipes);
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
      case Middle_Simple:
        if(fl == 0 && auton_timer.get() < 2){
          auton_timer.start();
          LGrabber.set(1);
          RGrabber.set(1);
        }else if(fl == 0){
          LGrabber.set(0);
          RGrabber.set(0);
          auton_timer.stop();
          fl = 3;
        }
        if(fl == 3){
          // if(BL.get() < 0.5){
          //   LWS.set(true);
          //   LW.set(BR.get()*2);
          // }
          if(Pitch < 5){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else if(Pitch > 7){
            mecanum.driveCartesian(0, 0, 0);
            balance_timer.start();
            fl = 4;
          }
        }if(fl == 4 && balance_timer.get() > 0.4){
          if(Pitch < 5){
            mecanum.driveCartesian(0.8, 0, 0);
          }else{
            fl = 5;
          }
        }else
        if(fl == 5){
          daniBalance();
        }
      break;
      case Middle:
        if(fl == 0 && auton_timer.get() < 2){
          ScoreHighCone();
        }else if(fl == 0){
          fl = 1;
        }if(fl == 1 && BL.getSelectedSensorPosition() > -30000){
          mecanum.driveCartesian(-0.45, 0, 0);
        }else if(fl == 1){
          mecanum.driveCartesian(0, 0, 0);
          fl = 2;
        }
        if(fl == 2 && LowerArmLimit.getVoltage() < 3){
          ArmMotor.set(-0.85);
        }else if(fl == 2 && LowerArmLimit.getVoltage() > 3){
          ArmMotor.set(0);
          fl = 3;
        }
        // if(fl == 1){
        //   if(Target == 0){
        //     mecanum.driveCartesian(-0.7, 0, 0);
        //   }else if(Target == 1){
        //     fl = 2;
        //   }
        // }else if(fl == 2){
        //   if(area > 0.7){
        //     mecanum.driveCartesian(-0.7, 0, 0);
        //   }else{
        //     fl = 3;
        //   }
        // }
        if(fl == 3){
          // if(BL.get() < 0.5){
          //   LWS.set(true);
          //   LW.set(BR.get()*2);
          // }
          if(Pitch < 5){
            mecanum.driveCartesian(-0.5, 0, 0);
          }else if(Pitch > 7){
            mecanum.driveCartesian(0, 0, 0);
            balance_timer.start();
            fl = 4;
          }
        }if(fl == 4 && balance_timer.get() > 0.4){
          if(Pitch < 5){
            mecanum.driveCartesian(0.8, 0, 0);
          }else{
            fl = 5;
          }
        }else
        if(fl == 5){
          daniBalance();
        }

        // else if(fl == 3){
        //   Dist_Balance();
        // }
      break;

      case Time_Balance_Test:
      if(fl ==0){
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
        Timed_Balance();
      }
      break;
      case Middle_April_Extended:
        if(fl == 0){
          fl = 1;
        }
        else if(fl == 1){
          if(Target == 0){
            mecanum.driveCartesian(-0.7, 0, 0);
          }else if(Target == 1){
            fl = 2;
          }
        }else if(fl == 2){
          if(area > 0.65){
            mecanum.driveCartesian(-0.7, 0, 0);
          }else{
            fl = 3;
          }
        }else if(fl == 3){
          Simple_Balance();
      }
      break;
        // case Side_No_Score:
        //   auton_timer.start();
        //   if(fl == 0){
        //     if(auton_timer.get() < 1){
        //       mecanum.driveCartesian(-0.6, 0, 0);
        //     }else{
        //       fl = 1;
        //     }
        //   }else if(fl ==1){
        //     if(Alex.getAngle() < 1000){
        //       mecanum.driveCartesian(0, 0, 0.5);
        //     }else{
        //       mecanum.driveCartesian(0, 0, 0);
        //       fl = 2;
        //     }
        //   }
        // break;
      case Side_April:
        //int sc = 0;
        if(fl == 0 && auton_timer.get() < 2){
          ScoreHighCone();
        }else if(fl == 0){
          fl = 1;
        }
        else if(fl == 1){
          if(Target == 0){
            if(LowerArmLimit.getVoltage() > 3 && auton_timer.get() > 4){
              ArmMotor.set(-0.7);
            }else if(LowerArmLimit.getVoltage() < 3){
              ArmMotor.set(0);
            }
            mecanum.driveCartesian(-0.5, 0, 0);
          }else if(Target == 1){
            fl = 2;
          }
        }else if(fl == 2){
          if(area > 0.17){
            if(LowerArmLimit.getVoltage() > 3 && auton_timer.get() > 4){
              ArmMotor.set(-0.7);
            }else if(LowerArmLimit.getVoltage() < 3){
              ArmMotor.set(0);
            }
            mecanum.driveCartesian(-0.5, 0, 0);
          }else{
            fl = 3;
          }
        }else if(fl == 3){
          if(LowerArmLimit.getVoltage() < 3){
            ArmMotor.set(-0.7);
          }else if(LowerArmLimit.getVoltage() > 3){
            ArmMotor.set(0);
          }
          mecanum.driveCartesian(0, 0, 0);
        }

      break;

      
      case Encoder_Back:
        if(fl == 0){
          if(BL.getSelectedSensorPosition() > -90*distEncode){
            mecanum.driveCartesian(-0.3, 0, 0);
          }else{
            mecanum.driveCartesian(0, 0, 0);
            LWS.set(true);
            fl = 1;
          }
        }
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
    pipes = 1;

  }

  public void ScoreHighCube(){
    // if(pvm == 0 && area > 1){
    //   mecanum.driveCartesian(-0.5, 0, 0);
    // }else{
    //   mecanum.driveCartesian(0, 0, 0);
    //   pvm = 1;
    // }
    if(pvm == 0 && UpperArmLimit.getVoltage() > 3){
      ArmMotor.set(0.7);
      mecanum.driveCartesian(-0.12, 0, 0);
    }else if(pvm == 0 && UpperArmLimit.getVoltage() < 3){
      ArmMotor.set(0);
      pvm = 2;
    }if(pvm == 2 && area < 0.3 && Target == 1){
      mecanum.driveCartesian(0.5, 0, 0);
    }else if(pvm == 2 && area > 0.6){
      mecanum.driveCartesian(0, 0, 0);
      pvm = 3;
      auton_timer.start();
    }
    if(pvm == 3){
      if(auton_timer.get() > 1){
        LGrabber.set(0.3);
        RGrabber.set(0.3);
      }
      if(auton_timer.get() > 2){
        LGrabber.set(0);
        RGrabber.set(0);
        pvm = 4;
      }
    }
  }
  public void ScoreHighCone(){
    if(pvm == 0){
      BL.setSelectedSensorPosition(0);
      pvm = 1;
    }
    if(pvm == 1 && UpperArmLimit.getVoltage() > 3){
      ArmMotor.set(0.85);
      if(Pitch < -2){
        mecanum.driveCartesian(0, 0, 0);
      }else{
        mecanum.driveCartesian(-0.15, 0, 0);
      }
    }else if(pvm == 1 && UpperArmLimit.getVoltage() < 3){
      // if(ArmMotor.getSelectedSensorPosition() > 600000){
      //   ArmMotor.set(-0.6);
      // }else if(pvm == 3){
      //   ArmMotor.set(0);
      //   pvm = 4;
      // }
      ArmMotor.set(0);
      pvm = 2;
    }
    if(pvm == 2 && BL.getSelectedSensorPosition() < -3000){
      mecanum.driveCartesian(0.3, 0, 0);
    }
    else if(pvm == 2){
      mecanum.driveCartesian(0, 0, 0);
      pvm = 3;
      auton_timer.start();
    }
    if(pvm == 4){
      if(auton_timer.get() > 1){
        LGrabber.set(0.3);
        RGrabber.set(0.3);
      }else if(auton_timer.get() > 2){
        LGrabber.set(0);
        RGrabber.set(0);
        pvm = 5;
      }
    }
  }

  public void Dist_Balance(){
    if(Pitch > 1.6 || Pitch < -1.6){
      mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.15, 0, 0);
      Alex.resetDisplacement();
    }else if(Pitch < 0.8 && Pitch > -0.8){
      if(Alex.getDisplacementX() < 0.5 && Alex.getDisplacementX() > -0.5){
        mecanum.driveCartesian(-0.1, 0, 0);
      }else{
        mecanum.driveCartesian(0, 0, 0);
      }
    }
  }
  public void Simple_Balance(){
    LWS.set(true);
    LW.set(ControlMode.Follower, BL.get()*2);
    if(Pitch > 1.6 || Pitch < -1.6){
      //LW.set(-1*Integer.signum(PitchInt)*0.16);
      mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.15, 0, 0);
      //LWS.set(false);
    }else if(Pitch < 0.8 || Pitch > -0.8){
      mecanum.driveCartesian(-1*Integer.signum(PitchInt)*0.09, 0, 0);
      //LWS.set(true);
      //LW.set(-1*Integer.signum(PitchInt)*0.1);
    }else if(Pitch < 0.6 || Pitch > -0.6){
      LWS.set(true);
      LW.set(0);
      mecanum.driveCartesian(0, 0, 0);
    }
  }
  public void Timed_Balance(){
    if(Pitch > 1.6 || Pitch < -1.6){
      int PitchInt = (int)Pitch;
      mecanum.driveCartesian((-1*Integer.signum(PitchInt)*0.15)/((auton_timer.get())*0.1+2), 0, 0);
    }else if(Pitch < 0.6 && Pitch > -0.6){
      auton_timer.start();
      mecanum.driveCartesian(0, 0, 0);
      LWS.set(true);
    }
  }
  public void Pitch_Balance(){
    if(BL.get() < 0.5){
      LWS.set(true);
      LW.set(BR.get()*2);
    }
    if((Pitch < 14 && Pitch > -14) && PitchStep == 0){
      mecanum.driveCartesian(Integer.signum(PitchInt)*0.499, 0, 0);
    }else{
      PitchStep = 1;
    }
    if(PitchStep == 1 && (Pitch > 14 && Pitch < -14)){
      mecanum.driveCartesian(0, 0, 0);
    }
  }

  public void daniBalance(){
    Pitch = Alex.getPitch();

    double outPut = Pitch*0.012;
    if(BL.get() < 0.5){
        LWS.set(true);
        LW.set(BR.get()*2);
      }
    mecanum.driveCartesian(outPut, 0, 0);



  }
  
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipelines").setNumber(pipes);
    //mecanum drive
    mecanum.driveCartesian(-1*Joy.getRawAxis(axisY), Joy.getRawAxis(axisX), Joy.getRawAxis(rotZ));
    
    //int PitchInt = (int)Pitch;

    //Arm Motor
    if(Bitterness.getRawAxis(LYAxis) <  -0.1){
      if(UpperArmLimit.getVoltage() > 3){
        ArmMotor.set(-0.75*Bitterness.getRawAxis(axisY));
      }else{
        ArmMotor.set(0);
      }
    }else if(Bitterness.getRawAxis(LYAxis) > 0.1){
      if(LowerArmLimit.getVoltage() < 3){
        ArmMotor.set(-0.75*Bitterness.getRawAxis(axisY));
      }else{
        ArmMotor.set(0);
      }
    }else{
      ArmMotor.set(0);
    }
    
    if(Bitterness.getRawButton(a)){
      if(ArmMotor.getSelectedSensorPosition() > 550000){
        ArmMotor.set(-0.7);
      }else if(ArmMotor.getSelectedSensorPosition() < 510000){
        ArmMotor.set(0.7);
      }else{
        ArmMotor.set(0);
      }
    }
    //Grabber Motors
    if(Bitterness.getRawAxis(RYAxis) < -0.1){
      LGrabber.set(-1*Bitterness.getRawAxis(RYAxis));
      RGrabber.set(-1*Bitterness.getRawAxis(RYAxis));
    }else if(Bitterness.getRawAxis(RYAxis) > 0.1){
      LGrabber.set(-1*Bitterness.getRawAxis(RYAxis));
      RGrabber.set(-1*Bitterness.getRawAxis(RYAxis));
    }else if(Bitterness.getRawButton(RB)){
      LGrabber.set(-0.3);
      RGrabber.set(-0.3);
    }else{
      RGrabber.set(0);
      LGrabber.set(0);
    }

    
    //Drop-down Wheels
    if(BL.get() < 0.5){
      if(Joy.getRawButton(Fire) || Bitterness.getRawButton(X)){
        LWS.set(true);
        //LW.set(ControlMode.Follower, BR.get()*2);
        LW.set(BR.get()*2);
      }
      // else if(Joy.getRawButton(A)){
        //   LWS.set(true);
        //   LW.set(0.5);
        // }else if(Joy.getRawButton(B)){
          //   LW.set(-0.5);
          //   LWS.set(true);
          // }
          else{
            LW.set(0);
            LWS.set(false);
          }
        }
        if(Bitterness.getRawButton(Start)){
          ArmMotor.setSelectedSensorPosition(0);
        }
        
        // if(Joy.getRawButton(D)){
          //   ScoreHighCube();
          // }
          
          if(Joy.getRawButton(E)){
            auton_timer.reset();
            BL.setSelectedSensorPosition(0);
            pvm = 0;
          }
          
          // if(Bitterness.getRawButton(LB)){
            //   LGrabber.set(1);
            // }
            // else if (Bitterness.getRawButton(RB)){
    //   RGrabber.set(1);
    // }else{
      //   LGrabber.set(0);
      //   RGrabber.set(0);
      // }
      //Auto_lining for Cone
      if(Bitterness.getRawButton(LB)){
        if(TX > 3 || TX < -3){
          mecanum.driveCartesian(0, 0, TX*0.1);
        }else{
          mecanum.driveCartesian(0, 0, 0);
        }
      }else{
        Target = 0;
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
