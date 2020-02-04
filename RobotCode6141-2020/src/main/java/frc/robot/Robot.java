/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();
  
  //motors

    //intake
  private Spark intake = new Spark(6);

  private VictorSP leftMotor1 = new VictorSP(0);
  private PWMVictorSPX leftMotor2 = new PWMVictorSPX(1);
  private PWMVictorSPX leftMotor3 = new PWMVictorSPX(2);

  private VictorSP rightMotor1 = new VictorSP(3);
  private PWMVictorSPX rightMotor2 = new PWMVictorSPX(4);
  private PWMVictorSPX rightMotor3 = new PWMVictorSPX(5);

  private SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
  private SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    //shooter
  private PWMSparkMax shooter1 = new PWMSparkMax(7);
  private PWMSparkMax shooter2 = new PWMSparkMax(8);
  private SpeedControllerGroup shooter = new SpeedControllerGroup(shooter1, shooter2);

  //limelight

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private NetworkTableEntry ledMode = limelight.getEntry("ledMode");
  private NetworkTableEntry camMode = limelight.getEntry("camMode");

  private boolean LimelightHasValidTarget = false;
  private double LimelightDriveCommand = 0.0;
  private double LimelightSteerCommand = 0.0;

  //limelight distance tracking

  private final double h1 = 82.5;
  private final double h2 = 115;
  private final double a1 = 5;
  private double distance;
  private final double desiredDistance = 200;

  //sensors

  private ADIS16470_IMU imu = new ADIS16470_IMU();

  //pneumatics
  private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

  // operator input

  private Joystick stick = new Joystick(0);
  private SlewRateLimiter stickFilterY = new SlewRateLimiter(1);
  private SlewRateLimiter stickFilterZ = new SlewRateLimiter(1);
  private SlewRateLimiter throttleFilter = new SlewRateLimiter(1);

  private XboxController xStick = new XboxController(1);
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", chooser);


    gearShift.set(DoubleSolenoid.Value.kForward);

    

    shooter2.setInverted(true);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    //limelight

    updateLimelightTracking();

    if (stick.getRawButton(1)) {
      camMode.setNumber(0);
      ledMode.setNumber(3);
      xStick.setRumble(RumbleType.kLeftRumble, 0.3);
      xStick.setRumble(RumbleType.kLeftRumble, 0.3);

      if (LimelightHasValidTarget){
        driveTrain.arcadeDrive(LimelightDriveCommand, LimelightSteerCommand);
      } else {
        driveTrain.arcadeDrive(0, 0);
      }

    } else {
      //drive
      xStick.setRumble(RumbleType.kLeftRumble, 0);
      xStick.setRumble(RumbleType.kLeftRumble, 0);
      
      camMode.setNumber(1);
      ledMode.setNumber(1);
      
      driveTrain.setMaxOutput(throttleFilter.calculate(((stick.getThrottle()) - 1) / 2));

      driveTrain.arcadeDrive(stickFilterY.calculate(stick.getY()), stickFilterZ.calculate(stick.getZ()));
    }


    //shifing gears
    if (stick.getRawButton(3)) {
      gearShift.set(DoubleSolenoid.Value.kReverse);
    } else if(stick.getRawButton(4)) {
      gearShift.set(DoubleSolenoid.Value.kForward);
    }

    //intake
    intake.set(xStick.getY(Hand.kLeft));

    //shooter
    if (xStick.getBButton()) {
      shooter.set(0.7);
    }
    


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    System.out.println((int)(imu.getAngle()));

    shooter2.set(1);


  }

  public void updateLimelightTracking(){
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 0.03;                    // how hard to drive fwd toward the target
    final double MAX_DRIVE = 0.5;                   // Simple speed limit so we don't drive too fast

    double tv = limelight.getEntry("tv").getDouble(0);
    double tx = limelight.getEntry("tx").getDouble(0);
    double ty = limelight.getEntry("ty").getDouble(0);

    estimateDistance();

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("d", distance);

    if (tv < 1.0)
    {
      LimelightHasValidTarget = false;
      LimelightDriveCommand = 0.0;
      LimelightSteerCommand = 0.0;
      return;
    }

    LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (distance - desiredDistance) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    } else if (drive_cmd < -MAX_DRIVE){
      drive_cmd = -MAX_DRIVE;
    }
    LimelightDriveCommand = drive_cmd;
  }

  public void estimateDistance() {

    double ty = limelight.getEntry("ty").getDouble(0);

    distance = (h2 - h1) / (Math.tan(Math.toRadians(a1 + ty)));

  }
}
