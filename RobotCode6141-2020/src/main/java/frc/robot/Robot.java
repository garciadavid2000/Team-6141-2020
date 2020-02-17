/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.AlternateEncoderType;

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

  // motors

  // intake
  // private Spark intake = new Spark(6);

  private VictorSP leftMotor1 = new VictorSP(1);
  private WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(1);
  private WPI_VictorSPX leftMotor3 = new WPI_VictorSPX(2);
  private VictorSP rightMotor1 = new VictorSP(2);
  private WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(3);
  private WPI_VictorSPX rightMotor3 = new WPI_VictorSPX(4);

  private SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
  private SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
  private DifferentialDrive driveTrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  // shooter
  private CANSparkMax shooter1 = new CANSparkMax(9, MotorType.kBrushless);
  private CANSparkMax shooter2 = new CANSparkMax(8, MotorType.kBrushless);
  private SpeedControllerGroup shooter = new SpeedControllerGroup(shooter1, shooter2);

  // Alt Encoder 1
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;

  private CANPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private CANEncoder m_altEncoder;

  // Alt Encoder 2
  private static final AlternateEncoderType kAltEncType_2 = AlternateEncoderType.kQuadrature;
  private static final int kCPR_2 = 8192;
  private CANPIDController m_pidController_2;
  public double kP_2, kI_2, kD_2, kIz_2, kFF_2, kMaxOutput_2, kMinOutput_2;

  private CANEncoder m_altEncoder2;

  // limelight

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private NetworkTableEntry ledMode = limelight.getEntry("ledMode");
  private NetworkTableEntry camMode = limelight.getEntry("camMode");

  private boolean LimelightHasValidTarget = false;
  private double LimelightDriveCommand = 0.0;
  private double LimelightSteerCommand = 0.0;

  // limelight distance tracking

  private final double h1 = 82.5;
  private final double h2 = 115;
  private final double a1 = 5;
  private double distance;
  private final double desiredDistance = 200;

  // sensors

  private ADIS16470_IMU imu = new ADIS16470_IMU();

  // pneumatics
  private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

  // operator input

  private Joystick stick = new Joystick(0);
  private SlewRateLimiter stickFilterY = new SlewRateLimiter(1);
  private SlewRateLimiter stickFilterZ = new SlewRateLimiter(1);
  private SlewRateLimiter throttleFilter = new SlewRateLimiter(1);

  private XboxController xStick = new XboxController(1);

  // //auto stuff
  // DifferentialDriveKinematics kinematics = new
  // DifferentialDriveKinematics(Units.inchesToMeters(28)); // this is just a
  // random value, insert actual value on tues
  // DifferentialDriveOdometry odometry = new
  // DifferentialDriveOdometry(getHeading(), new Pose2d(0.0, 0.0, new
  // Rotation2d()));// in the video, it says that there
  // should be another arguement for
  // the kinematics we just made,
  // but wpilib doesn't seem to want to have it.
  // endcoders

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    m_altEncoder = shooter1.getAlternateEncoder(kAltEncType, kCPR);
    m_altEncoder2 = shooter2.getAlternateEncoder(kAltEncType_2, kCPR_2);

    m_pidController = shooter1.getPIDController();
    m_pidController_2 = shooter2.getPIDController();

    m_pidController.setFeedbackDevice(m_altEncoder);
    m_pidController_2.setFeedbackDevice(m_altEncoder2);

    // PID Co-efficients
    kP = kP_2 = 0.1;
    kI = kI_2 = 1e-4;
    kD = kD_2 = 1;
    kIz = kIz_2 = 0;
    kFF = kFF_2 = 0;
    kMaxOutput = kMaxOutput_2 = 1;
    kMinOutput = kMinOutput_2 = -1;

    // Set PID Coefficients
    m_pidController.setP(kP);
    m_pidController_2.setP(kP_2);

    m_pidController.setI(kI);
    m_pidController_2.setI(kI_2);

    m_pidController.setD(kD);
    m_pidController_2.setD(kD_2);

    m_pidController.setIZone(kIz);
    m_pidController_2.setIZone(kIz_2);

    m_pidController.setFF(kFF);
    m_pidController_2.setFF(kFF_2);

    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController_2.setOutputRange(kMinOutput_2, kMaxOutput_2);

    // Display PID coefficients to SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    SmartDashboard.putNumber("P Gain 2", kP_2);
    SmartDashboard.putNumber("I Gain 2", kI_2);
    SmartDashboard.putNumber("D Gain 2", kD_2);
    SmartDashboard.putNumber("I Zone 2", kIz_2);
    SmartDashboard.putNumber("Feed Forward 2", kFF_2);
    SmartDashboard.putNumber("Max Output 2", kMaxOutput_2);
    SmartDashboard.putNumber("Min Output 2", kMinOutput_2);
    SmartDashboard.putNumber("Set Rotations 2", 0);

    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", chooser);
    imu.calibrate(); // used to recalibrate gyro when robot is turned on

    gearShift.set(DoubleSolenoid.Value.kForward);

    shooter1.setInverted(true);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
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
    imu.getAngle();// Can be used to get the angle of the gyro
                   // Can also be split into x y and z values
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
    // PID stuff
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    double p2 = SmartDashboard.getNumber("P Gain 2", 0);
    double i2 = SmartDashboard.getNumber("I Gain 2", 0);
    double d2 = SmartDashboard.getNumber("D Gain 2", 0);
    double iz2 = SmartDashboard.getNumber("I Zone 2", 0);
    double ff2 = SmartDashboard.getNumber("Feed Forward 2", 0);
    double max2 = SmartDashboard.getNumber("Max Output 2", 0);
    double min2 = SmartDashboard.getNumber("Min Output 2", 0);
    double rotations2 = SmartDashboard.getNumber("Set Rotations 2", 0);

    // if PID coefficients have changed write new values to controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;

    }
    if ((p2 != kP_2)) {
      m_pidController_2.setP(p2);
      kP_2 = p2;
    }
    if ((i2 != kI_2)) {
      m_pidController_2.setI(i2);
      kI_2 = i2;
    }
    if ((d2 != kD_2)) {
      m_pidController_2.setD(d2);
      kD_2 = d2;
    }
    if ((iz2 != kIz_2)) {
      m_pidController_2.setIZone(iz2);
      kIz_2 = iz2;
    }
    if ((ff2 != kFF_2)) {
      m_pidController_2.setFF(ff2);
      kFF_2 = ff2;
    }
    if ((max2 != kMaxOutput_2) || (min2 != kMinOutput_2)) {
      m_pidController_2.setOutputRange(min2, max2);
      kMinOutput_2 = min2;
      kMaxOutput_2 = max2;
    }

    m_pidController.setReference(rotations, ControlType.kPosition);
    m_pidController_2.setReference(rotations2, ControlType.kPosition);

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariables", m_altEncoder.getPosition());

    
    


    // limelight

    updateLimelightTracking();

    if (stick.getRawButton(1)) {
      camMode.setNumber(0);
      ledMode.setNumber(3);
      xStick.setRumble(RumbleType.kLeftRumble, 0.3);
      xStick.setRumble(RumbleType.kLeftRumble, 0.3);

      if (LimelightHasValidTarget) {
        driveTrain.arcadeDrive(LimelightDriveCommand, LimelightSteerCommand);
      } else {
        driveTrain.arcadeDrive(0, 0);
      }

    } else {
      // drive
      xStick.setRumble(RumbleType.kLeftRumble, 0);
      xStick.setRumble(RumbleType.kLeftRumble, 0);

      camMode.setNumber(1);
      ledMode.setNumber(1);

      driveTrain.setMaxOutput(throttleFilter.calculate(((stick.getThrottle()) - 1) / 2));

      driveTrain.arcadeDrive(stickFilterY.calculate(stick.getY()), stickFilterZ.calculate(stick.getZ()));
    }

    // shifing gears
    if (stick.getRawButton(3)) {
      gearShift.set(DoubleSolenoid.Value.kReverse);
    } else if (stick.getRawButton(4)) {
      gearShift.set(DoubleSolenoid.Value.kForward);
    }

    // intake
    // intake.set(xStick.getY(Hand.kLeft));

    // shooter
    if (xStick.getBButton()) {
      shooter.set(0.7);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    // shooter.set(0);

    // if (xStick.getAButton()) {
    // shooter.set(1);
    // }

    shooter.set(1);

  }

  public void updateLimelightTracking() {
    // These numbers must be tuned for your Robot! Be careful!
    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.03; // how hard to drive fwd toward the target
    final double MAX_DRIVE = 0.5; // Simple speed limit so we don't drive too fast

    double tv = limelight.getEntry("tv").getDouble(0);
    double tx = limelight.getEntry("tx").getDouble(0);
    double ty = limelight.getEntry("ty").getDouble(0);

    estimateDistance();

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("d", distance);

    if (tv < 1.0) {
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
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    } else if (drive_cmd < -MAX_DRIVE) {
      drive_cmd = -MAX_DRIVE;
    }
    LimelightDriveCommand = drive_cmd;
  }

  public void estimateDistance() {

    double ty = limelight.getEntry("ty").getDouble(0);

    distance = (h2 - h1) / (Math.tan(Math.toRadians(a1 + ty)));

  }

  // public Rotation2d getHeading(){
  // return Rotation2d.fromDegrees(-imu.getAngle());

  // public DifferentialDriveWheelSpeeds getSpeeds(){
  // return new DifferentialDriveWheelSpeeds
  // (leftMotor3.getEncoder().getVelocity() /7.29 * 2 * Math.PI *
  // Units.inchesToMeters(6) /60 ,
  // rightMotor3.getEncoder().getVelocity()/7.29 * 2 * Math.PI *
  // Units.inchesToMeters(6) /60
  // );
  // }

  // @Override
  // public void periodic(){
  // odometry.update(getHeading(), getSpeeds() );
  // }
}
