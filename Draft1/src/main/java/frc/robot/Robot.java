/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private CANSparkMax shooter = new CANSparkMax(1, MotorType.kBrushless);

  private VictorSP leftMotor1 = new VictorSP(0);
  private PWMVictorSPX leftMotor2 = new PWMVictorSPX(1);
  private PWMVictorSPX leftMotor3 = new PWMVictorSPX(2);

  private VictorSP rightMotor1 = new VictorSP(3);
  private PWMVictorSPX rightMotor2 = new PWMVictorSPX(4);
  private PWMVictorSPX rightMotor3 = new PWMVictorSPX(5);

  private SpeedControllerGroup leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
  private SpeedControllerGroup rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
  private DifferentialDrive drive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  private DoubleSolenoid gearShift = new DoubleSolenoid(0, 1);

  private Joystick stick = new Joystick(0);
  private XboxController xStick = new XboxController(1);

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private double tv = limelight.getEntry("ta").getDouble(0);
  private double tx = limelight.getEntry("tx").getDouble(0);
  private double ty = limelight.getEntry("ty").getDouble(0);

  private double h1 = 0;
  private double h2 = 0;
  private double a1 = 0;
  private double d;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);


    gearShift.set(DoubleSolenoid.Value.kForward);

    

    shooter.restoreFactoryDefaults();
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
    
    estimateDistance();

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("d", d);


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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
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
    
    //drive
    drive.arcadeDrive(stick.getY(), stick.getZ());


    //shifing gears
    if (stick.getRawButton(1)) {
      gearShift.set(DoubleSolenoid.Value.kReverse);
    } else if(stick.getRawButton(2)) {
      gearShift.set(DoubleSolenoid.Value.kForward);
    }

    //limelight




    


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }



  public void estimateDistance() {

    d = (h2 - h1) / Math.tan(a1 + ty);

  }
}
