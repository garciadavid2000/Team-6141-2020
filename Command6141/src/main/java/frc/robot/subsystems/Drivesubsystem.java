/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivesubsystem extends SubsystemBase {
  
  public static Object m_robotDrive;
  private final CANSparkMax leftMaster = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
  private final WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(1);
  private final WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(2);

  private final CANSparkMax rightMaster = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
  private final WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(3);
  private final WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(4);

  private final DifferentialDrive m_Drive = new DifferentialDrive(leftMaster, rightMaster);

  //encoders
  private final CANEncoder m_LeftEncoder = new CANEncoder(leftMaster);
  private final CANEncoder m_RightEncoder = new CANEncoder(rightMaster);

  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  private final DifferentialDriveOdometry m_Odometry;


  /**
   * Creates a new Drivesubsystem.
   */
  public Drivesubsystem() {
    leftSlave1.follow((IMotorController) leftMaster);
    leftSlave2.follow((IMotorController) leftMaster);

    rightSlave1.follow((IMotorController) rightMaster);
    rightSlave2.follow((IMotorController) rightMaster);

    m_LeftEncoder.setPosition(0);
    m_RightEncoder.setPosition(0);

    m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getAngle()));




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(Rotation2d.fromDegrees(imu.getAngle()), m_LeftEncoder.getPosition(), m_RightEncoder.getPosition());


  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(m_LeftEncoder.getVelocity(), m_RightEncoder.getVelocity());
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_Odometry.resetPosition(pose, Rotation2d.fromDegrees(imu.getAngle()));

  }

  public void arcadeDrive(double fwd, double rot){
    m_Drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    m_Drive.feed();

  }


  public void resetEncoders(){
    m_LeftEncoder.setPosition(0);
    m_RightEncoder.setPosition(0);
  }

  public double getAverageEncoderPosition(){
    return (m_LeftEncoder.getPosition() + m_RightEncoder.getPosition()) / 2;

  }

  public CANEncoder getLeftEncoder(){
    return m_LeftEncoder;
  }

  public CANEncoder getRightEncoder(){
    return m_RightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    m_Drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading(){
    imu.reset();

  }
  public double getHeading(){
    return imu.getAngle();
  }

  public double getTurnRate(){
    return imu.getRate();
  }



 

}
