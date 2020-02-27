/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

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
  
  private final double desiredDistance1 = Units.feetToMeters(10) * 100;
  private final double desiredDistance2 = Units.feetToMeters(7) * 100;
  private final double desiredDistance3 = 200;


  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  public double getLimelightSteerCommand() {
    return LimelightSteerCommand;
  }

  public void setLimelightSteerCommand(double limelightSteerCommand) {
    this.LimelightSteerCommand = limelightSteerCommand;
  }

  public double getLimelightDriveCommand() {
    return LimelightDriveCommand;
  }

  public void setLimelightDriveCommand(double limelightDriveCommand) {
    this.LimelightDriveCommand = limelightDriveCommand;
  }

  public boolean isLimelightHasValidTarget() {
    return LimelightHasValidTarget;
  }

  public void setLimelightHasValidTarget(boolean limelightHasValidTarget) {
    this.LimelightHasValidTarget = limelightHasValidTarget;
  }

  public NetworkTableEntry getCamMode() {
    return camMode;
  }

  public void setCamMode(NetworkTableEntry camMode) {
    this.camMode = camMode;
  }

  public NetworkTableEntry getLedMode() {
    return ledMode;
  }

  public void setLedMode(NetworkTableEntry ledMode) {
    this.ledMode = ledMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      setLimelightHasValidTarget(false);
      setLimelightDriveCommand(0.0);
      setLimelightSteerCommand(0.0);
      return;
    }

    setLimelightHasValidTarget(true);

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    setLimelightSteerCommand(steer_cmd);

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (desiredDistance1 - distance) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    } else if (drive_cmd < -MAX_DRIVE) {
      drive_cmd = -MAX_DRIVE;
    }
    setLimelightDriveCommand(drive_cmd);
  }

  public double estimateDistance() {
    double ty = limelight.getEntry("ty").getDouble(0);

    return (h2 - h1) / (Math.tan(Math.toRadians(a1 + ty)));
  }

  
  
}
