/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivesubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  private Drivesubsystem drive = new Drivesubsystem();
  private Shooter shooter = new Shooter();
  private Limelight limelight = new Limelight();
  // The robot's subsystems and commands are defined here...
  
  Joystick joystick = new Joystick(1);
  XboxController xboxController = new XboxController(0);
  JoystickButton triggerButton = new JoystickButton(joystick, 1);
  JoystickButton thumbButton = new JoystickButton(joystick, 2);
  JoystickButton topLeft = new JoystickButton(joystick, 3);
  JoystickButton topRight = new JoystickButton(joystick, 4);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(
      Constants.ksVolts,
      Constants.kvVoltsSecondsPerMeter,
      Constants.kaVoltsSecondSquaredPerMeter), Constants.kDriveKinematics, 10);

      //Create Trajectory Config
      TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAccel)
      .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);


       
      
      
      
        
     String trenchTrajectoryJSON = "path.ThroughTrench.wpilib.json";
     try{ 
       Path trenchPath = Filesystem.getDeployDirectory().toPath().resolve(trenchTrajectoryJSON);
      Trajectory trenchTrajectory = TrajectoryUtil.fromPathweaverJson(trenchPath);
    RamseteCommand ramseteCommand = new RamseteCommand(trenchTrajectory, drive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter),
      Constants.kDriveKinematics,
      drive::getWheelSpeeds,
      new PIDController(Constants.kP_drive, 0, 0),
      new PIDController(Constants.kP_drive, 0, 0),  
      drive::tankDriveVolts,
      drive);
  return ramseteCommand.andThen(() ->  drive.tankDriveVolts(0, 0));
     } catch (IOException ex){
       DriverStation.reportError("Unable to open: " + trenchTrajectoryJSON, ex.getStackTrace());
     }

     
     String trajectoryJSON2 = "path/ToShoot.wpilib.json";
    try{ 
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
      Trajectory trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    RamseteCommand ramseteCommand_2 = new RamseteCommand(trajectory2, drive::getPose,
  new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
  new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter),
  Constants.kDriveKinematics,
  drive::getWheelSpeeds,
  new PIDController(Constants.kP_drive, 0, 0),
  new PIDController(Constants.kP_drive, 0, 0),  
  drive::tankDriveVolts,
  drive);
    return ramseteCommand_2.andThen(() -> drive.tankDriveVolts(0, 0));
  } catch (IOException ex){
    DriverStation.reportError("Unable to open: " + trajectoryJSON2, ex.getStackTrace());

  }
    return null;
  
  
   
  
  }
  public void resetSensors(){
    
  }

  // private Trajectory getTrenchTrajectory() {
  //   String trajectoryJSON = "Path/ThroughTrench.wpilib.json";
  //   try{ 
  //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  //       Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //       return trajectory;
  //      } catch (IOException ex){
  //        DriverStation.reportError("Unable to open: " + trajectoryJSON, ex.getStackTrace());
  //        return null;
 
     

      
       
      //  private Trajectory getToShooterTrajectory(){
      //   String trajectoryJSON2 = "Path.ToShoot.wpilib.json";
      //   try {
      //     Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
      //     Trajectory trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
      //     return trajectory2;
      //   } catch (IOException ex){
      //     DriverStation.reportError("Unable to open: " + trajectoryJSON2, ex.getStackTrace());
      //   } return null;
       
  
//   public SequentialCommandGroup returnRamseteCommand2() {
//     String trajectoryJSON2 = "path/ToShoot.wpilib.json";
//     try{ 
//       Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
//       Trajectory trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
//     RamseteCommand ramseteCommand_2 = new RamseteCommand(trajectory2, drive::getPose,
//   new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
//   new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltsSecondsPerMeter),
//   Constants.kDriveKinematics,
//   drive::getWheelSpeeds,
//   new PIDController(Constants.kP_drive, 0, 0),
//   new PIDController(Constants.kP_drive, 0, 0),  
//   drive::tankDriveVolts,
//   drive);
//     return ramseteCommand_2.andThen(() -> drive.tankDriveVolts(0, 0));
//   } catch (IOException ex){
//     DriverStation.reportError("Unable to open: " + trajectoryJSON2, ex.getStackTrace());

//   }
  
// }
}
  
