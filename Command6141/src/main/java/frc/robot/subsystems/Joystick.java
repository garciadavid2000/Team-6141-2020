/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Joystick extends SubsystemBase {
  /**
   * 
   * Creates a new Joystick.
   */

   private Joystick joystick = new Joystick();
  public Joystick() {

  }

  public Joystick returnJoystick(){
    return joystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
