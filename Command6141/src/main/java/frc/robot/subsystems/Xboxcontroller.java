/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Xboxcontroller extends SubsystemBase {
  /**
   * Creates a new Xboxcontroller.
   */

   private XboxController xboxcontroller = new XboxController(0);
  public Xboxcontroller() {

  }

  public XboxController returnXboxcontroller(){
    return xboxcontroller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
