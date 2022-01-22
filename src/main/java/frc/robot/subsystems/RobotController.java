/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

// The best class ever made
public class RobotController extends XboxController {
  /**
   * Creates a new RobotController.
   */
  private static RobotController _instance = new RobotController(0);
  public RobotController(int port) {
    super(port);
  }

  public double getLeftXAxis() {
    return getRawAxis(0);
  }

  public double getLeftYAxis() {
    return getRawAxis(1);
  }

  public double getLeftTrigger() {
    return getRawAxis(2);
  }

  public double getRightTrigger() {
    return getRawAxis(3);
  }

  public double getRightXAxis() {
    return getRawAxis(4);
  }

  public double getRightYAxis() {
    return getRawAxis(5);
  }

  public static RobotController getInstance() {
    return _instance;
  }
}
