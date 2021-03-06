/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  Gamepad gamepad = new Gamepad(0);

  @Override
  public void robotInit() {
    RobotGyro.reset();
    DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions
  }

  @Override
  public void teleopInit() {
    DriveTrain.setAllTurnOrientation(0, false); // sets them back to calibrated zero position
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (gamepad.getButtonStateStart()) {
      System.out.println("Start pressed - reset");
      RobotGyro.reset();
      DriveTrain.resetTurnEncoders(); // sets encoders based on absolute encoder positions
      DriveTrain.setAllTurnOrientation(0);
    }

    double driveYAxisAmount = -gamepad.getLeftY();
    double driveStrafeAxisAmount = -gamepad.getLeftX();
    double driveRotAxisAmount = gamepad.getRightX();

    DriveTrain.fieldCentricDrive(driveYAxisAmount, driveStrafeAxisAmount, driveRotAxisAmount);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
