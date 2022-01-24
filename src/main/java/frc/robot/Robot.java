// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  Joystick joystick = new Joystick(0);
  DriveTrain driveTrain = new DriveTrain(joystick);
  
  @Override
  public void robotInit() {
    driveTrain.DriveTrainInit();
}

  @Override
  public void teleopPeriodic() {
    driveTrain.DriveTrainTeleop();
    // driveTrain.goToAngle(25);
  
  }
} 
