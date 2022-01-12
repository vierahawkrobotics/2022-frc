// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftDriveMotor1, rightDriveMotor1);
  private final Joystick joystick = new Joystick(0);
  public static Talon leftDriveMotor1 = new Talon(0);
  public static Talon leftFollower = new Talon(1);
  public static Talon rightDriveMotor1 = new Talon(2);
  public static Talon rightFollower = new Talon(3);
  
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightDriveMotor1.setInverted(true);
    leftDriveMotor1.setInverted(false);
    rightFollower.setInverted(true);
    leftFollower.setInverted(false);


  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-joystick.getY(), joystick.getX());
  }
}
