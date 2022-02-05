// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Lemonlight;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  Joystick joystick = new Joystick(0);
  DriveTrain driveTrain = new DriveTrain(joystick);
  private final Lemonlight JoshsLemon = new Lemonlight();

  @Override
  public void robotInit() {
    driveTrain.DriveTrainInit();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.DriveTrainTeleop();
    // driveTrain.goToAngle(25);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // JoshsLemon.LemonLight();
  
  }

} 
