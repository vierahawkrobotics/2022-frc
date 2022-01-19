// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.InvertType;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftDriveMotor1, rightDriveMotor1);
  private final Joystick joystick = new Joystick(0);

  public static WPI_TalonSRX leftDriveMotor1 = new WPI_TalonSRX(1);
  public static WPI_TalonSRX leftFollower = new WPI_TalonSRX(3);

  public static WPI_TalonSRX rightDriveMotor1 = new WPI_TalonSRX(2);
  public static WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private static final double driveP = 0.03;
  private static final double driveI = 0.00;
  private static final double driveD = 0.00;
  private static final double driveToleranceDegrees = 1.0f;
  

  PIDController turnController;
  AHRS ahrs;


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightDriveMotor1.setInverted(true);
    leftDriveMotor1.setInverted(false);

    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(false);


    try { // attempt to instantiate the NavX2. If it throws an exception, catch it and
      // report it.
  ahrs = new AHRS(SPI.Port.kMXP); // SPI is the protocol on the MXP connector that the navigator is plugged into
} catch (RuntimeException ex) {
  DriverStation.reportError("Error instantiating navX2 MXP:  " + ex.getMessage(), true);
}

  turnController = new PIDController(driveP, driveI, driveD);
  turnController.enableContinuousInput(-180.0f, 180.0f);
  turnController.setTolerance(driveToleranceDegrees);
  ahrs.zeroYaw();
  turnController.setSetpoint(ahrs.getYaw());

}

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    double rotateToAngleRate = turnController.calculate(ahrs.getYaw(), 0.0);
    double forward = 1.0 * joystick.getY(); // Sign this so forward is positive
    double turn = -0.5 * joystick.getZ();
    m_robotDrive.arcadeDrive(forward, turn + rotateToAngleRate);
  
  
  }
} 
