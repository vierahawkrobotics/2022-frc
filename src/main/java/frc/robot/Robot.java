// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Lemonlight;
=======
>>>>>>> testing
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Lemonlight JoshsLemon = new Lemonlight();
  private final Autonomous auto = new Autonomous();

  private XboxController m_stick = new XboxController(0);
  private CANSparkMax leftSpinnyBoi = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightSpinnyBoi = new CANSparkMax(2, MotorType.kBrushless);
  WPI_TalonSRX front = new WPI_TalonSRX(7);
  WPI_TalonSRX back = new WPI_TalonSRX(8);
  private constantVelSpin leftConstantVel = new constantVelSpin(leftSpinnyBoi, m_stick, false);
  private constantVelSpin rightConstantVel = new constantVelSpin(rightSpinnyBoi, m_stick, true);


  private final XboxController m_xbox = new XboxController(1);
  private final Joystick m_controller = new Joystick(0);
  private final DriveTrain m_drive = new DriveTrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  static boolean turnButtonPressed = false;
  static boolean driveButtonPressed = false;

  double xSpeed;
  double rot;

  @Override
  public void robotInit() {
    leftConstantVel.motorInit();
    rightConstantVel.motorInit();
    xSpeed = 0;
    rot = 0;
  }

  @Override
  public void teleopPeriodic() {
    //REMOVE THIS TESTING ONLY
    leftSpinnyBoi.set(0);
    rightSpinnyBoi.set(0);
    front.set(0);
    back.set(0);
    // driveTrain.goToAngle(25);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // JoshsLemon.LemonLight();
    if (m_xbox.getRawButton(1)) {
      leftConstantVel.motorTeleop();
      rightConstantVel.motorTeleop();
    } else {
      leftConstantVel.stop();
      rightConstantVel.stop();
    }

    if (m_xbox.getRawButton(2)) {
      front.set(.9);
      back.set(.9);
    } else if (m_xbox.getRawButton(3)) {
      front.set(-.9);
      back.set(-.9);
    } else {
      front.set(0);
      back.set(0);
    }

    if (m_xbox.getRawButton(4)) {
      auto.seeking();
    }

    if (m_xbox.getRawButton(5)) {
      auto.Aiming();
    }

    if (m_xbox.getRawButton(6)) {
      auto.seeking();
      auto.Aiming();
    }

    if (m_xbox.getRawButton(7)) {
      auto.getInRange();
    }

    if (m_xbox.getRawButton(8)) {
      leftConstantVel.manualShooter(2000);
      rightConstantVel.manualShooter(-2000);
    } else {
      leftConstantVel.stop();
      rightConstantVel.stop();
    }
    // System.out.print(leftConstantVel.getEncoder());
    // System.out.print(rightConstantVel.getEncoder());
    System.out.println(JoshsLemon.distanceGrab());
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // var xSpeed = -m_speedLimiter.calculate(m_controller.getY()) * 0;
    // var rot = -m_rotLimiter.calculate(m_controller.getZ()) * 0;
    System.out.println("Teleop");
    if ((Math.abs(m_controller.getY()) > 0.2)) {
      xSpeed = -m_speedLimiter.calculate(m_controller.getY()) * DrivetrainConstants.kMaxSpeed;
    } else {
      xSpeed = 0;
    }
    if ((Math.abs(m_controller.getZ()) > 0.2)) {
      rot = -m_rotLimiter.calculate(m_controller.getZ()) * DrivetrainConstants.kMaxAngularSpeed;
    }
    else {
      rot = 0;
    }

    if (m_controller.getRawButtonPressed(3)) {
      Robot.turnButtonPressed = true;
    } else {
    }

    if (Robot.turnButtonPressed) {
      m_drive.gotoAngle(90);
    } else {
      m_drive.drive(xSpeed, rot);
    }
  }
}