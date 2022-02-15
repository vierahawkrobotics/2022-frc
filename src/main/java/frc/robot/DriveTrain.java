/**
 * @author Pranav Ponnusamy
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/** Represents a **differential** drive style drivetrain. */
public class DriveTrain {
  /**
   * Sets the max speed(m/s) of the robot
   */
  public static final double kMaxSpeed = 3.0; // meters per second  
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  /**Sets the distance between the two wheels */
  private static final double kTrackWidth = 0.5799666582; // meters 22.833333 inches 
  private static final double kWheelRadius = 0.0762; // meters 3 inches
  private static final int kEncoderResolution = 2048; 

  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(3);


  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

  // private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final PIDController m_leftPIDController = new PIDController(0, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(0, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  // private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */


  public DriveTrain() {
    ahrs.zeroYaw();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.

    // m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());

    m_leftLeader.configClearPositionOnQuadIdx(true, 0);
  }

  /**
   * 
   * Gets the speed in m/s of the right side of the robot
   */
  public double getLeftRate(){
    // number of ticks per 100 ms -> m/s
    return m_leftLeader.getSelectedSensorVelocity() * (2 * Math.PI * kWheelRadius / kEncoderResolution);
  }

  /**
   * 
   * Gets the speed in m/s of the right side of the robot
   */
  public double getrightRate(){
    // number of ticks per 100 ms -> m/s
    return m_rightLeader.getSelectedSensorVelocity() * (2 * Math.PI * kWheelRadius / kEncoderResolution);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(getLeftRate(), speeds.leftMetersPerSecond);
    final double rightOutput =  
        m_rightPIDController.calculate(getrightRate(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  // public void updateOdometry() {
  //   m_odometry.update(
  //       ahrs.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  // }
}
