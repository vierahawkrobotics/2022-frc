/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class constantVelSpin {
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private boolean invert;
  Lemonlight ElisLemons = new Lemonlight();
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.069633,0.067054,0.0060954);

  public constantVelSpin(CANSparkMax curMotor, boolean curInvert) {
    m_motor = curMotor;
    invert = curInvert;
  }

  /**
   * 
   * @return the angular velocity necessary
   */
  public double getAngularV() {
    double linearV = ElisLemons.getVelocity();
    double angularV = 0;
    double radius = 2; // inches
    angularV = linearV / radius; // rad/sec
    return angularV;
  }

  /**
   * 
   * 
   * @return returns an RPM by converting Angular Velocity to it
   */
  public double VtoRPM() {
    double RPM = 0;
    double angularV = getAngularV();
    RPM = (60 * angularV) / (2 * Math.PI);
    System.out.println("DOGE: " + RPM);
    return RPM;
  }

  //

  /**
   * Puts PID coefficients on SmartDashboard
   */
  public void motorInit() {
    // initialize motor
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();
    ElisLemons.initTheLemon();
    // PID coefficients
    kP = 1.2292E-05; // 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    

    // set PID coefficients7
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  /**
   * Uses PID to make sure motors are spinning at the same speed
   * The function also inverts the motor based on the constructor
   * Gets PID from SmartDashboard
   * Also sets speeds for the motor to spin at using velocity translation to RPM
   */
  public void shoot(boolean shootButton) {
    if (shootButton) {

      System.out.println("Auto SHooting is Wonwerbglink");
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to
      // controller
      if ((p != kP)) {
        m_pidController.setP(p);
        kP = p;
      }
      if ((i != kI)) {
        m_pidController.setI(i);
        kI = i;
      }
      if ((d != kD)) {
        m_pidController.setD(d);
        kD = d;
      }
      if ((iz != kIz)) {
        m_pidController.setIZone(iz);
        kIz = iz;
      }
      if ((ff != kFF)) {
        m_pidController.setFF(ff);
        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        m_pidController.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
      }

      // double setPoint = VtoRPM();
      double setPoint = VtoRPM() / 5700;
      if (invert) {
        setPoint = -setPoint;
      }
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      double feed = m_feedforward.calculate(setPoint);
      // m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity, 0, feed);

      // this is just to make stuff spin

      m_motor.set(setPoint + feed);
      // m_motor.set(setPoint);
      System.out.println("RPM: "+ setPoint);

      // this is just to integrate limelight with it
      // m_motor.set(VtoRPM());

      // System.out.println(maxRPM);
      // System.out.println(m_stick.getRawAxis(1));
      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());

    } 
    else {
      // System.out.println("NO SHOOT NO SHOOT");
      m_motor.stopMotor();
    }
  }

  /**
   * 
   * @return encoder values
   */
  public double getEncoder() {
    return m_encoder.getVelocity();
  }
}
