// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
  
  
  private final XboxController m_xbox = new XboxController(1);
  //private final Joystick m_xbox = new Joystick(1);

  private final Joystick m_controller = new Joystick(0);
  private final DriveTrain m_drive = new DriveTrain();
  public Climb climb = new Climb(9,10);
  private final Shooter shoot = new Shooter( m_drive, climb);  

  private Autonomous autonomous = new Autonomous();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  static boolean turnButtonPressed = false;
  static boolean driveButtonPressed = false;

  double xSpeed;
  double rot;


  @Override
  public void robotInit() {
    shoot.shooterInit();
    climb.Init();
    xSpeed = 0;
    rot = 0;
  }

  @Override
  public void autonomousInit(){
    autonomous.count = true;
    autonomous.autoState = AutoState.doNothing;
  }

  @Override
  public void autonomousPeriodic() {
    autonomous.autoPeriodic(m_drive, shoot, climb);
    
  }

  @Override
  public void teleopPeriodic() {
    //REMOVE THIS TESTING ONLY
    System.out.println(shoot.putDash());
    // driveTrain.goToAngle(25);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // JoshsLemon.LemonLight();
    
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
    } 
    else if (m_xbox.getRawButton(5)) {
      m_drive.gotoAngle(-Lemonlight.getHorizontalOffset()*2);
    }
    else {
      m_drive.drive(xSpeed, rot);
    }
    shoot.shooterTeleop(m_controller.getRawButton(1), m_controller.getRawButton(2), m_xbox.getRawButton(3), m_xbox.getRawButton(2), m_xbox.getRawButton(5));
    //shoot.shootAutomation(m_controller.getRawButton(1), m_xbox.getRawButton(4), m_xbox.getRawButton(3));
    climb.Teleop(m_xbox.getRawButton(4), m_xbox.getRawButton(1), m_xbox.getRawButton(6));

    shoot.Idle(m_xbox.getRawButton(5), m_controller.getRawButton(1),  false, m_xbox.getRawButton(3),m_xbox.getRawButton(2));
  }
}