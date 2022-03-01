// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Lemonlight;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  
  // private final XboxController m_controller = new XboxController(0);
  private final Joystick m_controller = new Joystick(0);
  private final DriveTrain m_drive = new DriveTrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  static boolean turnButtonPressed = false;
  static boolean driveButtonPressed = false;
  
  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.


    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var xSpeed = -m_speedLimiter.calculate(m_controller.getY()) * 0;
    var rot = -m_rotLimiter.calculate(m_controller.getZ()) * 0;
      
    if ((Math.abs(m_controller.getY()) > 0.2)){
      xSpeed = -m_speedLimiter.calculate(m_controller.getY()) * DrivetrainConstants.kMaxSpeed;
     }
    else{}
    if ((Math.abs(m_controller.getZ()) > 0.2)){
      rot = -m_rotLimiter.calculate(m_controller.getZ()) * DrivetrainConstants.kMaxAngularSpeed;
  }

  if (m_controller.getRawButtonPressed(3)){
    Robot.turnButtonPressed = true;
  }
  else{}

  if(Robot.turnButtonPressed){
    m_drive.gotoAngle(90);
  }
  else{
  }
  }
}
