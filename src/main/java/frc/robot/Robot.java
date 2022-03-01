// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Lemonlight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Lemonlight JoshsLemon = new Lemonlight();

  private XboxController m_stick = new XboxController(0);
  private CANSparkMax leftSpinnyBoi = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightSpinnyBoi = new CANSparkMax(2, MotorType.kBrushless);
  WPI_TalonSRX front = new WPI_TalonSRX(7);
  WPI_TalonSRX back = new WPI_TalonSRX(8);
  private constantVelSpin leftConstantVel = new constantVelSpin(leftSpinnyBoi, m_stick, false);
  private constantVelSpin rightConstantVel = new constantVelSpin(rightSpinnyBoi, m_stick, true);

  WPI_TalonSRX leftleader = new WPI_TalonSRX(1);
  WPI_TalonSRX rightleader = new WPI_TalonSRX(3);
  DifferentialDrive drive = new DifferentialDrive(leftleader, rightleader);

  Joystick joystick = new Joystick(1);
  //DriveTrain driveTrain = new DriveTrain(joystick);
  
  @Override
  public void robotInit() {
    //driveTrain.DriveTrainInit();
    leftConstantVel.motorInit();
    rightConstantVel.motorInit();
    leftleader.setInverted(InvertType.InvertMotorOutput);
}

  @Override
  public void teleopPeriodic() {
    //driveTrain.DriveTrainTeleop();
    // driveTrain.goToAngle(25);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // JoshsLemon.LemonLight();
    if(joystick.getRawButton(1)){
      leftConstantVel.motorTeleop();
      rightConstantVel.motorTeleop();
    }else{
      leftConstantVel.stop();
      rightConstantVel.stop();
    }
    
    if(joystick.getRawButton(2)) {
      front.set(.9);
      back.set(.9);
    }
    else if (joystick.getRawButton(3)) {
      front.set(-.9);
      back.set(-.9);
    }
    else{
      front.set(0);
      back.set(0);
    }

    double driveY = joystick.getY();
    double driveZ = joystick.getZ();

    drive.arcadeDrive(driveY, driveZ);


    //System.out.print(leftConstantVel.getEncoder());
    //System.out.print(rightConstantVel.getEncoder());
    System.out.println(JoshsLemon.distanceGrab());

    
  }
} 
