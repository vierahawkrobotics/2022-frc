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


public class DriveTrain {
    private final DifferentialDrive robotDrive;

    private WPI_TalonSRX leftDriveMotor1;
    private WPI_TalonSRX leftFollower;
    private WPI_TalonSRX rightDriveMotor1;
    private WPI_TalonSRX rightFollower;

    private double driveP;
    private double driveI;
    private double driveD;
    private double driveToleranceDegrees;

    private double turnP;
    private double turnI;
    private double turnD;
    private double turnTolerence;


    private PIDController turnController;
    private AHRS ahrs;

    private double driveSetPoint;

    private Joystick joystick;

    

    public DriveTrain(Joystick joystick){

        this.joystick = joystick;

        this.leftDriveMotor1  = new WPI_TalonSRX(DriveConstants.leftDriveMotorCanID);
        this.rightDriveMotor1 = new WPI_TalonSRX(DriveConstants.rightDriveMotorCanID);
        this.leftFollower  = new WPI_TalonSRX(DriveConstants.leftfollowerMotorCanID);
        this.rightFollower = new WPI_TalonSRX(DriveConstants.rightfollowerMotorCanID);

        this.driveP = DriveConstants.driveP;
        this.driveI = DriveConstants.driveI;
        this.driveD = DriveConstants.driveD;
        
        this.turnP = DriveConstants.turnP;
        this.turnI = DriveConstants.turnI;
        this.turnD = DriveConstants.turnD;
        this.turnTolerence = DriveConstants.turnTolerence;


       this.robotDrive = new DifferentialDrive(this.leftDriveMotor1, this.rightDriveMotor1);
       this.turnController = turnController;
       this.ahrs = ahrs;
       this.driveSetPoint = 0;
  }

  public void DriveTrainInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightDriveMotor1.setInverted(true);
    leftDriveMotor1.setInverted(false);

    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);

    rightFollower.follow(rightDriveMotor1);
    leftFollower.follow(leftFollower);


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

  driveSetPoint = 0;

}

    public void DriveTrainTeleop(){
            // Drive with arcade drive.
            // That means that the Y axis drives forward
            // and backward, and the X turns left and right.
        
        
            double driveZ = joystick.getZ();
        
            double rotateToAngleRate = turnController.calculate(ahrs.getYaw(), driveSetPoint);
            // double forward = Math.pow(100, joystick.getY() - 1) - 0.01; // Sign this so forward is positive
            double forward = joystick.getY(); // Sign this so forward is positive
        
            // double turn = -0.5 * joystick.getZ();
        
            System.out.println("Z: " + joystick.getZ());
            System.out.println("Setpoint: " + driveSetPoint);


            if (driveZ < -0.2 || driveZ > 0.2){
                driveSetPoint+=driveZ*0.4; //this is 20 degrees per second
            }
        
            robotDrive.arcadeDrive(forward, rotateToAngleRate);
        }
    

    public void goToAngle(double rotateSetPoint){
        //give it an angle and it will turn

        double forward = 0;
        double rotateToAngleRate = turnController.calculate(ahrs.getYaw(), rotateSetPoint);
        robotDrive.arcadeDrive(forward, rotateToAngleRate);
    }

    public double getCurrentAngle(){
        return ahrs.getYaw();
    }

}