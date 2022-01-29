package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//using cim motors (dont know how to use in code)
//using three sets of arms (havent implemented, only one arm that doesn't have final port ids)
//COMPLETELY UNTESTED, probably works?
public class Climb {

    Joystick joystick = new Joystick(0);
    public ElevatorArm arm;
    /**stops the elevators when false */
    boolean canInteract;

    public Climb() {
        arm = new ElevatorArm(1, 3);//set ints for roborio ports, not set properly
    }

    /**
     * used in robot periodic, takes a predefined axis from a predefined joystick and uses that as a percent on how far the elevator extends.
     * requires "canInteract" to be true
     */
    public void ClimbTestIter() {

        double a = joystick.getRawAxis(3);
        arm.Set(a);
    }

    // State change
    // Execute State
    // Safety Stop
    // Reverse
    //

    // Can motors
}

class ElevatorArm {

    static double speedMult = 0.4;
    static double deadZone = 0.1;
    static double maxHeightPercent = 1;

    WPI_TalonSRX left;
    WPI_TalonSRX right;
    PIDController pidController;
    /**
     * 
     * @param rightIndex the right arm index for can
     * @param leftIndex the left arm index for can
     */
    public ElevatorArm(int rightIndex, int leftIndex) {
        //initiate pid
        pidController = new PIDController(0.03, 0.01, 0);
        pidController.setIntegratorRange(0, maxHeightPercent);
        pidController.setTolerance(0.02);
        
        //initiate motors
        //left = new WPI_TalonSRX(leftIndex);
        right = new WPI_TalonSRX(rightIndex);
        right.configFactoryDefault();
        //left.configFactoryDefault();

        //settings
        
        //config follower
        //left.setInverted(true);
        //left.follow(right);

        //right.getSelectedSensorPosition()
    }

    /**
     * 
     * @param target 0 to 1, percentage for how much the elevator moves;
     * @return if at target (returns controller.atSetpoint)
     */
    public boolean Set(double target) {
        target = MathUtil.clamp(target, -1, 1);
        //gets controller output (assuming input for motor)
        double speed = pidController.calculate(GetPos(), target);
        //altering the speed
        speed = MathUtil.clamp(speed*100, -1, 1);
        speed *= speedMult;
        System.out.println("speed:" + speed);
        //setting the speed
        right.set(speed);
        
        //returns if the controller is at the positions
        return pidController.atSetpoint();
    }

    public double GetPos() {
        double v = -right.getSelectedSensorPosition();
        // There are 360 UNITS per encoder rotation

        //https://www.andymark.com/products/climber-in-a-box
        //.79 in winch radius, 24.5 in extendable height
        //4.93581426833 rotations for elevator
        //16:1 gearbox ratio for cim motor
        // this is math from a person that can barely spell and count

        double perUnit = 1;//start value
        perUnit *= 360;//amount of units per rotation
        perUnit *= 1;//amount of rotations to raise one stage
        //perPulse /= 16;//if encoder is on motor, NOT AFTER motor, compensate for gearbox ratio

        //put info in calculator, the double should be percise enough for maximum percision. (perPulse calculation:11 decimals, double's max decimal:16)
        return v / perUnit;
    }

    public void Stop() {
        right.stopMotor();
        //left.stopMotor();
    }
}