package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

class ElevatorArm {

    WPI_TalonSRX left;
    WPI_TalonSRX right;
    PIDController pidController;
    public double maxSpeed = 1;
    boolean exsists;

    public ElevatorArm() {
        exsists = false;
    }

    /**
     * 
     * @param rightIndex the right arm index for can
     * @param leftIndex the left arm index for can
     */
    public ElevatorArm(int rightIndex, int leftIndex) {
        exsists = true;
        //initiate pid
        pidController = new PIDController(0.015, 0.007, 0);
        pidController.setTolerance(0.02);
        
        //initiate motors
        right = new WPI_TalonSRX(rightIndex);
        right.configFactoryDefault();

        //settings
        
        //config follower
        left.setInverted(true);

        //right.getSelectedSensorPosition()

        if(leftIndex >= 0) {
            left = new WPI_TalonSRX(leftIndex);
            left.configFactoryDefault();
            left.follow(right);
        }
    }

    /**
     * 
     * @param target 0 to 1, percentage for how much the elevator moves;
     * @return if at target (returns controller.atSetpoint)
     */
    public boolean Set(double target) {
        if(!exsists) return false;

        target = MathUtil.clamp(target, -1, 1);
        //gets controller output (assuming input for motor)
        double speed = pidController.calculate(GetPos(), target);
        //altering the speed
        speed = MathUtil.clamp(speed*100, -1, 1);
        speed *= maxSpeed;
        System.out.println("speed:" + speed);
        //setting the speed
        right.set(speed);
        
        //returns if the controller is at the positions
        return pidController.atSetpoint();
    }

    public double GetPos() {
        if(!exsists) return 0;

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
        //perPulse *= 16;//if encoder is on motor, NOT AFTER motor, compensate for gearbox ratio

        //put info in calculator, the double should be percise enough for maximum percision. (perPulse calculation:11 decimals, double's max decimal:16)
        return v / perUnit;
    }

    public void Stop() {
        right.stopMotor();
    }
}