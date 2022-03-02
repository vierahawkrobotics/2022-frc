package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * @deprecated
 */
public class Climb {

    /**The animation controller for the climber mechanism */
    ElevatorArmAnimator anim;
    /**boolean that stops the animator from continueing, automatically sets to false when the animator is finished.*/
    public boolean cont;
    /**
     * Initiates new climber preset to an animation
     */
    public Climb() {
        anim = new ElevatorArmAnimator(new ElevatorArm(1,0,1), new ElevatorArm(), 1);
        anim.itters = new EAItter[] {
            new EAItter(1, 0, 1, 1),
            new EAItter(0, 1, 1, 1),
            new EAItter(0.5, 0.5, 1, 1),
            new EAItter(0.25, 0.5, 1, 1),
            new EAItter(0.8, 0.5, 1, 1),
        };
    }
    /**resets the animator to the start. Required to function*/
    public void Start() {
        anim.Start();
    }

    /**itterates the animator to rotate motors. call this every robot itteration*/
    public void Itterate() {
        if(cont) {
            cont = anim.Update();                
        }
    }

    // State change
    // Execute State
    // Safety Stop
    // Reverse
    //

    // Can motors
}

class ElevatorArm {

    WPI_TalonSRX right;
    PIDController pidController;
    public double maxSpeed = 1;
    DigitalInput min;
    DigitalInput max;
    boolean exsists;

    /**initiates an arm for testing, will not spin any motors */
    public ElevatorArm() {
        exsists = false;
    }

    /**
     * initiates motors for arm
     * @param index the right arm index for can ID
     * @param digitalMin ID for min sensor
     * @param digitalMax ID for max sensor
     */
    public ElevatorArm(int index, int digitalMin, int digitalMax) {
        exsists = true;
        
        //initiate motors
        right = new WPI_TalonSRX(index);
        right.configFactoryDefault();

        min = new DigitalInput(digitalMin);
        max = new DigitalInput(digitalMax);
    }

    /**
     * sets the motors at a specified position
     * @param target 0 to 1, percentage for how much the elevator moves;
     * @return true if at target (returns controller.atSetpoint)
     * @deprecated
     */
    public boolean SetPos(double target) {
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
    
    /**sets the motor speed
     * @param speed speed to be set to motor
    */
    public void SetSpeed(double speed) {
        if(min.get() && speed < 0) speed = 0;
        if(max.get() && speed > 0) speed = 0;

        right.set(speed*maxSpeed);
    }

    /**using encoders to get the rotation (0 to 1) for how far the motor spun
     * @deprecated
     */
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

    /**calls the motors to stops*/
    public void Stop() {
        right.stopMotor();
    }
}