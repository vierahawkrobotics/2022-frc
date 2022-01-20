package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//using cim motors (dont know how to use in code)
//using three sets of arms (havent implemented, only one arm that doesn't have final port ids)
//COMPLETELY UNTESTED, probably works?
public class Climb {

    Joystick joystick = new Joystick(0);
    ElevatorArm arm;
    /**stops the elevators when false */
    boolean canInteract;

    public Climb() {
        arm = new ElevatorArm(0, 1, 0, 1);//set ints for roborio ports, not set properly
    }

    /**
     * used in robot periodic, takes a predefined axis from a predefined joystick and uses that as a percent on how far the elevator extends.
     * requires "canInteract" to be true
     */
    public void ClimbTestIter() {
        if(joystick.getRawButton(0) || !canInteract) {
            arm.Stop();
            return;
        }

        double a = joystick.getRawAxis(0);
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

    static double speedMult = 0.3;
    static double deadZone = 0.1;
    static double maxHeightPercent = 1;

    WPI_TalonSRX left;
    WPI_TalonSRX right;
    PIDController controller;
    Encoder encoder;

    /**
     * 
     * @param leftIndex the left arm index for can
     * @param rightIndex the right arm index for can
     * @param encoderA the encoder port A
     * @param encoderB the encoder port B
     */
    public ElevatorArm(int leftIndex, int rightIndex, int encoderA, int encoderB) {
        //initiate pid
        controller = new PIDController(0.03, 0, 0);
        controller.setIntegratorRange(0, maxHeightPercent);
        controller.setTolerance(0.02);

        //initiate encoder
        encoder = new Encoder(encoderA, encoderB);
        
        // There are 256 pulses per encoder rotation --https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#driving-to-a-distance
        //1 unit per 1 rotation

        //https://www.andymark.com/products/climber-in-a-box
        //.79 in winch radius, 24.5 in extendable height
        //4.93581426833 rotations for elevator
        //16:1 gearbox ratio for cim motor
        // this is math from a person that can barely spell and count

        double perPulse = 1;//start value
        perPulse /= 256;//256 pulses per rotation
        perPulse /= 4.93581426833;//amount of rotations to raise one stage
        perPulse /= 16;//if encoder is on motor, NOT AFTER motor, compensate for gearbox ratio

        //put info in calculator, the double should be percise enough for maximum percision. (perPulse calculation:11 decimals, double's max decimal:16)

        encoder.setDistancePerPulse(perPulse);//encoder after gearbox
        encoder.setMaxPeriod(.1);
        encoder.setMinRate(10);
        encoder.setReverseDirection(false);
        encoder.setSamplesToAverage(5);

        //initiate motors
        left = new WPI_TalonSRX(leftIndex);
        right = new WPI_TalonSRX(rightIndex);
        right.configFactoryDefault();
        left.configFactoryDefault();

        //settings
        right.configNeutralDeadband(deadZone);
        
        //config follower
        left.setInverted(true);
        left.follow(right);
    }

    /**
     * 
     * @param target 0 to 1, percentage for how much the elevator moves;
     * @return if at target (returns controller.atSetpoint)
     */
    public boolean Set(double target) {
        target = MathUtil.clamp(target, 0, maxHeightPercent);
        //gets controller output (assuming input for motor)
        double speed = controller.calculate(encoder.getDistance(), target);
        //altering the speed
        speed = MathUtil.clamp(speed, -1, 1);
        speed *= speedMult;
        //setting the speed
        right.set(speed);
        
        //returns if the controller is at the positions
        return controller.atSetpoint();
    }

    public void Stop() {
        right.stopMotor();
        left.stopMotor();
    }
}