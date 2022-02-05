package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

//using cim motors (dont know how to use in code)
//using three sets of arms (havent implemented, only one arm that doesn't have final port ids)
//COMPLETELY UNTESTED, probably works?
public class Climb {

    Joystick joystick = new Joystick(0);
    public ElevatorArm arm;
    /**stops the elevators when false */
    boolean canInteract;

    public Climb() {
        arm = new ElevatorArm(1, -1);//set ints for roborio ports, not set properly
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