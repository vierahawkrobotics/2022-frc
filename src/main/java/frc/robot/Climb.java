package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.*;

public class Climb {
    public PWMSparkMax StaticArmRight;
    public PWMSparkMax StaticArmLeft;

    public PWMSparkMax RotateArmRight;
    public PWMSparkMax RotateArmLeft;

    public Climb() {

    }

    public void ClimbInit() {
        
    }

    public void ArmInputUpdate(double deltaStaticArm, double deltaRotateArm) {

    }

    //State change
    //Execute State
    //Safety Stop
    //Reverse
    //

    //Can motors
}

enum ClimbState {
    None,
    ExtendStatic
}
