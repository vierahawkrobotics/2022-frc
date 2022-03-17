package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climb {
    public WPI_TalonSRX left;
    public WPI_TalonSRX right;
    public WPI_TalonSRX secondClimb;
    public WPI_TalonSRX angleAdjust;

    public Climb(int leftIndex, int rightIndex, int secondClimbIndex, int angleAdjustIndex) {
        right = new WPI_TalonSRX(rightIndex);
        left = new WPI_TalonSRX(leftIndex);

        secondClimb = new WPI_TalonSRX(secondClimbIndex);
        angleAdjust = new WPI_TalonSRX(angleAdjustIndex);
    }
    
    public void Init() {
        
        right.configFactoryDefault();
        left.configFactoryDefault();
        secondClimb.configFactoryDefault();
        
        right.setNeutralMode(NeutralMode.Brake);
        left.setNeutralMode(NeutralMode.Brake);
        secondClimb.setNeutralMode(NeutralMode.Brake);
        angleAdjust.setNeutralMode(NeutralMode.Brake);

        left.setInverted(true);
        left.follow(right);
    }

    public void Set(WPI_TalonSRX motor,double speed) {
        right.set(speed);
    }

    public void Stop() {
        right.stopMotor();
    }

    public void Teleop(boolean firstClimbUp, boolean firstClimbDown, boolean firstClimbSmallDown, boolean secondClimbUp,
     boolean secondClimbDown, boolean adjustAngleUp, boolean adjustAngleDown ) {
        if(firstClimbUp) InterpMode(ClimberMode.firstClimbUp);
        else if(firstClimbDown) InterpMode(ClimberMode.firstClimbDown);
        else if(firstClimbSmallDown) InterpMode(ClimberMode.firstClimbSmallDown);
        else if(secondClimbUp) InterpMode(ClimberMode.secondClimbUp);
        else if(secondClimbDown) InterpMode(ClimberMode.secondClimbDown);
        else if(adjustAngleUp) InterpMode(ClimberMode.adjustUp);
        else if(adjustAngleDown) InterpMode(ClimberMode.adjustDown);
        else InterpMode(ClimberMode.doNothing);
    }

    void InterpMode(ClimberMode mode) {
        switch(mode) {
            case firstClimbDown:
                Set(right,.4);
                break;
            case firstClimbSmallDown:
                Set(right,0.15);
                break;
            case firstClimbUp:
                Set(right,-.4);
                break;
            case secondClimbUp:
                Set(secondClimb,-.4);
                break;
            case secondClimbDown:
                Set(secondClimb,0.4);
                break;
            case adjustUp:
                Set(angleAdjust,0.4);
                break;
            case adjustDown:
                Set(angleAdjust,-0.4);
                break;
            case doNothing:
                Set(right,0);
                Set(secondClimb,0);
                Set(angleAdjust,0);
                break;
            default:
                mode = ClimberMode.doNothing;
                break;
        }
    }
}

enum ClimberMode {
    firstClimbUp,
    firstClimbDown,
    firstClimbSmallDown,
    secondClimbUp,
    secondClimbDown,
    adjustUp,
    adjustDown,
    doNothing
}
