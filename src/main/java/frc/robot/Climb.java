package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climb {
    public WPI_TalonSRX left;
    public WPI_TalonSRX right;
    public WPI_TalonSRX thirdUD;
    public WPI_TalonSRX thirdFB;

    public Climb(int leftIndex, int rightIndex, int thirdUDIndex, int thirdFBIndex) {
        right = new WPI_TalonSRX(rightIndex);
        left = new WPI_TalonSRX(leftIndex);
        //thirdUD is third arm up and down
        //thirdFB is third arm forward and backward
        thirdUD = new WPI_TalonSRX(thirdUDIndex);
        thirdFB = new WPI_TalonSRX(thirdFBIndex);
    }
    
    public void Init() {
        right.configFactoryDefault();
        left.configFactoryDefault();
        thirdFB.configFactoryDefault();
        thirdUD.configFactoryDefault();
        
        right.setNeutralMode(NeutralMode.Brake);
        left.setNeutralMode(NeutralMode.Brake);
        left.setInverted(true);
        left.follow(right);

        thirdFB.setNeutralMode(NeutralMode.Brake);
        thirdUD.setNeutralMode(NeutralMode.Brake);
    }

    public void Set(double speed) {
        right.set(speed);
    }

    public void SetThirdFB(double speed) {
        thirdFB.set(speed);
    }

    public void SetThirdUD(double speed) {
        thirdUD.set(speed);
    }

    public void Stop() {
        right.stopMotor();
        thirdFB.stopMotor();
        thirdUD.stopMotor();
    }

    public void Teleop(boolean up, boolean down, boolean smallDown, boolean thirdUp, boolean thirdDown, boolean thirdForward, boolean thirdBackward) {
        if(up) InterpMode(ClimberMode.Up);
        else if(down) InterpMode(ClimberMode.Down);
        else if(smallDown) InterpMode(ClimberMode.SmallDown);
        else if(thirdUp) InterpMode(ClimberMode.ThirdUp);
        else if(thirdDown) InterpMode(ClimberMode.ThirdDown);
        else if(thirdForward) InterpMode(ClimberMode.ThirdForward);
        else if(thirdBackward) InterpMode(ClimberMode.ThirdBackward);
        else InterpMode(ClimberMode.doNothing);
    }

    void InterpMode(ClimberMode mode) {
        switch(mode) {
            case Down:
                Set(.4);
                break;
            case SmallDown:
                Set(0.15);
                break;
            case Up:
                Set(-.4);
                break;
            case ThirdDown:
                SetThirdUD(.4);
                break;
            case ThirdUp:
                SetThirdUD(-.4);
                break;
            case ThirdForward:
                SetThirdFB(-.4);
                break;
            case ThirdBackward:
                SetThirdFB(.4);
                break;
            case doNothing:
                Set(0);
                SetThirdUD(0);
                SetThirdFB(0);
                break;
            default:
                break;
        }
    }
}

enum ClimberMode {
    Up,
    Down,
    SmallDown,
    ThirdUp,
    ThirdDown,
    ThirdForward,
    ThirdBackward,
    doNothing
}
