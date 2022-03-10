package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climb {
    public WPI_TalonSRX left;
    public WPI_TalonSRX right;

    public Climb(int leftIndex, int rightIndex) {
        right = new WPI_TalonSRX(rightIndex);
        left = new WPI_TalonSRX(leftIndex);
    }

    public void Init() {
        right.configFactoryDefault();
        left.configFactoryDefault();

        left.setInverted(true);
        left.follow(right);
    }

    public void Set(double speed) {
        right.set(speed);
    }

    public void Stop() {
        right.stopMotor();
    }

    public void Teleop(boolean up, boolean down, boolean smallDown) {
        if(up) InterpMode(ClimberMode.Up);
        else if(down) InterpMode(ClimberMode.Down);
        else if(smallDown) InterpMode(ClimberMode.SmallDown);
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
            case doNothing:
                Set(0);
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
    doNothing
}
