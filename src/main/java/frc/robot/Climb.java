package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climb {
    public WPI_TalonSRX left;
    public WPI_TalonSRX right;
    public WPI_TalonSRX angle;
    public WPI_TalonSRX extension;

    public Climb(int leftIndex, int rightIndex, int angleIndex, int extensionIndex) {
        right = new WPI_TalonSRX(rightIndex);
        left = new WPI_TalonSRX(leftIndex);
        angle = new WPI_TalonSRX(angleIndex);
        extension = new WPI_TalonSRX(extensionIndex);



        
    }
    
    public void Init() {
        
        right.configFactoryDefault();
        left.configFactoryDefault();

        angle.configFactoryDefault();
        extension.configFactoryDefault();
        
        right.setNeutralMode(NeutralMode.Brake);
        left.setNeutralMode(NeutralMode.Brake);
        angle.setNeutralMode(NeutralMode.Brake);
        extension.setNeutralMode(NeutralMode.Brake);
        left.setInverted(true);
        left.follow(right);
    }

    public void Set(double speed) {
        right.set(speed);
    }

    public void setAngle(double speed){
        angle.set(speed);
    }
    
    public void setExtension(double speed){
        extension.set(speed);
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
            case angleUp:
                setAngle(-.4);
                break;
            case angleDown:
                setAngle(.4);
                break;
            case extensionUp:
                setAngle(-.4);
                break;
            case extensionDown:
                setAngle(.4);
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
    doNothing,
    angleUp,
    angleDown,
    extensionUp,
    extensionDown
}
