package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climb {
    WPI_TalonSRX staticLeft;
    WPI_TalonSRX staticRight;
    double currentDistance;
    Encoder encoder;

    Joystick joystick = new Joystick(0);

    public Climb(int joystickPort, int staticLeftPort, int staticRightPort, int encoderA, int encoderB) {
        staticLeft = new WPI_TalonSRX(staticLeftPort);
        staticRight = new WPI_TalonSRX(staticRightPort);
        joystick = new Joystick(joystickPort);
        encoder = new Encoder(encoderA, encoderB);
    }

    public void ClimbInit() {
        staticRight.configFactoryDefault();
        staticLeft.configFactoryDefault();
        staticRight.configNeutralDeadband(0.1);
        staticLeft.configFactoryDefault();
        staticLeft.setInverted(true);
        staticLeft.follow(staticRight);

        //256 pulses per rotation
        encoder.setDistancePerPulse(4./256.);
        encoder.setMaxPeriod(.1);
        encoder.setMinRate(10);
        encoder.setReverseDirection(false);
        encoder.setSamplesToAverage(5);
    }

    public void ClimbTestIter() {
        double a = joystick.getRawAxis(0);
        SetDistance(a, staticRight);
    }

    //DO NOT USE-- USE PID CONTROLLER INSTEAD
    void SetDistance(double distance, WPI_TalonSRX motor) {
        double speed = (currentDistance - distance);
        speed = Math.max(Math.min(speed*10, 1),-1);
        
        motor.set(speed);

        double dis = encoder.getDistance();
        currentDistance += dis;
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
