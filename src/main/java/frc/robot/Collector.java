package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

public class Collector {
    Joystick m_stick = new Joystick(0);
    ColorSensor sensor = new ColorSensor();
    private WPI_TalonSRX bottomFeeder = new WPI_TalonSRX(1);
    private WPI_TalonSRX topFeeder = new WPI_TalonSRX(2);
    double speed = .1;
    Servo servoArm;
    public ServoState servoState;
    /**
     * @return if the ball should move up the conveyor belt or not
     */
    public void moveUp(){
        String color = sensor.ColorToString();
        if(color == "Green"){
            bottomFeeder.set(speed);
            topFeeder.set(speed);
        }else{
            bottomFeeder.set(0);
            topFeeder.set(0);
        }
    }

    public void UpdateServo() {

        switch(servoState) {
            case Close:
                servoArm.set(.5);
                break;
            case Open:
            default:
                servoArm.set(0);
                break;
            
        }
    }

    /**
     * teleop for Collector, needs to be called every teleop period
     * @param OpenServo opens servo
     * @param CloseServo closes servo
     */
    public void CollectorTeleop(boolean OpenServo, boolean CloseServo) {
        if(OpenServo) servoState = ServoState.Open;
        if(CloseServo) servoState = ServoState.Close;

        UpdateServo();
    }
}

enum ServoState {
    Open,
    Close,
}
