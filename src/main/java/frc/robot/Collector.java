package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Collector {
    Joystick m_stick = new Joystick(0);
    ColorSensor sensor = new ColorSensor();
    private WPI_TalonSRX bottomFeeder = new WPI_TalonSRX(1);
    private WPI_TalonSRX topFeeder = new WPI_TalonSRX(2);
    double speed = .1;
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
}
