package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbNoEncoder {
    ElevatorArm armA;
    ElevatorArm armB;

    DigitalInput limit = new DigitalInput(2);

    public boolean cont;
    boolean prev;

    /**
     * initiates a climber without encoders
     * @param canA can ID for the one set of arms
     * @param canB can ID for the another set of arms
     * @param maxSpeed the maxSpeed of both arms
     */
    public ClimbNoEncoder(int canA, int canB, double maxSpeed) {
        armA = new ElevatorArm(canA);
        armB = new ElevatorArm(canB);
        armA.maxSpeed = maxSpeed;
        armB.maxSpeed = maxSpeed;
        cont = true;
    }

    public void StopMotors() {
        armA.Stop();
        armB.Stop();
    }
    
    /**
     * sets the speed of the motors, does not set when 'cont' is false
     * @param speedA the speed of the first arm
     * @param speedB the speed of the second arm
     */
    public void SetMotors(double speedA, double speedB) {
        if(!cont) {
            StopMotors();
            return;
        }
        armA.SetSpeed(speedA);
        armB.SetSpeed(speedB);

        boolean next = ReadSensor();
        
        if(prev && !next) {
            cont = false;
        }
        prev = next;
    }

    boolean ReadSensor() {
        return limit.get();
    }
}
