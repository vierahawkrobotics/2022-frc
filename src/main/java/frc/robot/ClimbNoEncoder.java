package frc.robot;

public class ClimbNoEncoder {
    ElevatorArm armA;
    ElevatorArm armB;

    public boolean cont;
    boolean prev;

    /**
     * initiates a climber without encoders
     * @param canA can ID for the one set of arms
     * @param canB can ID for the another set of arms
     * @param startDigitalInput the start of the 4 switches, incremented by 1
     */
    public ClimbNoEncoder(int canA, int canB, int startDigitalInput) {
        armA = new ElevatorArm(canA,startDigitalInput,startDigitalInput+1);
        armB = new ElevatorArm(canB,startDigitalInput+2,startDigitalInput+3);
        armA.maxSpeed = 0.2;
        armB.maxSpeed = 0.4;
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
    }
}
