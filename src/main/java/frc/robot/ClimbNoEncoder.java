package frc.robot;

public class ClimbNoEncoder {
    ElevatorArm armA;
    ElevatorArm armB;

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
    }
    
    /**
     * sets the speed of the motors
     * @param speedA the speed of the first arm
     * @param speedB the speed of the second arm
     */
    public void SetMotors(double speedA, double speedB) {
        armA.SetSpeed(speedA);
        armB.SetSpeed(speedB);
    }
}
