package frc.robot;

import com.ctre.phoenix.time.StopWatch;

public class ElevatorArmAnimator {
    public ElevatorArm armA;
    public ElevatorArm armB;
    public double delay;
    public EAItter[] itters;
    double prevIterTime;
    int index;
    StopWatch timer;

    /**
     * initiates an animator controller
     * @param armA armA
     * @param armB armB
     * @param delay delay (in seconds) for how long each animation state lasts
     */
    public ElevatorArmAnimator(ElevatorArm armA, ElevatorArm armB, double delay) {
        this.delay = delay;
        this.armA = armA;
        this.armB = armB;
        timer = new StopWatch();
        index = 0;
        prevIterTime = 0;
    }

    /**restarts animator (required) */
    public void Start() {
        timer.start();
        index = 0;
        prevIterTime = 0;
    }

    /**
     * updates the animator to next state after the delay (call every robot itteration)
     * @return true if animation is not done;
     */
    public boolean Update() {

        itters[index].SetAnim(this);

        double time = timer.getDuration();
        if(prevIterTime < time) {
            prevIterTime = time + delay;
            index++;
        }

        return index < itters.length;
    }
    
}

class EAItter {
    public double armPosA;
    public double armPosB;
    
    public double armSpeedA;
    public double armSpeedB;

    /**
     * an elevator arm animation state (0 to 1 for all param)
     * @param armPosA the position of the armA
     * @param armPosB the position of the armB
     * @param armSpeedA the speed of the armA
     * @param armSpeedB the speed of the armB
     */
    public EAItter(double armPosA, double armPosB, double armSpeedA, double armSpeedB) {
        this.armPosA = armPosA;
        this.armPosB = armPosB;
        this.armSpeedA = armSpeedA;
        this.armSpeedB = armSpeedB;
    }

    /**
     * sets the animator to this animation state
     * @param anim the animator that called this
     * @deprecated
     */
    public void SetAnim(ElevatorArmAnimator anim) {
        anim.armA.SetPos(armPosA);
        anim.armB.SetPos(armPosB);
        anim.armA.maxSpeed = armSpeedA;
        anim.armB.maxSpeed = armSpeedB;
    }
}
