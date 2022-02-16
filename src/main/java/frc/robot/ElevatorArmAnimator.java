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

    public ElevatorArmAnimator(ElevatorArm armA, ElevatorArm armB, double delay) {
        this.delay = delay;
        this.armA = armA;
        this.armB = armB;
        timer = new StopWatch();
        index = 0;
        prevIterTime = 0;
    }

    public void Start() {
        timer.start();
        index = 0;
        prevIterTime = 0;
    }

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

    public EAItter(double armPosA, double armPosB, double armSpeedA, double armSpeedB) {
        this.armPosA = armPosA;
        this.armPosB = armPosB;
        this.armSpeedA = armSpeedA;
        this.armSpeedB = armSpeedB;
    }

    public void SetAnim(ElevatorArmAnimator anim) {
        anim.armA.Set(armPosA);
        anim.armB.Set(armPosB);
        anim.armA.maxSpeed = armSpeedA;
        anim.armB.maxSpeed = armSpeedB;
    }
}
