package frc.robot;

public class Climb {

    ElevatorArmAnimator anim;
    public boolean cont;
    public Climb() {
        anim = new ElevatorArmAnimator(new ElevatorArm(1, -1), new ElevatorArm(), 1);
        anim.itters = new EAItter[] {
            new EAItter(1, 0, 1, 1),
            new EAItter(0, 1, 1, 1),
            new EAItter(0.5, 0.5, 1, 1),
            new EAItter(0.25, 0.5, 1, 1),
            new EAItter(0.8, 0.5, 1, 1),
        };
    }

    public void Start() {
        anim.Start();
    }

    public void Itterate() {
        if(cont) {
            cont = anim.Update();                
        }
    }

    // State change
    // Execute State
    // Safety Stop
    // Reverse
    //

    // Can motors
}