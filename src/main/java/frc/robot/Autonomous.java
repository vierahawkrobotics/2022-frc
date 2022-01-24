package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import static java.lang.Math.*;
//import frc.robot.LemonTest;
import frc.robot.ControlPanel;
import frc.robot.Lemonlight;

public class Autonomous {

    private final Joystick a_stick = new Joystick(0);
    private static int findButt = 9;

    // public void visionAiming(){
    //     double offsetX = Lemonlight.getHorizontalOffset();
    //     double kp = Lemonlight.proportionalControlConstant();
    //     double steeringAdjust = 0.0;
    //     double min =  0.05;

    //     if(a_stick.getRawButton(findButt)){
    //         double headingError = -offsetX;
    //         steeringAdjust = kp*offsetX;
    //         //left command and right command are used to adjust robot
    //         // left_command+=steering_adjust;
    //         // right_command-=steering_adjust;
    //     }
    // }

    public double seeking(){
        double offsetX = Lemonlight.getHorizontalOffset();
        double valid = Lemonlight.validTarget();
        double degreeTurn = 0;
        if(valid == 0.0){
            degreeTurn = 3;

        }else{
            degreeTurn = 0;
        }
        return degreeTurn;
    }

}
