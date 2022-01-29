package frc.robot;
import static java.lang.Math.*;
//import frc.robot.LemonTest;

public class Autonomous {

    Lemonlight ParkersLemon = new Lemonlight();

    public double seeking(){
        //double offsetX = ParkersLemon.getHorizontalOffset();
        double valid = Lemonlight.validTarget();
        double degreeTurn = 0;
        if(valid == 0.0){
            degreeTurn = 10;

        }else{
            degreeTurn = 0;
        }
        return degreeTurn;
    }

    public double getInRange(){
        double range = 1;
        double distance = ParkersLemon.distanceGrab();
        return distance-range;
    }

}
