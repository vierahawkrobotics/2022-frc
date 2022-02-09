package frc.robot;


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

    //first 15 seconds you need to get out and get a ball
    public double getOut(){
        double get = 3.045;
        return get; 
    }

    //shootBall

}
