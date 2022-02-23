package frc.robot;


public class Autonomous {

    Lemonlight ParkersLemon = new Lemonlight();

    /**
     * 
     * @return The amount you are adjusting steering by
     */
    public double seeking(){
        //double offsetX = ParkersLemon.getHorizontalOffset();
        double valid = Lemonlight.validTarget();
        double kp = -.1f;
        double tx = ParkersLemon.getHorizontalOffset();
        double steeringAdjust = 0;
        if(valid == 0.0){
            steeringAdjust = tx*Math.PI/180;

        }else{
            steeringAdjust = 0;

        }
        return steeringAdjust;
    }

    /**
     * 
     * @return how many inches you move forward
     */
    public double getInRange(){
        double range = 1;
        double distance = ParkersLemon.distanceGrab();
        return distance-range; 
    }

    /**
     * 
     * @return getting out of zone during 15 second autonomous period
     */
    public double getOut(){
        double get = 3.045;
        return get; 
    }
}
