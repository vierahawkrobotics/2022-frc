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
        double tx = Lemonlight.getHorizontalOffset();
        double steeringAdjust = 0;
        if(valid == 0.0){
            steeringAdjust = tx*Math.PI/180;

        }else{
            steeringAdjust = 0;

        }
        return steeringAdjust;
    }

    public double Aiming(){
        double rad = 0;
        double offset = Lemonlight.getHorizontalOffset();
        double valid = Lemonlight.validTarget();

        if (valid == 1.0){
            rad = (Math.PI/180);
        }else{
            rad = offset*(Math.PI/180);
        }

        return rad;
    }


    /**
     * 
     * @return how many inches you move forward
     */
    public double getInRange(){
        double range = 12*12;
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
