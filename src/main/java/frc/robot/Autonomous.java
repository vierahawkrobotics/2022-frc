package frc.robot;


public class Autonomous {

    Lemonlight ParkersLemon = new Lemonlight();

    /**
     * 
     * @return In degress, how much you are changing the steering by
     * to see the reflective when it doesnt see it
     */
    public double seeking(){
        //double offsetX = ParkersLemon.getHorizontalOffset();
        double valid = Lemonlight.validTarget();
        double steeringAdjust = 0;
        if(valid == 0.0){
            steeringAdjust = 30;

        }else{
            steeringAdjust = 0;

        }
        return steeringAdjust;
    }
    /**
     * 
     * @return In degress the amount you are adjusting the
     * robot to line it up with the reflective tape
     */
    public double Aiming(){
        double offset = Lemonlight.getHorizontalOffset();
        double steeringAdjust = 0;

        if (offset>=1){
            steeringAdjust = offset;
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
        double range = 3.28;
        double distance = ParkersLemon.distanceGrab()/(12*3.28);
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
