package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Lemonlight {

    /**
     * Network table values are returned
     * straight from the limelight.These
     * values can be turned into useful things
     * such as horizontal offset (tx), vertical
     * offset (ty), validity of a target (tv)
     * and area the target takes on a screen (tx)
     */

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTable m_table;
    private String m_tableName = "limelight";

     /**
     * read values periodically
     */
    public static void lemonLightPeriodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    
    }

    /**
    * initialization function, need to call this in 
    * order to use Lemonlight functions
    */
   public void LemonTest() {
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
   }

     /**
     * @return vertical offset to object from limeLight
     */
    public static double getVertOffset(){
        NetworkTableEntry ty = m_table.getEntry("ty");
        double a = ty.getDouble(0.0);
        return a;
    }
    /**
     * @return horizontal offset to an object from limelight
     */
    public static double getHorizontalOffset(){
        NetworkTableEntry tx = m_table.getEntry("tx");
        double b = tx.getDouble(0.0);
        return b;
    }

     /**
     * @return area of object on the limelight
     */

    public static double getArea(){
        NetworkTableEntry ta = m_table.getEntry("ta");
        double c = ta.getDouble(0.0);
        return c;
    }

    /**
     * @returns if liemlight detects a reflective tape
     */
    public static double validTarget(){
        NetworkTableEntry tv = m_table.getEntry("tv");
        double b = tv.getDouble(0.0);
        return b;
    }

    
    /**
     * This will vary on the actual robot, it just 
     * gives the angle the limelight is mounted at
     * @return mounting angle of Limelight
    */
    public static double getMountingAngle(){
        double mountingAngle = -10;
        return mountingAngle;
    }

    public static double GetDegreeOffset(double distance, double heightOffset, double currentAngle) {
        return Math.asin(heightOffset / distance) / Math.PI * 180 - currentAngle; 
    }
    
    /**
     * 
     * @return height of limelight on robot to floor
     */
    public double getMountedHeight(){
        double mountHeight = 36;
        return mountHeight;
    }

    /**
     * @return Height of target, 9 feet on game day
     */
    public double getTargetHeight(){
        double tarHeight = 42;
        return tarHeight;
    }

    /**
     * @return gives the angle of launch
     */
    public double getTheta(){
        double h = getTargetHeight()-getMountedHeight();
        double r = distanceGrab();
        double theta = Math.atan((4*h/r));
        return theta;
    }

    /**
     * @return gives the required velocity to lauch ball a certain distance
     */
    public double getVelocity(){
        double x = distanceGrab();
        double g = 32.17*12;
        double theta = getTheta();
        double sqrt = (x*g)/(2*Math.sin(theta));
        double v = Math.sqrt((sqrt));
        return v; 
    }
    /**
     * @return distance from limeLight to a piece of reflective tape
     */
    public double distanceGrab(){
        //target height - camera height
        double distance = 0;
        double heightOffset = getTargetHeight()-getMountedHeight();
        double staticDistance = 107;
        double readAngle = 1.73;

        double offset = GetDegreeOffset(staticDistance, heightOffset, readAngle);

        double angle = Lemonlight.getVertOffset();// this in degrees

        distance = (heightOffset)/((Math.sin((angle+offset) / 180 * Math.PI)));
        return distance;
    }

     /**
     * @return double True distance to reflective tape, hypotenuse of horizonatal distance and vertical distance
     */
    public double getHypot(){
        double hypot = Math.sqrt(distanceGrab()*distanceGrab()+(getTargetHeight()-getMountedHeight())*(getTargetHeight()-getMountedHeight()));
        return hypot;
    }

    /**
     * If it sees a valid target, it will line up with it
     * Needs work, no longer works with PID
     * @return double adjustment necessary for driveTrain
     */
    // public double Aiming(){
    //     double tx = getHorizontalOffset();
    //     double kp = -.1;//Proportional Control Constant
    //     double min = .05; // minimum amount needed to move robot
    //     double error = -1*tx;
    //     double steeringAdjust = kp*tx;
    //     if(tx>1.0){
    //         steeringAdjust= error*kp + min;
    //     }
    //     else if (tx<1.0) {
    //         steeringAdjust = error*kp - min;
    //     }
    //     return steeringAdjust;

    // }

    public double Aiming(){
        double rad = 0;
        double offset = getHorizontalOffset();
        double valid = validTarget();

        if (valid == 1.0){
            rad = (Math.PI/180);
        }else{
            rad = offset*(Math.PI/180);
        }

        return rad;
    }

    /**
     * 
     * @param distance distance from limelight to robot
     * @param heightOffset targetHeight - moutned Height
     * @param currentAngle The current mounted Angle
     * @return Complete degree offset in limelight
     */
    

   
    
}
