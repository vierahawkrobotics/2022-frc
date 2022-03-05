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

     /**
     * 
     * @param distance distance from limelight to robot
     * @param heightOffset targetHeight - moutned Height
     * @param currentAngle The current mounted Angle
     * @return Complete degree offset in limelight
     */
    

    public static double GetDegreeOffset(double distance, double heightOffset, double currentAngle) {
        return Math.atan(heightOffset / distance) / Math.PI * 180 - currentAngle; 
    }
    
    /**
     * 
     * @return height of limelight on robot to floor
     */
    public double getMountedHeight(){
        double mountHeight = 26;
        return mountHeight;
    }

    /**
     * @return Height of target, 9 feet on game day
     */
    public double getTargetHeight(){
        double tarHeight = 8*12+9;
        return tarHeight;
    }

    /**
     * @return gives the angle of launch
     */
    public double getTheta(){
        // double h = getTargetHeight()-getMountedHeight();
        // double r = distanceGrab();
        // double theta = Math.atan((4*h/r));
        double theta = 74;
        return theta; //degrees
    }

    /**
     * @return gives the required velocity to lauch ball a certain distance
     */
    public double getVelocity(){
        double distance = distanceGrab();//inches
        double heightFinal = getTargetHeight();//inches
        double heightInitial = getMountedHeight();//inches
        double gravity = -4.9*3.28*12; //inches
        double toDeg = Math.PI/180;
        double theta = getTheta()*toDeg;
        double delta = heightFinal-heightInitial; //inches
        double sqrt = (delta-(Math.tan(theta)*distance))/(gravity);
        double denom = Math.cos(theta)*Math.sqrt(sqrt);
        double velocity = distance/denom;

        return velocity; //inches
    }
    /**
     * @return distance from limeLight to a piece of reflective tape
     */
    public double distanceGrab(){
        //target height - camera height
        double distance = 0;
        double heightOffset = getTargetHeight()-getMountedHeight();
        double staticDistance = 125;
        double readAngle = -3.09;

        double offset = GetDegreeOffset(staticDistance, heightOffset, readAngle);

        double angle = Lemonlight.getVertOffset();// this in degrees

        distance = (heightOffset)/((Math.tan((angle+offset) / 180 * Math.PI)));
        return distance+24;
    }
    
    // public double doubleDistance(){
    //     return 2*distanceGrab();
    // }
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

    
   

   
    
}
