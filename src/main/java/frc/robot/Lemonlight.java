package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Notifier;
//import static java.lang.Math.*;
//import frc.robot.LemonTest;
//import frc.robot.ControlPanel;

public class Lemonlight {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTable m_table;
    private String m_tableName = "limelight";

    public static void lemonLightPeriodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        //double initialVelocity = 0;
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    
    }
   public void LemonTest() {
        //m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
        //hearBeat.startPeriodic(_hearBeatPeriod);
    }

    /**
     * read values periodically
     * @return
     */
    public static double getVertOffset(){
        //System.out.println("hello");
        NetworkTableEntry ty = m_table.getEntry("ty");
        double a = ty.getDouble(0.0);
        return a;
    }
    public static double getHorizontalOffset(){
        NetworkTableEntry tx = m_table.getEntry("tx");
        double b = tx.getDouble(0.0);
        return b;
    }

    public static double getArea(){
        NetworkTableEntry ta = m_table.getEntry("ta");
        double c = ta.getDouble(0.0);
        return c;
    }
    public static double getMountingAngle(){
        double mountingAngle = -10;
        return mountingAngle;
    }
    public double distanceGrab(){
        //target height - camera height
        double distance = 0;
        double heightOffset = getTargetHeight()-getMountedHeight();
        double staticDistance = 107;
        double readAngle = 1.73;

        //distance = (targetHeight-mountedHeight)/((Math.tan((mountingAngle+targetAngle) / 180 * 3.141592653589)));
        double offset = GetDegreeOffset(staticDistance, heightOffset, readAngle);

        double angle = Lemonlight.getVertOffset();// this in degrees

        distance = (heightOffset)/((Math.sin((angle+offset) / 180 * Math.PI)));
        //distance = (targetHeight-mountedHeight)/((Math.tan(mountingAngle+targetAngle)));
        return distance;
    }

    public double seeking(){
        double tx = getHorizontalOffset();
        double kp = -.1;//Proportional Control Constant
        double min = .05; // minimum amount needed to move robot
        double error = -1*tx;
        double steeringAdjust = kp*tx;
        if(tx>1.0){
            steeringAdjust= error*kp + min;
        }
        else if (tx<1.0) {
            steeringAdjust = error*kp - min;
        }
        return steeringAdjust;

    }
    public static double GetDegreeOffset(double distance, double heightOffset, double currentAngle) {
        return Math.asin(heightOffset / distance) / Math.PI * 180 - currentAngle; 
    }
    
   
    public static double proportionalControlConstant(){
        double kp = -0.1;
        return kp;
    }
    public static double validTarget(){
        NetworkTableEntry tv = m_table.getEntry("tv");
        double b = tv.getDouble(0.0);
        return b;
    }

    public double getMountedHeight(){
        double mountHeight = 36;
        return mountHeight;
    }

    public double getTargetHeight(){
        double tarHeight = 42;
        return tarHeight;
    }

    public double getTheta(){
        double h = getTargetHeight()-getMountedHeight();
        double r = distanceGrab();
        double theta = Math.atan((4*h/r));
        return theta;
    }

    public double getVelocity(){
        double x = distanceGrab();
        double g = 32.17;
        double theta = getTheta();
        double sqrt = (x*g)/(2*Math.sin(theta));
        double v = Math.sqrt((sqrt));
        return v; 
    }

    public double getHypot(){
        double hypot = Math.sqrt(distanceGrab()*distanceGrab()+(getTargetHeight()-getMountedHeight())*(getTargetHeight()-getMountedHeight()));
        return hypot;
    }

    
}
