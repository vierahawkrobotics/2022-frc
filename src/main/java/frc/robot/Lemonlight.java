package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import static java.lang.Math.*;
import frc.robot.LemonTest;
import frc.robot.ControlPanel;

public class Lemonlight {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTable m_table;
    private String m_tableName;
    
    //read values periodically
    public static double getVertOffset(){
        NetworkTableEntry ty = m_table.getEntry("ty");
        double a = ty.getDouble(0.0);
        return a;
    }
    public static double getHorizontalOffset(){
        NetworkTableEntry ty = m_table.getEntry("tx");
        double b = tx.getDouble(0.0);
        return b;
    }

    public static double getArea(){
        NetworkTableEntry ta = m_table.getEntry("ta");
        double c = ta.getDouble(0.0);
        return c;
    }
    public static double distanceGrab(){
        //measurements in inches, sorry ik you need centimeters
        double distance = 0;
        //a1 - Angle that the camera is mounted
        double mountingAngle = 180;
        //a2 - retrieve from camera
        double targetAngle = Lemonlight.getVertOffset();
        //how tall camera is too floor
        double mountedHeight = 24;
        //heght of target off floor
        double targetHeight = 26;

        distance = (mountedHeight-targetHeight)/(Math.tan(mountingAngle+targetAngle));

        return distance;
    }
    public static void lemonLightPeriodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double initialVelocity = 0;
    //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    
    }
    
    // public static double distanceGrab(){
    //     //measurements in inches, sorry ik you need centimeters
    //     double distance = 0;
    //     //a1 - Angle that the camera is mounted
    //     double mountingAngle = 180;
    //     //a2 - retrieve from camera
    //     double targetAngle = 0;
    //     //how tall camera is too floor
    //     double mountedHeight = 24;
    //     double targetHeight = 26;

    //     distance = (mountedHeight-targetHeight)/(Math.tan(mountingAngle+targetAngle));

    //     return distance;
    // }

    public static void initialVelocity(){

    }
}
