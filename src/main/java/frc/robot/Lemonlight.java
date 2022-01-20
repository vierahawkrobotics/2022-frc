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
    private NetworkTable m_table;
    private String m_tableName;
    
    //read values periodically
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
