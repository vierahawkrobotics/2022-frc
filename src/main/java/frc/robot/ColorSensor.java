package frc.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor {
     /**
   * Change the I2C port below to match the connection of your color sensor
   */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
    private final ColorMatch m_colorMatcher = new ColorMatch();
    
    /**
     * 
     * @return a color value for what color ball is in the belt
     */
    public String ColorToString(){
        String Color = "";
        //Color sensed = m_colorSensor.getColor();
        int colorR = m_colorSensor.getRed();
        int colorB = m_colorSensor.getBlue();
        int colorG = m_colorSensor.getGreen();

        if(colorR>colorB && colorR>colorG){
            Color = "Red";            
        }else if(colorB>colorR && colorB>colorG){
            Color = "Blue";
        }else if (colorG>colorB && colorG>colorR){
            Color = "Green";
        }
        SmartDashboard.putString("ColorSensor", Color);
        return Color;

    }

    
    

}
