package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Shooter {
    private final Lemonlight JoshsLemon = new Lemonlight();
    private final Autonomous auto = new Autonomous();
    private CANSparkMax leftSpinnyBoi = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax rightSpinnyBoi = new CANSparkMax(2, MotorType.kBrushless);
    WPI_TalonSRX frontCollector = new WPI_TalonSRX(7);
    WPI_TalonSRX backCollector = new WPI_TalonSRX(8);
    private constantVelSpin leftConstantVel = new constantVelSpin(leftSpinnyBoi, false);
    private constantVelSpin rightConstantVel = new constantVelSpin(rightSpinnyBoi, true);

    Joystick m_stick = new Joystick(0);
    ColorSensor sensor = new ColorSensor();
    // private WPI_TalonSRX bottomFeeder = new WPI_TalonSRX(1);
    // private WPI_TalonSRX topFeeder = new WPI_TalonSRX(2);
    double speed = .1;
    Servo servoArm = new Servo(0);
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
     * A Rev Color Match object is used to register and detect known colors. This
     * can
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch m_colorMatcher = new ColorMatch();

    /**
     * Note: Any example colors should be calibrated as the user needs, these
     * are here as a basic example.
     */
    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
    private final Color kBlackTarget = new Color(0, 0, 0);

    Color[] colors = { kBlueTarget, kRedTarget, kYellowTarget };
    int[] states = { 1, 0, 1, 0 };

    public ServoState servoState = ServoState.Open;
    public ColorSeen colorSeen;

    /**
     * @return if the ball should move up the conveyor belt or not
     */

    public void shooterInit() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        leftConstantVel.motorInit();
        rightConstantVel.motorInit();
        JoshsLemon.initTheLemon();
    }

    // public void moveUp() {
    // String color = sensor.ColorToString();
    // if (color == "Green") {
    // bottomFeeder.set(speed);
    // topFeeder.set(speed);
    // } else {
    // bottomFeeder.set(0);
    // topFeeder.set(0);
    // }
    // }

    public void UpdateColor() {
        Color color = kBlackTarget;
        switch (colorSeen) {
            case Red:
                color = kRedTarget;
                // change motor speed
                break;
            case Blue:
                color = kBlueTarget;
                // change motor speed
                break;
            case Abyss:
                color = kBlackTarget;
                // change motor speed;
                break;
        }
    }

    // return color;
    // }
    public String putDash() {
        Color detectedColor = m_colorSensor.getColor();
        int proximity = m_colorSensor.getProximity();
        /**
         * Run the color match algorithm on our detected color
         */
        String colorString = "None";
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        if (match.color == kBlueTarget) {
            colorString = "Blue";
            colorSeen = ColorSeen.Blue;
        } else if (match.color == kRedTarget) {
            colorString = "Red";
            colorSeen = ColorSeen.Red;
        } else {
            colorString = "Black";
            colorSeen = ColorSeen.Abyss;
        }

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the
         * sensor.
         */
        SmartDashboard.putString("Detected Color", colorString);
        return colorString;
    }

    public void UpdateServo() {

        switch (servoState) {
            case Close:
                servoArm.set(.5);
                break;
            case Open:
            default:
                servoArm.set(0);
                break;

        }
    }

    /**
     * teleop for Collector, needs to be called every teleop period
     * 
     * @param OpenServo  opens servo
     * @param CloseServo closes servo
     */
    double shootStartTime = 0;
    boolean isTimeRecorded = false;

    public void shooterTeleop(boolean shootButton, boolean seekAimButton, boolean collectButton, boolean ejectButton, boolean manualShootButton) {
        if (shootButton && !manualShootButton) {
            System.out.println("SHOOOOOT");
            // run shoot motors
            leftConstantVel.shoot(true);
            rightConstantVel.shoot(true);
            //if this is the first loop itteration since the button was pressed, record the current time
            if (!isTimeRecorded) {
                isTimeRecorded = true;
                shootStartTime = System.currentTimeMillis();
            }
            //if three seconds since pushing the button has elapsed, do shootything
            if (System.currentTimeMillis() >= shootStartTime + 2000) {
                // move servo to allow balls to move into shooter
                servoState = ServoState.Open;
                // run collector motors
                frontCollector.set(.9);
                backCollector.set(.9);
                // System.out.print(leftConstantVel.getEncoder());
                // System.out.print(rightConstantVel.getEncoder());
                System.out.println("DISTANCE: " + JoshsLemon.distanceGrab());
            }
            else {
                //keep balls secure untill ready to shoot
                servoState = ServoState.Close;
            }
        }
        else if (manualShootButton) {
            leftSpinnyBoi.set(1300.0/5000.0);
            rightSpinnyBoi.set(1300.0/5000.0);

        } 
        else {
            servoState = ServoState.Close;
            rightConstantVel.shoot(false);
            leftConstantVel.shoot(false);
            isTimeRecorded = false;
            shootStartTime = 0;
        }
        if (seekAimButton) {
            auto.seeking();
            auto.Aiming();
        }
        if (collectButton && !shootButton) {
            frontCollector.set(.9);
            backCollector.set(.9);
        } else if (ejectButton && !shootButton) {
            frontCollector.set(-.9);
            backCollector.set(-.9);
        } else if (!shootButton) {
            frontCollector.set(0);
            backCollector.set(0);
        }
        //make servo do the thing we told it to do
        UpdateServo();
        putDash();
    }
}

enum ServoState {
    Open,
    Close,
}

enum ColorSeen {
    Red,
    Blue,
    Abyss
}
