package frc.robot;

import java.time.OffsetDateTime;

import javax.swing.text.AbstractDocument.BranchElement;

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
import com.ctre.phoenix.motorcontrol.NeutralMode;
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

    ShooterState shooterState = ShooterState.DO_NOTHING;

    double moveDownStartTime = 0;
    double spinUpStartTime = 0;
    double shoot1StartTime = 0;
    double shoot2StartTime = 0;

    DriveTrain m_drive;
    Shooter shoot;
    Climb climb;

    // Joystick m_stick = new Joystick(0);
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

    public Shooter(DriveTrain m_drive, Climb climb) {
        this.m_drive = m_drive;
        this.climb = climb;
    }

    public void shooterInit() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        leftConstantVel.motorInit();
        rightConstantVel.motorInit();
        //JoshsLemon.initTheLemon();
        frontCollector.setNeutralMode(NeutralMode.Coast);
        backCollector.setNeutralMode(NeutralMode.Coast);
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

    public void Idle(boolean manualShootButton, boolean oneBallShootButton, boolean twoBallShootButton, boolean collectButton, boolean ejectButton) {
        if (!(manualShootButton || oneBallShootButton || twoBallShootButton)) {
            rightConstantVel.shoot(false);
            leftConstantVel.shoot(false);

            if (!collectButton && !ejectButton) {
                frontCollector.set(0);
                backCollector.set(0);
            }
        }

    }

    public void shootAutomation(boolean shootButton, boolean collectButton, boolean ejectButton) {
        switch (shooterState) {
            case DO_NOTHING:
                System.out.println("Begin DO NOTHING");

                moveDownStartTime = 0;
                spinUpStartTime = 0;
                shoot1StartTime = 0;
                shoot2StartTime = 0;

                if (shootButton) {
                    frontCollector.set(0);
                    backCollector.set(0);
                    rightConstantVel.shoot(false);
                    leftConstantVel.shoot(false);
                    shooterState = ShooterState.MOVE_DOWN;
                }
                break;

            case MOVE_DOWN:
                System.out.println("Begin MOVE DOWN");
                if (shootButton) {
                    rightConstantVel.shoot(false);
                    leftConstantVel.shoot(false);
                    if (moveDownStartTime == 0) {
                        moveDownStartTime = System.currentTimeMillis();
                        frontCollector.set(-0.2);
                        backCollector.set(-0.2);
                    }

                    else if (moveDownStartTime + 750 >= System.currentTimeMillis()) {
                        frontCollector.set(-0.2);
                        backCollector.set(-0.2);
                    }

                    else {
                        frontCollector.set(0);
                        backCollector.set(0);
                        shooterState = ShooterState.SPIN_UP;
                    }
                }

                else {
                    frontCollector.set(0);
                    backCollector.set(0);
                    shooterState = ShooterState.DO_NOTHING;
                }
                break;

            case SPIN_UP:
                System.out.println("Begin SPIN UP");
                if (shootButton) {
                    if (spinUpStartTime == 0) {
                        spinUpStartTime = System.currentTimeMillis();
                        leftConstantVel.shoot(true);
                        rightConstantVel.shoot(true);
                    }

                    else if (spinUpStartTime + 1500 >= System.currentTimeMillis()) {
                        leftConstantVel.shoot(true);
                        rightConstantVel.shoot(true);
                    }

                    else {
                        shooterState = ShooterState.SHOOT_BALL_1;
                    }
                } else {
                    leftConstantVel.shoot(false);
                    rightConstantVel.shoot(false);
                    shooterState = ShooterState.DO_NOTHING;
                }

                break;

            case SHOOT_BALL_1:
                System.out.println("Begin SHOOT BALL 1");
                if (shootButton) {
                    leftConstantVel.shoot(true);
                    rightConstantVel.shoot(true);
                    if (shoot1StartTime == 0) {
                        shoot1StartTime = System.currentTimeMillis();
                        frontCollector.set(0.1);
                        backCollector.set(0.1);
                    }

                    else if (shoot1StartTime + 250 >= System.currentTimeMillis()) {
                        frontCollector.set(0.1);
                        backCollector.set(0.1);
                    }

                    else {
                        frontCollector.set(0);
                        backCollector.set(0);
                        shooterState = ShooterState.SHOOT_BALL_2;
                    }

                } else {
                    frontCollector.set(0);
                    backCollector.set(0);
                    leftConstantVel.shoot(false);
                    rightConstantVel.shoot(false);
                    shooterState = ShooterState.DO_NOTHING;
                }
                break;

            case SHOOT_BALL_2:
                System.out.println("Begin SHOOT BALL 2");
                if (shootButton) {
                    leftConstantVel.shoot(true);
                    rightConstantVel.shoot(true);
                    if (shoot2StartTime == 0) {
                        shoot2StartTime = System.currentTimeMillis();
                        frontCollector.set(0.7);
                        backCollector.set(0.7);
                    }

                    else if (shoot2StartTime + 1000 > System.currentTimeMillis()) {
                        frontCollector.set(0.7);
                        backCollector.set(0.7);
                    }

                    else {
                        frontCollector.set(0);
                        backCollector.set(0);
                        // leftConstantVel.shoot(false);
                        // rightConstantVel.shoot(false);
                        shooterState = ShooterState.DO_NOTHING;
                    }

                } else {
                    frontCollector.set(0);
                    backCollector.set(0);
                    leftConstantVel.shoot(false);
                    rightConstantVel.shoot(false);
                    shooterState = ShooterState.DO_NOTHING;
                }
                break;
        }
    }

    public void shooterTeleop(boolean shootButton, boolean seekAimButton, boolean collectButton, boolean ejectButton,
            boolean manualShootButton) {

        if (shootButton && !manualShootButton) {
            System.out.println("SHOOOOOT");
            // run shoot motors
            leftConstantVel.shoot(true);
            rightConstantVel.shoot(true);
            // if this is the first loop itteration since the button was pressed, record
            // the current time
            if (!isTimeRecorded) {
                isTimeRecorded = true;
                shootStartTime = System.currentTimeMillis();
            }
            // if three seconds since pushing the button has elapsed, do shootything
            if (System.currentTimeMillis() >= shootStartTime + 2000) {
                // move servo to allow balls to move into shooter
                servoState = ServoState.Open;
                // run collector motors
                frontCollector.set(.9);
                backCollector.set(.9);
                // System.out.print(leftConstantVel.getEncoder());
                // System.out.print(rightConstantVel.getEncoder());
                System.out.println("DISTANCE: " + JoshsLemon.distanceGrab());
            } else {
                // keep balls secure untill ready to shoot
                servoState = ServoState.Close;
            }
        } else if (manualShootButton) {
            leftSpinnyBoi.set(1455.0 / 5000.0);
            rightSpinnyBoi.set(1455.0 / 5000.0);

        } else {
            servoState = ServoState.Close;
            isTimeRecorded = false;
        }

        if (seekAimButton) {
            // auto.seeking();
            Aiming();
        }
        if (collectButton && !shootButton) {
            frontCollector.set(.9);
            backCollector.set(.9);
        } else if (ejectButton && !shootButton) {
            frontCollector.set(-.9);
            backCollector.set(-.9);
        } else if (!shootButton) {
            // frontCollector.set(0);
            // backCollector.set(0);
        }
        // make servo do the thing we told it to do
        // UpdateServo();
        // putDash();
    }

    public void Aiming() {
        double offset = Lemonlight.getHorizontalOffset();
        double steeringAdjust = 0;

        if (Math.abs(offset) >= 3) {
            steeringAdjust = -offset;
        } 
        else {
            steeringAdjust = 0;
        }

        m_drive.gotoAngle(steeringAdjust);
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

enum ShooterState {
    DO_NOTHING,
    MOVE_DOWN,
    SPIN_UP,
    SHOOT_BALL_1,
    SHOOT_BALL_2,
}
