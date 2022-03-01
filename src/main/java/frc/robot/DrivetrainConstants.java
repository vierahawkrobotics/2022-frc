package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DrivetrainConstants {
    public static NeutralMode driveMode = NeutralMode.Brake;

    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
    
    public static final double kTrackWidth = 0.5799666582; // meters 22.833333 inches 
    public static final double kWheelRadius = 0.0762; // meters 3 inches
    public static final int kEncoderResolution = 2048;

    public static final double autoTurnSpeed = Math.PI;
    public static final double autoLinearSpeed = 1;     
}
