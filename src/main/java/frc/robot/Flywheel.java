package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Flywheel {
    private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private boolean invert;
  Lemonlight ElisLemons = new Lemonlight();
  private final PIDController m_leftPIDController = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.069633,0.067054,0.0060954);
    
}
