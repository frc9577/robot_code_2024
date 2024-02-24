// Supports the rotation of the goose arm.
// The coordinate system used here has 0 as the intake down position with angles
// increasing as the arm raises. The assumption is that it starts at a non-zero
// position defined by the constant GooseRotationConstants.kStartingAngle.
// All angles at the interface are in degrees.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GooseRotationConstants;

public class GooseRotationSubsystem extends SubsystemBase {
  private double m_angleSet = 0.0;
  private final CANSparkMax m_rotationMotor = new CANSparkMax(GooseRotationConstants.kRotateMotorCANID, 
                                                              MotorType.kBrushless);
  private SparkAbsoluteEncoder m_Encoder;
  private SparkPIDController m_pidController;

  public double m_kP, m_kI, m_kD, m_kIz, m_kFF, m_kMaxOutput, m_kMinOutput;
                                                        
  /** Creates a new GooseRotationSubsystem. */
  public GooseRotationSubsystem() 
  {
    m_rotationMotor.restoreFactoryDefaults();
    m_rotationMotor.setInverted(true);
    m_rotationMotor.setSmartCurrentLimit(GooseRotationConstants.kMotorCurrentLimit);
    
    m_Encoder = m_rotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    // TODO: We may need to set the current position here or, perhaps, just read the encoder
    // so that we know where our reference is.

    m_pidController = m_rotationMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_Encoder);

    // Set default PID coefficients.
    m_kP = GooseRotationConstants.kP;
    m_kI = GooseRotationConstants.kI;
    m_kD = GooseRotationConstants.kD;
    m_kIz = GooseRotationConstants.kIZone;
    m_kFF = GooseRotationConstants.kFF;
    m_kMaxOutput = GooseRotationConstants.kMaxOutput;
    m_kMinOutput = GooseRotationConstants.kMinOutput;

    // Set PID coefficients
    m_pidController.setP(m_kP);
    m_pidController.setI(m_kI);
    m_pidController.setD(m_kD);
    m_pidController.setIZone(m_kIz);
    m_pidController.setFF(m_kFF);
    m_pidController.setOutputRange(m_kMinOutput, m_kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", m_kP);
    SmartDashboard.putNumber("I Gain", m_kI);
    SmartDashboard.putNumber("D Gain", m_kD);
    SmartDashboard.putNumber("I Zone", m_kIz);
    SmartDashboard.putNumber("Feed Forward", m_kFF);
    SmartDashboard.putNumber("Max Output", m_kMaxOutput);
    SmartDashboard.putNumber("Min Output", m_kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
  }

  // Given a position read from the encoder, return the equivalent arm
  // angle.
  private double getAngleFromMotorPosition(double encoderReading)
  {
    // TODO: Perform scaling and translation as necessary.
    // For now, just return the provided value.
    return encoderReading;
  }

  // Given a desired arm angle in degrees, return the equivalent setpoint
  // value to pass to the PID controller.
  private double getMotorPositionFromAngle(double angle)
  {
    // TODO: We will likely have to scale and offset the result. For now, we just pass back
    // the same value.
    return angle;
  }

  public void setAngle(double angleDegrees)
  {
    m_angleSet = angleDegrees;
    double setpoint = getMotorPositionFromAngle(angleDegrees);
    m_pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public double getSetPointAngle()
  {
    return m_angleSet;
  }

  public double getSpeed()
  {
    return m_rotationMotor.get();
  }

  public double getRawMeasurement()
  {
    return m_Encoder.getPosition();
  }

  public void initDefaultCommand(XboxController speedJoystick)
  {
    // TODO: Reinstate this one tuning is done and we're not using the 
    // Shuffleboard widgets to set coefficients.
    //
    // setDefaultCommand(new ManualRotateCommand(this, speedJoystick));
  }

  @Override
  public void periodic()
  {
    // TODO: Remove coefficient reading when we've finished tuning!

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double setangle = SmartDashboard.getNumber("Goose Set Point", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != m_kP)) { m_pidController.setP(p); m_kP = p; }
    if((i != m_kI)) { m_pidController.setI(i); m_kI = i; }
    if((d != m_kD)) { m_pidController.setD(d); m_kD = d; }
    if((iz != m_kIz)) { m_pidController.setIZone(iz); m_kIz = iz; }
    if((ff != m_kFF)) { m_pidController.setFF(ff); m_kFF = ff; }
    if((max != m_kMaxOutput) || (min != m_kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      m_kMinOutput = min; m_kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     */
    m_angleSet = setangle;
    m_pidController.setReference(setangle, CANSparkMax.ControlType.kPosition);
  }

}
