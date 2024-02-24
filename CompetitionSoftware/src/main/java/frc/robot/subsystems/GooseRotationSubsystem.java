// Supports the rotation of the goose arm.
// The coordinate system used here has 0 as the intake down position with angles
// increasing as the arm raises. The assumption is that it starts at a non-zero
// position defined by the constant GooseRotationConstants.kStartingAngle.
// All angles at the interface are in degrees.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.GooseRotationConstants;
import frc.robot.commands.ManualRotateCommand;

public class GooseRotationSubsystem extends PIDSubsystem {
  private double m_angleSet = 0.0;
  private double m_motorSpeed = 0.0;
  private final CANSparkMax m_rotationMotor = new CANSparkMax(GooseRotationConstants.kRotateMotorCANID, 
                                                              MotorType.kBrushless);
  private final RelativeEncoder m_Encoder = m_rotationMotor.getEncoder();
                                                        
  /** Creates a new GooseRotationSubsystem. */
  public GooseRotationSubsystem() 
  {
    super(new PIDController(GooseRotationConstants.kP, 
                            GooseRotationConstants.kI, 
                            GooseRotationConstants.kD));
    
    m_rotationMotor.setSmartCurrentLimit(GooseRotationConstants.kMotorCurrentLimit);
    m_Encoder.setPosition(getMotorPositionFromAngle(GooseRotationConstants.kStartingAngle));

    this.enable();
  }

  private double getAngleFromMotorPosition(double motorShaftRotations)
  {
    double outputShaftRotations = motorShaftRotations / GooseRotationConstants.kOutputShaftRatio;
    double angle = outputShaftRotations * 360.0;

    // It is know that it can return angles outside 0-360. 
    // This is fine for our application.
    return angle;
  }

  private double getMotorPositionFromAngle(double angle)
  {
    double outputShaftRotations = angle / 360.0;
    double motorShaftRotations = outputShaftRotations * GooseRotationConstants.kOutputShaftRatio;
    return motorShaftRotations;
  }

  public void setAngle(double angleDegrees)
  {
    super.setSetpoint(angleDegrees);
    m_angleSet = angleDegrees;
  }

  public double getSetPointAngle()
  {
    return m_angleSet;
  }

  public void setSpeed(double output) {
    m_motorSpeed = output;
    m_rotationMotor.set(m_motorSpeed);
  }

  public double getSpeed()
  {
    return m_motorSpeed;
  }

  public double getRawMeasurement()
  {
    return m_Encoder.getPosition();
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_rotationMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    double position = m_Encoder.getPosition();
    return getAngleFromMotorPosition(position);
  }

  public void initDefaultCommand(XboxController speedJoystick)
  {
    setDefaultCommand(new ManualRotateCommand(this, speedJoystick));
  }
}
