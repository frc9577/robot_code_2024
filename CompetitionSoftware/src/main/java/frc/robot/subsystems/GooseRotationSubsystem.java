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
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.GooseRotationConstants;

public class GooseRotationSubsystem extends PIDSubsystem {
  private double m_angleSet = 0.0;
  private final CANSparkMax m_rotationMotor = new CANSparkMax(GooseRotationConstants.kRotateMotorCANID, 
                                                              MotorType.kBrushless);
  private final RelativeEncoder m_Encoder = m_rotationMotor.getEncoder();
                                                        
  /** Creates a new GooseRotationSubsystem. */
  public GooseRotationSubsystem() 
  {
    super(new PIDController(GooseRotationConstants.kP, 
                            GooseRotationConstants.kI, 
                            GooseRotationConstants.kD));
    
    m_Encoder.setPosition(getMotorPositionFromAngle(GooseRotationConstants.kStartingAngle));
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

  @Override
  protected void useOutput(double output, double setpoint) {
    m_rotationMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    double position = m_Encoder.getPosition();
    return getAngleFromMotorPosition(position);
  }
}
