package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheeledIntakeConstants;

public class WheeledIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor = new CANSparkMax(WheeledIntakeConstants.kLeftMotorCANID,
                                                            MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(WheeledIntakeConstants.kRightMotorCANID,
                                                            MotorType.kBrushless);

    private double m_leftSpeed = 0.0;
    private double m_rightSpeed = 0.0;

    /** Creates a new WheeledIntakeSubsystem. */  
    public WheeledIntakeSubsystem()
    {
        m_leftMotor.setSmartCurrentLimit(WheeledIntakeConstants.kLeftMotorCurrentLimit);
        m_rightMotor.setSmartCurrentLimit(WheeledIntakeConstants.kRightMotorCurrentLimit);
    }

    // TODO: Make motors spin at the same speed in opisate directions (NEED TEST)
    public void setSpeed(double speed)
    {
        m_leftMotor.set(speed);
        m_leftSpeed = speed;

        m_rightMotor.set(-speed);
        m_rightSpeed = -speed;
    }

    // Returns last COMMANDED speed
    public double getSpeed()
    {
        return m_leftSpeed;
    }

    public void setLeftSpeed(double speed)
    {
        m_leftMotor.set(speed);
        m_leftSpeed = speed;
    }

    // Returns last COMMANDED speed
    public double getLeftSpeed()
    {
        return m_leftSpeed;
    }

    public void setRightSpeed(double speed)
    {
        m_rightMotor.set(speed);
        m_rightSpeed = speed;
    }

    // Returns last COMMANDED speed
    public double getRightSpeed()
    {
        return m_rightSpeed;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
