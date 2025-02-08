package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawdiaConstants;

public class ClawdiaSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(ClawdiaConstants.kMotorCANID, 
                                                            MotorType.kBrushless);

    private double m_motorSpeed = 0.0;

    /** Creates a new WheeledIntakeSubsystem. */  
    public ClawdiaSubsystem()
    {
        m_motor.setSmartCurrentLimit(ClawdiaConstants.kMotorCurrentLimit);
    }

    // Returns last COMMANDED speed
    public double getSpeed()
    {
        return m_motorSpeed;
    }

    public void setSpeed(double speed)
    {
        m_motor.set(speed);
        m_motorSpeed = speed;
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
