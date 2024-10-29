package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheeledIntakeConstants;

public class WheeledIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_leftMotor = new CANSparkMax(WheeledIntakeConstants.kLeftMotorCANID,
                                                            MotorType.kBrushless);

    public WheeledIntakeSubsystem()
    {

    }
}
