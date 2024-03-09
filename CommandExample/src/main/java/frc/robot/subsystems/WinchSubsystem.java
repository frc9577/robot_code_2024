// This is the class provides methods to control a motor based winch.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WinchConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class WinchSubsystem extends SubsystemBase {
  private final CANSparkMax m_WinchMotor = new CANSparkMax(
          WinchConstants.kMotorCANid, MotorType.kBrushless);

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {}

  // Returns winch motor controller.
  public MotorController getMotorController()
  {
    return m_WinchMotor;
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
