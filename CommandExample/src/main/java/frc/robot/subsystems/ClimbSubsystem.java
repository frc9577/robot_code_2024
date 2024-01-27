// This is the class provides methods to control a pnuematics based climb.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
 private final DoubleSolenoid m_ClimbSolenoidL = new DoubleSolenoid(
                  ClimbConstants.kPhCANid, PneumaticsModuleType.REVPH, 
                  ClimbConstants.kExtendChannelL, ClimbConstants.kRetractChannelL);
 private final DoubleSolenoid m_ClimbSolenoidR = new DoubleSolenoid(
                  ClimbConstants.kPhCANid, PneumaticsModuleType.REVPH, 
                  ClimbConstants.kExtendChannelR, ClimbConstants.kRetractChannelR);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  // Returns solinoid one.
  public TwoSolenoids getDoubleSolenoid()
  {
    return new TwoSolenoids(m_ClimbSolenoidL, m_ClimbSolenoidR);
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
