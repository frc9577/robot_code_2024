// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.TwoSolenoids;

/** An example command that uses an example subsystem. */
public class DoubleSolenoidSetCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Subsystem m_subsystem;
  private TwoSolenoids m_twoSolenoids;
  private DoubleSolenoid.Value m_position = DoubleSolenoid.Value.kOff;

  /**
   * Creates a new DoublSolenoidSetCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleSolenoidSetCommand(Subsystem subsystem, TwoSolenoids twoSolenoids, DoubleSolenoid.Value position) {
    m_subsystem = subsystem;
    m_twoSolenoids = twoSolenoids;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_twoSolenoids.left.set(m_position);
    m_twoSolenoids.right.set(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
