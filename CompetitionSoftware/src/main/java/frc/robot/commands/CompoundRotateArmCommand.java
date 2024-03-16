// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GooseRotationConstants;
import frc.robot.subsystems.GooseRotationSubsystem;

/** An example command that uses an example subsystem. */
public class CompoundRotateArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final GooseRotationSubsystem m_subsystem;

  /**
   * Creates a new CompoundRotateArm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CompoundRotateArmCommand(GooseRotationSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    // idk if this is the proper way to format compound commands
    andThen(new RotateCommand(m_subsystem,GooseRotationConstants.kMiddleAngle));
    andThen(new RotateCommand(m_subsystem,GooseRotationConstants.kIntakeAngle));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new RotateCommand(m_subsystem,GooseRotationConstants.kScoreAngle);
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
