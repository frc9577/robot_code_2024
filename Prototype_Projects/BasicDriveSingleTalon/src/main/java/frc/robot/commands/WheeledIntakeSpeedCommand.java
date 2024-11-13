// This Supports setting a motor to a set speed.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WheeledIntakeSubsystem;

/** An example command that uses an example subsystem. */
public class WheeledIntakeSpeedCommand extends Command {
  private final WheeledIntakeSubsystem m_subsystem;
  private double m_speed = 0.0;

  /**
   * Creates a new WinchStartCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheeledIntakeSpeedCommand(WheeledIntakeSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSpeed(m_speed);
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
