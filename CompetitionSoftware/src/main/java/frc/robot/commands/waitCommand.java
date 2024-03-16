// This is the default auto command that drives the robot across the auto line.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class waitCommand extends Command {
  private long m_startTime;
  private float m_ms;

  /**
   * Creates a new waitCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  // Takes MS.
  public waitCommand(float ms) 
  {
    m_ms = ms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
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
    return (System.currentTimeMillis() - m_startTime) >= m_ms;
  }
}
