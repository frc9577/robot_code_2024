// This Supports setting a Double Solenoid to a set position.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GooseRotationSubsystem;

/** An example command that uses an example subsystem. */
public class ManualRotateCommand extends Command {
  private final GooseRotationSubsystem m_subsystem;
  private XboxController m_joystick;

  /**
   * Creates a new ManualManualRotateCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualRotateCommand(GooseRotationSubsystem subsystem, XboxController joystick) {
    m_subsystem = subsystem;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpeed(-m_joystick.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_subsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
