// This is one of the default commands for drive subsystem and allows control
// using a single joystick configured for arcade drive (Y axis controls speed,
// yaw/twist axis controls turn rate).

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommand extends Command {
  private final DriveSubsystem m_subsystem;
  private Joystick m_Joystick;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(DriveSubsystem subsystem, Joystick driveJoystick) 
  {
    m_subsystem = subsystem;
    m_Joystick = driveJoystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Note: We negate the Y axis value so that pushing the joystick forwards
    // (which makes the readin more negative) increases the speed. The twist
    // axis is already correct, increasing clockwise, so we don't need to negate
    // it.
    m_subsystem.setArcadeSpeeds(-m_Joystick.getY(), m_Joystick.getTwist());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
