// This is one of the default commands for drive subsystem and allows control
// using a single joystick configured for arcade drive (Y axis controls speed,
// yaw/twist axis controls turn rate).

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/** An example command that uses an example subsystem. */
public class ArcadeDriveCommand extends Command {
  private final Drive m_subsystem;
  private XboxController m_Joystick;

  /**
   * Creates a new DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDriveCommand(Drive subsystem, XboxController driveJoystick) 
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
    // Note: We negate both axis values so that pushing the joystick forwards
    // (which makes the readin more negative) increases the speed and twisting clockwise
    // turns the robot clockwise.
    m_subsystem.arcadeDrive(-m_Joystick.getLeftY(), -m_Joystick.getRightX());
    m_subsystem.outputDistance();
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
