// This is the default auto command that drives the robot across the auto line.

package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GooseRotationSubsystem;

/** An example command that uses an example subsystem. */
public class AutonomousDriveArmSet extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final GooseRotationSubsystem m_rotationSubsystem;
  private double m_leftSpeed = 0.0;
  private double m_rightSpeed = 0.0;
  private long m_startTime;
  private int m_ms;

  /**
   * Creates a new AutonomousDriveArmUp.
   *
   * @param subsystem The subsystem used by this command.
   */
  // Takes seconds not MS.
  public AutonomousDriveArmSet(DriveSubsystem driveSubsystem, GooseRotationSubsystem rotationSubsystem, int seconds) 
  {
    m_driveSubsystem = driveSubsystem;
    m_rotationSubsystem = rotationSubsystem;
    m_ms = seconds*1000; // seconds to MS

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_rotationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
    m_driveSubsystem.setTankSpeeds(0.0, 0.0);

    m_leftSpeed = AutoConstants.kPassLineSpeed;
    m_rightSpeed = AutoConstants.kPassLineSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setTankSpeeds(m_leftSpeed, m_rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setTankSpeeds(0.0, 0.0);

    if (interrupted == false)
    {
      new CompoundRotateArmCommand(m_rotationSubsystem).execute();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime) >= m_ms;
  }
}
