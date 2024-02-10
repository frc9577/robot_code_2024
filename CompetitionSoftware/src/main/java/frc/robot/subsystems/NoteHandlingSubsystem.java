// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteHandlingSubsystem extends SubsystemBase {
  private double m_intakeSpeed = 0;
  private double m_outputSpeed = 0;

  /** Creates a new NoteHandlingSubsystem. */  
  public NoteHandlingSubsystem() {}

  public void setIntakeSpeed(double speed)
  {
    // TODO: Sets intake motor speed.
    m_intakeSpeed = speed;
  }
  
  // Returns the last COMMANDED speed
  public double getIntakeSpeed()
  {
    return m_intakeSpeed;
  }

  public void setOutputSpeed(double speed)
  {
    // TODO: Sets intake motor speed.
    m_outputSpeed = speed;
  }

  // Returns the last COMMANDED speed.
  public double getOutputSpeed()
  {
    return m_outputSpeed;
  }

  public boolean hasNote() // Gets sensor result if it has the note and then and returns it
  {
    boolean result = false;
    // TODO: Reads sensor, When its not default return true.

    return result;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() 
  {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() 
  {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
