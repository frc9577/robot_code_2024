// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.DoubleSolenoidSetCommand;
//import frc.robot.commands.MotorConstSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private int m_tickCount = 0;

  // The robot's subsystems and commands are defined here...
  private final PneumaticHub   m_pnuematicHub   = new PneumaticHub();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  // Joystick
  private final Joystick m_joystick = new Joystick(OperatorConstants.kWinchJoystick);

  // Pnuematics Climb Buttons
  private final JoystickButton m_climbUpButton =
    new JoystickButton(m_joystick, OperatorConstants.kClimbUp);
  private final JoystickButton m_climbDownButton =
    new JoystickButton(m_joystick, OperatorConstants.kClimbDown);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pnuematicHub.enableCompressorAnalog(RobotConstants.minPnuematicsPressure,RobotConstants.maxPnuematicsPressure);

    // Configure the trigger bindings
    configureBindings();
  }

  public void reportStatus() {
    if(m_tickCount % (RobotConstants.periodicTicksPerSecond/RobotConstants.pnuematicReportingFreq) == 0)
    {
      // Report pneumatic state
      SmartDashboard.putNumber("Pressure", m_pnuematicHub.getPressure(0));
      SmartDashboard.putBoolean("Compressor Running", m_pnuematicHub.getCompressor());
    }

    m_tickCount += 1;
  }

  private void configureBindings() 
  {
    // Pnuematics Climb Buttons
    m_climbUpButton.onTrue(new DoubleSolenoidSetCommand(m_climbSubsystem, 
                                                        m_climbSubsystem.getDoubleSolenoid(), 
                                                        DoubleSolenoid.Value.kForward));
    m_climbUpButton.onFalse(new DoubleSolenoidSetCommand(m_climbSubsystem, 
                                                         m_climbSubsystem.getDoubleSolenoid(), 
                                                         DoubleSolenoid.Value.kOff));

    m_climbDownButton.onTrue(new DoubleSolenoidSetCommand(m_climbSubsystem, 
                                                          m_climbSubsystem.getDoubleSolenoid(),
                                                          DoubleSolenoid.Value.kReverse));
    m_climbDownButton.onFalse(new DoubleSolenoidSetCommand(m_climbSubsystem, 
                                                           m_climbSubsystem.getDoubleSolenoid(), 
                                                           DoubleSolenoid.Value.kOff));

    // TODO: Add bindings for drive subsystem.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Do nothing in auto.
    return null;
  }
}
