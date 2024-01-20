// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.MotorConstSpeedCommand;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final WinchSubsystem m_winchSubsystem = new WinchSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_joystick = new Joystick(OperatorConstants.kWinchJoystick);
  private final JoystickButton m_winchUpButton =
      new JoystickButton(m_joystick,OperatorConstants.kWinchUp);
  private final JoystickButton m_winchDownButton =
      new JoystickButton(m_joystick,OperatorConstants.kWinchDown);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // The winch up command and winch down command triggers.

    m_winchUpButton.onTrue(new MotorConstSpeedCommand(m_winchSubsystem, 
                                                      m_winchSubsystem.getMotorController(), 
                                                      WinchConstants.kSpeedUp));
    m_winchUpButton.onFalse(new MotorConstSpeedCommand(m_winchSubsystem, 
                                                       m_winchSubsystem.getMotorController(), 
                                                       WinchConstants.kSpeedStop));

    m_winchDownButton.onTrue(new MotorConstSpeedCommand(m_winchSubsystem, 
                                                      m_winchSubsystem.getMotorController(), 
                                                      WinchConstants.kSpeedDown));
    m_winchDownButton.onFalse(new MotorConstSpeedCommand(m_winchSubsystem, 
                                                       m_winchSubsystem.getMotorController(), 
                                                       WinchConstants.kSpeedStop));
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
