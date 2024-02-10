// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.*;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.NoteHandlingSpeedCommand;
//import frc.robot.commands.MotorConstSpeedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GooseRotationSubsystem;
import frc.robot.subsystems.NoteHandlingSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
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
  private final GooseRotationSubsystem m_rotationSubsystem    = new GooseRotationSubsystem();
  private final NoteHandlingSubsystem m_noteHandlingSubsystem = new NoteHandlingSubsystem();

  // Joysticks
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorController);
  private final Joystick m_driverJoystickLeft = new Joystick(DriverConstants.kLeftDriveJoystick);
  private final Joystick m_driverJoystickRight = new Joystick(DriverConstants.kRightDriveJoystick);

  // Pnuematics Climb Buttons
  private final JoystickButton m_climbUpButton =
    new JoystickButton(m_operatorController, OperatorConstants.kClimbUp);
  private final JoystickButton m_climbDownButton =
    new JoystickButton(m_operatorController, OperatorConstants.kClimbDown);

  // Note Handling Buttons
  private final JoystickButton m_intakeAutoButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kIntakeAuto);
  private final JoystickButton m_outputButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kOutput);
  private final JoystickButton m_intakeManualButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kIntakeManual);
  private final JoystickButton m_fullStopButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kFullStop);

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
    // Drive Controllers
    m_driveSubsystem.initDefaultCommand(m_driverJoystickLeft, m_driverJoystickRight);

    // Climb Buttons
    m_climbUpButton.onTrue(new ClimbCommand(m_climbSubsystem, ClimbSubsystem.State.LIFTED));
    m_climbDownButton.onTrue(new ClimbCommand(m_climbSubsystem, ClimbSubsystem.State.GROUNDED));

    // Note handling Buttons
    m_intakeManualButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 
                                                             NoteHandlingConstants.kIntakeSpeed));
    m_intakeManualButton.onFalse(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0));
    
    m_outputButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 
                                                       NoteHandlingConstants.kOutputSpeed));
    m_outputButton.onFalse(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0));

    m_fullStopButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0));

    // TODO: Bind auto intake button
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
