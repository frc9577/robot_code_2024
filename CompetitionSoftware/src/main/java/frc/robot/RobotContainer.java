// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.*;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousDriveTimed;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.NoteHandlingSpeedCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.SetGearCommand;
import frc.robot.commands.SetModeCommand;
import frc.robot.commands.TimedCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GooseRotationSubsystem;
import frc.robot.subsystems.NoteHandlingSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final PneumaticHub   m_pnuematicHub   = new PneumaticHub();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final GooseRotationSubsystem m_rotationSubsystem    = new GooseRotationSubsystem();
  private final NoteHandlingSubsystem m_noteHandlingSubsystem = new NoteHandlingSubsystem();

  // Joysticks
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorController);
  private final Joystick m_driverJoystick = new Joystick(DriverConstants.kDriveJoystick);

  // Driver Buttons
  public final JoystickButton m_lowGearButton =
    new JoystickButton(m_driverJoystick, DriverConstants.kLowGear);

  public final JoystickButton m_forwardModeButton = 
    new JoystickButton(m_driverJoystick, DriverConstants.kForwardMode);
  public final JoystickButton m_reverseModeButton = 
    new JoystickButton(m_driverJoystick, DriverConstants.kReverseMode);

  // Pnuematics Climb Buttons
  private final JoystickButton m_climbUpButton =
    new JoystickButton(m_operatorController, OperatorConstants.kClimbUp);
  private final JoystickButton m_climbDownButton =
    new JoystickButton(m_operatorController, OperatorConstants.kClimbDown);

  // Note Handling Buttons
  //private final JoystickButton m_intakeAutoButton = 
    //new JoystickButton(m_operatorController, OperatorConstants.kIntakeAuto);
  private final JoystickButton m_outputButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kOutput);
  private final JoystickButton m_intakeManualButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kIntakeManual);
  private final JoystickButton m_fullStopButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kFullStop);
  private final JoystickButton m_SpitOutButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kSpitOut);

  // Goose Rotation Buttons
  private final JoystickButton m_moveArmTopButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kMoveArmTop);
  private final JoystickButton m_moveArmBottomButton = 
    new JoystickButton(m_operatorController, OperatorConstants.kMoveArmBottom);
  private final JoystickButton m_moveArmMiddleButton =
    new JoystickButton(m_operatorController, OperatorConstants.kMoveArmMiddle);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_pnuematicHub.enableCompressorAnalog(RobotConstants.minPnuematicsPressure,RobotConstants.maxPnuematicsPressure);
    
    // Set up SmartDashboard/Shuffleboard widgets for driver/operator use.
    configureDriverStationControls();

    // Configure the trigger bindings
    configureBindings();

    m_tickCount = 0;
    reportStatus();
  }

  public void reportStatus() {
    if(m_tickCount % (RobotConstants.periodicTicksPerSecond/RobotConstants.pnuematicReportingFreq) == 0)
    {
      // Report pneumatic state
      SmartDashboard.putNumber("Pressure", m_pnuematicHub.getPressure(0));
      SmartDashboard.putBoolean("Compressor Running", m_pnuematicHub.getCompressor());
    }

    if(m_tickCount % (RobotConstants.periodicTicksPerSecond/RobotConstants.gooseReportingFreq) == 0)
    {
      // Report Goose ARM state.
      SmartDashboard.putNumber("Goose Raw Measurement", m_rotationSubsystem.getEncoderValue());
      SmartDashboard.putNumber("Goose Set Point", m_rotationSubsystem.getSetPointAngle());
      SmartDashboard.putNumber("Goose Current Angle", m_rotationSubsystem.getAngle());
    }
    m_tickCount += 1;
  }

  //
  // Configure all options that we want to display on the Shuffleboard dashboard.
  //
  private void configureDriverStationControls()
  {
    // Drop-down chooser for auto program.
    m_autoChooser.setDefaultOption("Pass Auto Line", new AutonomousDriveTimed(m_driveSubsystem, AutoConstants.kPassLineDuration_mS));
    m_autoChooser.addOption("Drive forward 2 Seconds", new AutonomousDriveTimed(m_driveSubsystem, 2000));
    m_autoChooser.addOption("Drive forward 4 Seconds", new AutonomousDriveTimed(m_driveSubsystem, 4000));
    m_autoChooser.addOption("Drive forward 6 Seconds", new AutonomousDriveTimed(m_driveSubsystem, 6000));

    AutonomousDrive autoDrive = new AutonomousDrive(m_driveSubsystem);
    m_autoChooser.addOption("Drive Forward 2 Seconds !! NOT TESTED !!", new TimedCommand(autoDrive, 2000));

    SmartDashboard.putData(m_autoChooser);
  }

  private void configureBindings() 
  {
    // Drive Controllers. We set arcade drive as the default here but may change this
    // during teleopInit depending upon the value of a dashboard chooser.
    m_lowGearButton.onTrue(new SetGearCommand(m_driveSubsystem, true));
    m_lowGearButton.onFalse(new SetGearCommand(m_driveSubsystem, false));

    m_forwardModeButton.onTrue(new SetModeCommand(m_driveSubsystem, false));
    m_reverseModeButton.onTrue(new SetModeCommand(m_driveSubsystem, true));

    // Climb Buttons
    m_climbUpButton.onTrue(new ClimbCommand(m_climbSubsystem, ClimbSubsystem.State.LIFTED));
    m_climbDownButton.onTrue(new ClimbCommand(m_climbSubsystem, ClimbSubsystem.State.GROUNDED));

    // Note handling Buttons
    // m_intakeAutoButton.onTrue(new AutoNoteIntakeCommand(m_noteHandlingSubsystem, 
    //                                                    NoteHandlingConstants.kIntakeSpeed));

   m_intakeManualButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 
                                                            NoteHandlingConstants.kRollerSpeed, true));
    m_intakeManualButton.onFalse(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0, true));
    
    m_SpitOutButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 
                                                        -NoteHandlingConstants.kRollerSpeed, 
                                                        false));
    m_SpitOutButton.onFalse(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 0.0, 
                                                        false));
                                                      

    m_outputButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem, 
                                                       NoteHandlingConstants.kRollerSpeed, false));
    m_outputButton.onFalse(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0, false));

    m_fullStopButton.onTrue(new NoteHandlingSpeedCommand(m_noteHandlingSubsystem,0.0, false));

    // Goose Rotation Buttons
    m_moveArmTopButton.onTrue(new RotateCommand(m_rotationSubsystem, 
                                                GooseRotationConstants.kScoreAngle));
    m_moveArmMiddleButton.onTrue(new RotateCommand(m_rotationSubsystem, 
                                                GooseRotationConstants.kMiddleAngle));
    m_moveArmBottomButton.onTrue(new RotateCommand(m_rotationSubsystem, 
                                                   GooseRotationConstants.kIntakeAngle));

    m_rotationSubsystem.initDefaultCommand(m_operatorController);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  // Set drive command to arcade
  public void setDriveType()
  {
    m_driveSubsystem.initDefaultCommand(m_driverJoystick);
  }
}
