// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.ClawdiaSpeedCommand;
import frc.robot.commands.WheeledIntakeSpeedCommand;
import frc.robot.subsystems.ClawdiaSubsystem;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WheeledIntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private int m_tickCount = 0;

    // The robot's subsystems are defined here
    //private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    //private final WheeledIntakeSubsystem m_wheeledIntakeSubsystem = new WheeledIntakeSubsystem();
    private final ClawdiaSubsystem m_clawdiaSubsystem = new ClawdiaSubsystem(); 

    // TODO: Control Speed via Slider axis 3 (need testing)

    // Joysticks
    //private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorController);
    //private final Joystick m_driverJoystick = new Joystick(DriverConstants.kDriveJoystick);

    // Joystick Buttons
    //public final JoystickButton m_intakeButton =
    //    new JoystickButton(m_driverJoystick, DriverConstants.kIntake);

    // Controllers
    private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorController);

    // Controller Buttons
    public final JoystickButton m_runForwardButton =
        new JoystickButton(m_operatorController, OperatorConstants.kRunForwardButton);
    public final JoystickButton m_runBackwardButton =
        new JoystickButton(m_operatorController, OperatorConstants.kRunBackwardButton);

    // Joystick Values
    public Double m_driverThrottle = 0.0;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        m_tickCount = 0;
        periodicFunction();
    }

    public void periodicFunction() {
        // Updating Values
        //m_driverThrottle = m_driverJoystick.getThrottle() * DriverConstants.kThrottleMultiplier;

        // Reporting Status'
        //if(m_tickCount % (RobotConstants.periodicTicksPerSecond/RobotConstants.intakeReportingFreq) == 0)
        //{
        //    SmartDashboard.putNumber("Intake Speed", m_driverThrottle);
        //}
        m_tickCount += 1;
    }

    // The function that connects the buttons to commands / subsystems.
    private void configureBindings() 
    {
        // Wheeled Intake Bindings
        // inversing the throttle so up = posive and down = negitive
        //m_intakeButton.onTrue(new WheeledIntakeSpeedCommand(m_wheeledIntakeSubsystem, m_driverThrottle));
        //m_intakeButton.onFalse(new WheeledIntakeSpeedCommand(m_wheeledIntakeSubsystem, 0));

        // Clawdia Bindings
        m_runForwardButton.onTrue(new ClawdiaSpeedCommand(m_clawdiaSubsystem, ClawdiaConstants.kClawSpeed));
        m_runForwardButton.onFalse(new ClawdiaSpeedCommand(m_clawdiaSubsystem, 0));

        m_runBackwardButton.onTrue(new ClawdiaSpeedCommand(m_clawdiaSubsystem, -ClawdiaConstants.kClawSpeed));
        m_runBackwardButton.onFalse(new ClawdiaSpeedCommand(m_clawdiaSubsystem, 0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    // Set drive command to arcade
    public void setDriveType()
    {
        //m_driveSubsystem.initDefaultCommand(m_driverJoystick);
    }
}
