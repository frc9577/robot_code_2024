// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    // Joysticks
    //private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorController);
    private final Joystick m_driverJoystick = new Joystick(DriverConstants.kDriveJoystick);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    // The function that connects the buttons to commands / subsystems.
    private void configureBindings() 
    {

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
        m_driveSubsystem.initDefaultCommand(m_driverJoystick);
    }
}
