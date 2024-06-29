// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This is a simple program supporting a basic drivetrain powered by a single Kraken (TalonFX)
 * on each side. The drivetrain is controlled via tank drive using two joysticks.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;

  private final TalonFX m_leftMotor  = new TalonFX(10);
  private final TalonFX m_rightMotor = new TalonFX(20);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_leftStick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-m_leftStick.getY(), -(m_leftStick.getTwist()/2.0), true);
  }
}
