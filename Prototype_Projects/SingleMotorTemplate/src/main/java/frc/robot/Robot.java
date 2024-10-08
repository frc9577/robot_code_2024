// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 */
public class Robot extends TimedRobot {
  private static final int kMotorID      = 10;
  private static final int kJoystickPort = 0;

  private CANSparkMax m_motor;
  private Joystick m_joystick;

  @Override
  public void robotInit() {
    m_motor = new CANSparkMax(kMotorID);
    m_joystick = new Joystick(kJoystickPort);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    // Put SmartDashboard stuff here.
  }

  @Override
  public void teleopPeriodic() {
    m_motor.set(-m_joystick.getY());
  }
}
