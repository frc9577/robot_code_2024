// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final WPI_VictorSPX m_leftFrontMotor  = new WPI_VictorSPX(10);
  private final WPI_VictorSPX m_leftBackMotor   = new WPI_VictorSPX(11);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(20);
  private final WPI_VictorSPX m_rightBackMotor  = new WPI_VictorSPX(21);


  
  /** Constants that define the settings of the driver camera */
  public static final int kDriverCameraResolutionX = 640;
  public static final int kDriverCameraResolutionY = 360;
  public static final int kDriverCameraFPS         = 15;

  private UsbCamera driverCamera;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    // NOTE: Invert needs to be done for both motors BEFORE follow
    m_rightFrontMotor.setInverted(true);
    m_rightBackMotor.setInverted(true);

    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    m_myRobot = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    // Start the driver camera streaming.
    driverCamera = CameraServer.startAutomaticCapture("Driver Camera", 0);
    driverCamera.setResolution(kDriverCameraResolutionX, kDriverCameraResolutionY);
    driverCamera.setFPS(kDriverCameraFPS);
    driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
