// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final TalonFX m_leftLeader = new TalonFX(DriveConstants.kLeftMotor1Port);

  // The motors on the right side of the drive.
  private final TalonFX m_rightLeader = new TalonFX(DriveConstants.kRightMotor1Port);
  
  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  // The gyro sensor. NOTE: we discovered experimentally that SPI and UART do not work
  private final AHRS m_gyro = new AHRS(I2C.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLeader.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = DriveConstants.kDistancePerTalonRotation;

    resetEncoders();
    double leftPosition = m_leftLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    double rightPosition = m_rightLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), leftPosition, rightPosition);
  }

  @Override
  public void periodic() {
    double leftPosition = m_leftLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    double leftVelocity = m_leftLeader.getVelocity().getValue() * DriveConstants.kDistancePerTalonRotation;

    double rightPosition = m_rightLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    double rightVelocity = m_rightLeader.getVelocity().getValue() * DriveConstants.kDistancePerTalonRotation;

    double gyroRotation = m_gyro.getAngle();

    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), leftPosition, rightPosition);

    // Update Smart Dashbord
    SmartDashboard.putNumber("gyroRotation", gyroRotation);
    SmartDashboard.putNumber("leftPosition", leftPosition);
    SmartDashboard.putNumber("leftVelocity", leftVelocity);

    SmartDashboard.putNumber("rightPosition", rightPosition);
    SmartDashboard.putNumber("rightVelocity", rightVelocity);
    
    SmartDashboard.putNumber("gyroUpdates", m_gyro.getUpdateCount());
    SmartDashboard.putBoolean("gyroConnected", m_gyro.isConnected());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftVelocity = m_leftLeader.getVelocity().getValue() * DriveConstants.kDistancePerTalonRotation;
    double rightVelocity = m_rightLeader.getVelocity().getValue() * DriveConstants.kDistancePerTalonRotation;

    return new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    double leftPosition = m_leftLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    double rightPosition = m_rightLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), leftPosition, rightPosition, pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot, boolean squared) {
    m_drive.arcadeDrive(fwd, rot, squared);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // todo: write this please
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    double leftPosition = m_leftLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;
    double rightPosition = m_rightLeader.getPosition().getValue() * DriveConstants.kDistancePerTalonRotation;

    return (leftPosition + rightPosition) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
