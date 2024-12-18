// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class RobotConstants {
    public static final int periodicTicksPerSecond = 50;
    public static final int intakeReportingFreq = 1;
  }

  public static class DrivetrainConstants {
    public static final int kLeftMotorCANID  = 10;
    public static final int kRightMotorCANID = 20;
  }

  public static class DriverConstants {
    public static final int kDriveJoystick = 0;
    public static final double kTurnDivider = 2.0;

    // Buttons
    public static final int kIntake = 1; // Joystick Trigger
    public static final double kThrottleMultiplier = -1.0;
  }

  public static class OperatorConstants {
    public static final int kOperatorController = 1;
    
    public static final int kRunForwardButton = 4; // Button Y
    public static final int kRunBackwardButton = 3;  // Button X
  }

  public static class WheeledIntakeConstants {
    public static final int kLeftMotorCANID = 30;
    public static final int kRightMotorCANID = 40;

    public static final int kLeftMotorCurrentLimit = 3;
    public static final int kRightMotorCurrentLimit = 3;

    public static final double kRollerSpeed = -0.6; // "random" value
  }

  public static class ClawdiaConstants {
    public static final int kMotorCANID = 50;

    public static final int kMotorCurrentLimit = 3;

    public static final double kClawSpeed = 0.1; // low value for testing
  }
}