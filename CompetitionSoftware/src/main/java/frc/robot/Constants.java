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
    public static final double minPnuematicsPressure = 80.0;
    public static final double maxPnuematicsPressure = 120.0;

    public static final int periodicTicksPerSecond = 50;
    public static final int pnuematicReportingFreq = 1;
  }

  public static class DrivetrainConstants {
    public static final int kLeftFrontMotorCANID  = 10;
    public static final int kLeftBackMotorCANID   = 11;
    public static final int kRightFrontMotorCANID = 20;
    public static final int kRightBackMotorCANID  = 21;
  }

  public static class DriverConstants {
    public static final int kLeftDriveJoystick = 0;
    public static final int kRightDriveJoystick = 1;
  }

  public static class OperatorConstants {
    public static final int kOperatorJoystick = 2;

    public static final int kClimbUp = 5;
    public static final int kClimbDown = 3;
  }

  public static class WinchConstants {
    public static final int kMotorCANid   = 50;

    public static final double kSpeedUp   = 0.25;
    public static final double kSpeedDown = -0.25;
    public static final double kSpeedStop = 0.0;
  }

  public static class ClimbConstants {
    public static final int kPneumaticsHubCANid = 1;

    // Temp Values, Needs to be changed when solunoid is set up onto the robot
    public static final int kExtendChannelL = 1;
    public static final int kRetractChannelL = 2;
    
    public static final int kExtendChannelR = 3;
    public static final int kRetractChannelR = 4;
  }
}
