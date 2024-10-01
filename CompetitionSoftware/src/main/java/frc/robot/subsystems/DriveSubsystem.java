//
// A basic tank drive subsystem class using 2 Kraken/TalonFX motors
// per side.
//
package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.commands.ArcadeDriveCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DriveSubsystem extends SubsystemBase
{
  private final TalonFX m_leftFrontMotor  = new TalonFX(DrivetrainConstants.kLeftFrontMotorCANID);
  private final TalonFX m_leftBackMotor   = new TalonFX(DrivetrainConstants.kLeftBackMotorCANID);
  private final TalonFX m_rightFrontMotor = new TalonFX(DrivetrainConstants.kRightFrontMotorCANID);
  private final TalonFX m_rightBackMotor  = new TalonFX(DrivetrainConstants.kRightBackMotorCANID);
  private DifferentialDrive m_Drivetrain;
  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;
  private double m_speedDivider = 2.0; // Default 1.0
  private double m_modeMultiplier = 1.0;
  private boolean m_driveStraight = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem()
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    // NOTE: Invert needs to be done for both motors BEFORE follow
    m_rightFrontMotor.setInverted(true);
    m_rightBackMotor.setInverted(true);

    // Set all motors to brake mode for safety. In the default, coast mode,
    // the robot takes very much longer to stop if the joystick is released or
    // and emergency stop occurs. In this mode, it stops very quickly.
    m_leftFrontMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftBackMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightFrontMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightBackMotor.setNeutralMode(NeutralModeValue.Brake);

    /* Set back motors to follow the front motors. */
    m_leftBackMotor.setControl(new Follower(m_leftFrontMotor.getDeviceID(), false));
    m_rightBackMotor.setControl(new Follower(m_rightFrontMotor.getDeviceID(), false));

    m_Drivetrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
  }

  public void initDefaultCommand(Joystick leftJoystick)
  {
    setDefaultCommand(new ArcadeDriveCommand(this, leftJoystick));
  }

  // Sets left and right motors to set speeds to support tank drive models.
  // rightInput is ignored when straight mode is enabled.
  public void setTankSpeeds(double leftInput, double rightInput)
  {
    m_leftSpeed = (leftInput / m_speedDivider) * m_modeMultiplier;
    m_rightSpeed = m_driveStraight ? m_leftSpeed : (rightInput / m_speedDivider) * m_modeMultiplier;

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.tankDrive(m_leftSpeed, m_rightSpeed, true);
  }

  public void setArcadeSpeeds(double speed, double rotation)
  {
    m_leftSpeed = (speed / m_speedDivider) * m_modeMultiplier;
    m_rightSpeed = rotation; // NOTE: Deliberately did not slow down in low gear.

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.arcadeDrive(m_leftSpeed, m_rightSpeed, true);
  }
  
  // Runs robot into slower mode for a higher precision driving.
  public void setLowGear(boolean lowGear)
  {
    m_speedDivider = lowGear ? DriverConstants.kLowGearDivider : 1.0;
    SmartDashboard.putBoolean("Low Gear", lowGear);
  }

  // Changes controls to normal or reverse mode's
  public void setReverseMode(boolean reverseMode)
  {
    m_modeMultiplier = reverseMode ? -1.0 : 1.0;
    SmartDashboard.putBoolean("Reverse Mode", reverseMode);
  }

  public void setStraightDriveMode(boolean straightDriveMode)
  {
    m_driveStraight = straightDriveMode;
    SmartDashboard.putBoolean("Drive Straight", straightDriveMode);
  }

  public double getSpeed(boolean bLeft)
  {
    if (bLeft)
    {
      return m_leftSpeed;
    }
    else
    {
      return m_rightSpeed;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
