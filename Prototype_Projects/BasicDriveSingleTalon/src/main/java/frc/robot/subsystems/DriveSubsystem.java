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

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DriveSubsystem extends SubsystemBase
{
  private final TalonFX m_leftMotor  = new TalonFX(DrivetrainConstants.kLeftMotorCANID);
  private final TalonFX m_rightMotor = new TalonFX(DrivetrainConstants.kRightMotorCANID);
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
    m_rightMotor.setInverted(true);

    // Set all motors to brake mode for safety. In the default, coast mode,
    // the robot takes very much longer to stop if the joystick is released or
    // and emergency stop occurs. In this mode, it stops very quickly.
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightMotor.setNeutralMode(NeutralModeValue.Brake);

    m_Drivetrain = new DifferentialDrive(m_leftMotor, m_rightMotor);
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