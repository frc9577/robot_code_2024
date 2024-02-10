//
// A basic tank drive subsystem class using 2 Kraken/TalonFX motors
// per side.
//
package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class DriveSubsystem extends SubsystemBase
{
  private final TalonFX m_leftFrontMotor  = new TalonFX(DrivetrainConstants.kLeftFrontMotorCANID);
  private final TalonFX m_leftBackMotor   = new TalonFX(DrivetrainConstants.kLeftBackMotorCANID);
  private final TalonFX m_rightFrontMotor = new TalonFX(DrivetrainConstants.kRightFrontMotorCANID);
  private final TalonFX m_rightBackMotor  = new TalonFX(DrivetrainConstants.kRightBackMotorCANID);
  private DifferentialDrive m_Drivetrain;
  private double m_leftSpeed  = 0.0;
  private double m_rightSpeed = 0.0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem()
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    
    // NOTE: Invert needs to be done for both motors BEFORE follow
    m_rightFrontMotor.setInverted(true);
    m_rightBackMotor.setInverted(true);

    /* Set back motors to follow the front motors. */
    m_leftBackMotor.setControl(new Follower(m_leftFrontMotor.getDeviceID(), false));
    m_rightBackMotor.setControl(new Follower(m_rightFrontMotor.getDeviceID(), false));

    m_Drivetrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
  }

  public void initDefaultCommand(Joystick leftJoystick, Joystick rightJoystick)
  {
    setDefaultCommand(new DriveCommand(this, leftJoystick, rightJoystick));
  }

  public void SetSpeeds(double leftInput, double rightInput)
  {
    m_leftSpeed = leftInput;
    m_rightSpeed = rightInput;

    // NOTE: We are squaring the input to improve driver response
    m_Drivetrain.tankDrive(leftInput, rightInput, true);
  }

  public double GetSpeed(boolean bLeft)
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
