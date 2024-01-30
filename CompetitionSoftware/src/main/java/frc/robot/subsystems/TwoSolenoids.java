package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class TwoSolenoids {
    public final DoubleSolenoid left;
    public final DoubleSolenoid right;
    public TwoSolenoids(DoubleSolenoid l, DoubleSolenoid r)
    {
      left = l;
      right = r;
    }
  }