// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
  
  private Compressor compressor;
  private DoubleSolenoid solenoid;
  private boolean statement = true;

  /** Creates a new PneumaticSubsystem. */
  public PneumaticSubsystem() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 8);
  }

  public void SolenoidForward(boolean SolenoidOn){
    if(SolenoidOn){
      solenoid.set(Value.kForward);
      statement = true;
    }
  }

  public void SolenoidReverse(boolean SolenoidOff){
    if(SolenoidOff){
      solenoid.set(Value.kReverse);
      statement = false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Piston", statement);
    compressor.enableDigital();
  }
}
