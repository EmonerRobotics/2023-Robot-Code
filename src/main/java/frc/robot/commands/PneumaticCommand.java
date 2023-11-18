// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticCommand extends CommandBase {

  private final PneumaticSubsystem pneumaticSubsystem;
  private final boolean SolenoidOn;
  private final boolean SolenoidOff;
  /** Creates a new PneumaticCommand. */
  public PneumaticCommand(PneumaticSubsystem pneumaticSubsystem, boolean SolenoidOn, boolean SolenoidOff) {
    this.pneumaticSubsystem = pneumaticSubsystem;
    this.SolenoidOn = SolenoidOn;
    this.SolenoidOff = SolenoidOff;
    addRequirements(pneumaticSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumaticSubsystem.SolenoidForward(SolenoidOn);
    pneumaticSubsystem.SolenoidReverse(SolenoidOff);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(SolenoidOff){
      return true;
    }else if(SolenoidOn){
      return true;
    }else{
      return false;
    }
  }
}
