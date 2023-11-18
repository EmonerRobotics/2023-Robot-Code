// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;
  private final PIDController pidController;  
  private final IntakePosition intakePosition;

  public static enum IntakePosition{
    CLOSED,
    HALF,
    STRAIGHT,
    STRAIGHTHD
  }

  /** Creates a new AutoIntake. */
  public AutoIntake(IntakeSubsystem intakeSubsystem, double setPoint, IntakePosition intakePosition) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakePosition = intakePosition;
    this.pidController = new PIDController(1.2, .3, .1);
    pidController.setSetpoint(setPoint);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("IntakePIDCmd started!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(intakeSubsystem.getEncoderMeters());
    intakeSubsystem.manuelIntakeAngle(-speed * .5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.manuelIntakeAngle(0);
    System.out.println("IntakePIDCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakePosition == IntakePosition.STRAIGHT){
      return this.intakeSubsystem.isAtStraight();

    }else if(intakePosition == IntakePosition.CLOSED){
      return this.intakeSubsystem.isAtClose();

    }else if(intakePosition == IntakePosition.HALF){
      return this.intakeSubsystem.isAtHalf();
    
    }else if(intakePosition == IntakePosition.STRAIGHTHD){
      return this.intakeSubsystem.isAtStraighTHuman();
    }else{
      return false;
    }
  }
}
