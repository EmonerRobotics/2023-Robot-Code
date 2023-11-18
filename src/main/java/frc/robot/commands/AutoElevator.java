// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class AutoElevator extends CommandBase {

  private final LiftSubsystem liftSubsystem;
  private final PIDController pidController;
  private final ElevatorPosition elevatorPosition;

  public static enum ElevatorPosition{
    GROUND,
    MIDDLE,
    TOP,
    HUMANP
  }

  /** Creates a new AutoElevator. */
  public AutoElevator(LiftSubsystem liftSubsystem, double setPoint, ElevatorPosition elevatorPosition) {
    this.liftSubsystem = liftSubsystem;
    this.elevatorPosition = elevatorPosition;
    this.pidController = new PIDController(2.5, 0.3,0);
    pidController.setSetpoint(setPoint);
    addRequirements(liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("ElevatorPIDCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(liftSubsystem.getEncoderMeters());
    liftSubsystem.manuelLiftControl(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    liftSubsystem.manuelLiftControl(0);
    System.out.println("ElevatorPIDCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      
    if(elevatorPosition == ElevatorPosition.GROUND){
        return this.liftSubsystem.isAtGround();
      
      }else if(elevatorPosition == ElevatorPosition.MIDDLE){
        return this.liftSubsystem.isAtMid();
      
      }else if(elevatorPosition == ElevatorPosition.TOP){
        return this.liftSubsystem.isAtTop();
      
      }else if(elevatorPosition == ElevatorPosition.HUMANP){
        return this.liftSubsystem.isAtHuman();
      
      }else{
        return false;
      }
  }
}
