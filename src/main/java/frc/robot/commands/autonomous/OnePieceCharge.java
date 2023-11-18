// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoElevator;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.PneumaticCommand;
import frc.robot.commands.AutoElevator.ElevatorPosition;
import frc.robot.commands.AutoIntake.IntakePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceCharge extends SequentialCommandGroup {
  /** Creates a new OnePieceCharge. */
  public OnePieceCharge() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoElevator(RobotContainer.getLiftSubsystem(), Constants.LiftMeasurements.TOPH, ElevatorPosition.TOP),
      new AutoIntake(RobotContainer.getIntakeSubsystem(), Constants.IntakeMeasurements.IntakeStraightOpenD, IntakePosition.STRAIGHT),
      new WaitCommand(1.5),
      new PneumaticCommand(RobotContainer.getPneumaticSubsystem(), true, false),
      new AutoIntake(RobotContainer.getIntakeSubsystem(), Constants.IntakeMeasurements.IntakeClosedD, IntakePosition.CLOSED),
      new AutoElevator(RobotContainer.getLiftSubsystem(), Constants.LiftMeasurements.GROUNDH, ElevatorPosition.GROUND)
    );
  }
}
