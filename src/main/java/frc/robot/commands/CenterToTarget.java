// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CenterToTarget extends CommandBase {
    private final Drivetrain drivetrain;
    private Rotation2d fieldOrientationZeroOffset = new Rotation2d();
    PhotonCamera camera = new PhotonCamera("photonCam");
    PIDController turnController = new PIDController(0.1, 0, 0);
    public static double center;

  /** Creates a new AimAtTarget. */
  public CenterToTarget(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    drivetrain.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera.getLatestResult();


    if(result.hasTargets()){
      
      center = turnController.calculate(result.getBestTarget().getYaw(), 0);
      //rotation = 1;

      ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, center, 0);

      System.out.println("Centering:" + center);

    }else{
      System.out.println("No Target");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}