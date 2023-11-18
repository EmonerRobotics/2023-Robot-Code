// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimation;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.AllianceUtils;

public class GetInRange extends CommandBase {

  public static enum LimelightPositionCheck{
    fiftyFive,
    oneTwenty
  }

  private final LimelightSubsystem limelightSubsystem;
  private final PIDController pidController;
  private final Drivetrain drivetrain;
  private final PoseEstimation poseEstimation;
  private final LimelightPositionCheck limelightCheck;
  private Rotation2d fieldOrientationZeroOffset = new Rotation2d();
  private double setPoint;
  private double speedY;

  /** Creates a new GetInRange. */
  public GetInRange(Drivetrain drivetrain, PoseEstimation poseEstimation, LimelightSubsystem limelightSubsystem, double setPoint, LimelightPositionCheck limelightCheck) {
    this.drivetrain = drivetrain;
    this.poseEstimation = poseEstimation;
    this.pidController = new PIDController(0.4, 0.05, 0);
    this.limelightCheck = limelightCheck;
    this.setPoint = setPoint;
    //pidController.setSetpoint(setPoint);
    this.limelightSubsystem = limelightSubsystem;
    drivetrain.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    System.out.println("GetInRangePIDCommand Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(limelightSubsystem.hasTargets()){
      
      double distanceError = (limelightSubsystem.EstimatedDistance() - setPoint);
      SmartDashboard.putNumber("DistanceError: ", distanceError);
      double speedX = 0.04 * distanceError;

      if(limelightSubsystem.getId() == 4 || limelightSubsystem.getId() == 5){
        Constants.LimelightConstants.goalHeightInches = 30.31496; //26.37795
      }else{
        Constants.LimelightConstants.goalHeightInches = 21.25984; //21.25984  
      }

      if(Math.abs(limelightSubsystem.getAim()) > -18){
        if(limelightSubsystem.getAim() < -18){
          speedY = 0.04 * limelightSubsystem.getAim() + 0.01;
        }else{
          speedY = 0.04 * limelightSubsystem.getAim() - 0.01;
        }
      }
      
      //double speed = pidController.calculate(limelightSubsystem.EstimatedDistance());
      ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(-speedX, 0, 0);
      ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelSpeeds, poseEstimation.getEstimatedPose().getRotation().minus(AllianceUtils.getFieldOrientationZero().plus(fieldOrientationZeroOffset)));
      drivetrain.drive(robotRelSpeeds);
      System.out.println("Getting in range: " + speedX);
      System.out.println(Constants.LimelightConstants.goalHeightInches);
      
    }else{
      System.out.println("NO TARGET");
    } 
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds fieldRelSpeeds = new ChassisSpeeds(0, 0, 0);
    System.out.println("GetInRangePIDCommand Finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(limelightCheck == LimelightPositionCheck.fiftyFive){
      return this.limelightSubsystem.isAtDistance0();
    }else if(limelightCheck == LimelightPositionCheck.oneTwenty){
      return this.limelightSubsystem.isAtDistance1();
    }else{
      return false;
    }
  }
}
