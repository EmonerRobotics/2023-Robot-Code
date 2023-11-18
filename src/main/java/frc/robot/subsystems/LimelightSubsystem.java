// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    EstimatedDistance();
    SmartDashboard.putBoolean("Found Target", hasTargets());
  }

  public double getAim(){
    NetworkTableEntry tx = table.getEntry("tx");
    double headingError = tx.getDouble(0.0);
    return headingError;
  }

  public double EstimatedDistance(){
    
    NetworkTableEntry ty = table.getEntry("ty");
    
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 28;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 10.62992; //18.58268
    
    // distance from the target to the floor
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
    //calculate distance
    double distanceFromLimelightToGoalInches = ((Constants.LimelightConstants.goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians));
    SmartDashboard.putNumber("DISTANCE: ", distanceFromLimelightToGoalInches);
    return distanceFromLimelightToGoalInches;
  }

  public double getId(){
    NetworkTableEntry tid = table.getEntry("tid");
    double tId = tid.getDouble(0.0);
    return tId;
  }

  public boolean hasTargets(){
    NetworkTableEntry tv = table.getEntry("tv");
    float isTrue = tv.getFloat(0);  
    return (isTrue == 0.0f) ? false : true;
  }

  public boolean isAtDistance0(){
    double error = EstimatedDistance() - 21.65;
    return (Math.abs(error) < 0.5);
  }

  public boolean isAtDistance1(){
    double error = EstimatedDistance() - 120;
    return (Math.abs(error) < 0.5);
  }
}
