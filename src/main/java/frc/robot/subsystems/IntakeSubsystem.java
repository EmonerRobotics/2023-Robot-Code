// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax neoMotor;

  private RelativeEncoder m_angleEncoder;

  
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    neoMotor = new CANSparkMax(IntakeConstants.AngleMechanismId, MotorType.kBrushless);
    neoMotor.setSmartCurrentLimit(20);
    m_angleEncoder = neoMotor.getEncoder();
    
  }

  public void manuelIntakeAngle(double controller){
    neoMotor.set(controller * .5);
  }

  public double getEncoderMeters() {
    return m_angleEncoder.getPosition() * -0.1; 

  }

  public boolean isAtClose(){
    double error = getEncoderMeters() - Constants.IntakeMeasurements.IntakeClosedD;
    return (Math.abs(error) < Constants.IntakeMeasurements.IntakeAllowedError);
  }

  public boolean isAtHalf(){
    double error = getEncoderMeters() - Constants.IntakeMeasurements.IntakeHalfOpenD;
    return (Math.abs(error) < Constants.IntakeMeasurements.IntakeAllowedError);
  }

  public boolean isAtStraight(){
    double error = getEncoderMeters() - Constants.IntakeMeasurements.IntakeStraightOpenD;
    return (Math.abs(error) < Constants.IntakeMeasurements.IntakeAllowedError);
  }

  public boolean isAtStraighTHuman(){
    double error = getEncoderMeters() - Constants.IntakeMeasurements.IntakeStraightOpenHD;
    return (Math.abs(error) < Constants.IntakeMeasurements.IntakeAllowedError);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Degrees: ", getEncoderMeters());  
  }
}
