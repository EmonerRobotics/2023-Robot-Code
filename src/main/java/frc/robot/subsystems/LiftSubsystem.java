// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class LiftSubsystem extends SubsystemBase {
  
  private PWMVictorSPX liftMotor;

  private Encoder liftEncoder;

  public LiftSubsystem() {

    //redline motor
    liftMotor = new PWMVictorSPX(IntakeConstants.LiftRedline1);

    //lift encoder
    liftEncoder = new Encoder(6, 7); //true, EncodingType.k4X
    liftEncoder.reset();
  }

  public void manuelLiftControl(double speed){
    liftMotor.set(-speed);
  }

  public double getEncoderMeters(){
    return (liftEncoder.get() * -IntakeConstants.kEncoderTick2Meter);
  }

  public boolean isAtGround(){
    double error = getEncoderMeters() - Constants.LiftMeasurements.GROUNDH;
    return (Math.abs(error) < Constants.LiftMeasurements.LiftAllowedError);
  }

  public boolean isAtMid(){
    double error = getEncoderMeters() - Constants.LiftMeasurements.MIDH;
    return (Math.abs(error) < Constants.LiftMeasurements.LiftAllowedError);
  }

  public boolean isAtTop(){
    double error = getEncoderMeters() - Constants.LiftMeasurements.TOPH;
    return (Math.abs(error) < Constants.LiftMeasurements.LiftAllowedError);
  }

  public boolean isAtHuman(){
    double error = getEncoderMeters() - Constants.LiftMeasurements.HUMANPH;
    return (Math.abs(error) < Constants.LiftMeasurements.LiftAllowedError);
  }

  @Override
  public void periodic() {
    //double liftHeight = liftEncoder.get() / IntakeConstants.degrees2Cm;
    SmartDashboard.putNumber("Meters: ", getEncoderMeters());  
  }
}
