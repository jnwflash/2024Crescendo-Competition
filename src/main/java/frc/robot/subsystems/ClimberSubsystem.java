// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.ScoringConstants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  climberMotor1 = new CANSparkMax(ClimberConstants.kClimber1CanId, MotorType.kBrushless);
  climberMotor2 = new CANSparkMax(ClimberConstants.kClimber2CanId, MotorType.kBrushless);

  climberMotor1.restoreFactoryDefaults();
  climberMotor1.setIdleMode(IdleMode.kBrake);
  climberMotor1.burnFlash();
  climberMotor2.restoreFactoryDefaults();
  climberMotor2.setIdleMode(IdleMode.kBrake);
  climberMotor2.burnFlash();
  
  }

  public void climber1Up() {
    //System.out.println("climber1Up run");
    climberMotor1.set(-ClimberConstants.kSpeed);
  }
  public void climber1Stop() {
    //System.out.println("climber1Stop run");
    climberMotor1.set(0);
  }
  public void climber1Down() {
    //System.out.println("climber1Down run");
    climberMotor1.set(ClimberConstants.kSpeed);
  }
  public void climber2Up() {
    //System.out.println("climber2Up run");
    climberMotor2.set(-ClimberConstants.kSpeed);
  }
  public void climber2Stop() {
    //System.out.println("climber2Stop run");
    climberMotor2.set(0);
  }
  public void climber2Down() {
    //System.out.println("climber2Down run");
    climberMotor2.set(ClimberConstants.kSpeed);
  }
    
  
  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
