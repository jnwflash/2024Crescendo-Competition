// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

  private static final int Arm1ID = 10;
  private static final int Arm2ID = 11;

  private CANSparkMax armMotor1;
  private CANSparkMax armMotor2;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

  armMotor1 = new CANSparkMax(Arm1ID, MotorType.kBrushless);
  armMotor2 = new CANSparkMax(Arm2ID, MotorType.kBrushless);

  armMotor1.restoreFactoryDefaults();
  armMotor2.restoreFactoryDefaults();

  armMotor1.setInverted(true);
  armMotor2.follow(armMotor1);

  }
  public void armUp (){

    armMotor1.set(-.1);
  }
   public void armStop (){

    armMotor1.set(0);
  }
  
  public void armDown (){

    armMotor1.set(.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
