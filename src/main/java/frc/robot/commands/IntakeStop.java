package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ScoringIntakeSubsystem;
public class IntakeStop extends SequentialCommandGroup{
    public IntakeStop(
        ScoringIntakeSubsystem m_Intake
    ){
    addCommands(
        new InstantCommand(() -> m_Intake.stop()));
    }
}