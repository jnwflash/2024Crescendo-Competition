package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ClimberSubsystem;
public class ClimbersUp extends SequentialCommandGroup{
    public ClimbersUp(
        ClimberSubsystem m_climber

    ){
    addCommands(
        new ParallelCommandGroup(
        new InstantCommand(() -> m_climber.climber1Up()),
        new InstantCommand(() -> m_climber.climber2Up())
        )
    );
    }
}