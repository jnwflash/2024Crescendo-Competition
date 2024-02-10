package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ScoringArmRotationSubsystem;

import static frc.robot.ScoringConstants.ArmConstants.Position;

public class AimToSpeakerFromAnywhere extends SequentialCommandGroup {
    public AimToSpeakerFromAnywhere(
            ScoringArmRotationSubsystem m_arm,
            Position position) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to " + position.label + " Position**")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_arm.setArmPositionDegrees(position.armPosition))
                        //,new InstantCommand(() -> m_arm.TBD_EXTEND_ARM(ArmConstants.kAmpExtension))
            ));
    }

}