package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ScoringConstants.ArmConstants;
import frc.robot.subsystems.ScoringArmRotationSubsystem;

public class AimToSpeakerFromPodium extends SequentialCommandGroup {
    /*
    public AimToSpeakerFromPodium(
            ScoringArmRotationSubsystem m_arm) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to Podium Position**")),
                new ParallelCommandGroup(
                    new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kPodiumArm))
                    //,new InstantCommand(() -> m_arm.TBD_EXTEND_ARM(ArmConstants.kAmpExtension))                    )
        ));
    }
    */
}