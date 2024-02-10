package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ScoringConstants.ArmConstants;
import frc.robot.subsystems.ScoringArmRotationSubsystem;

public class AimToAMP extends SequentialCommandGroup {
    public AimToAMP(
            ScoringArmRotationSubsystem m_arm) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to Travel Position**")),
                new ParallelCommandGroup(
                    new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kAmpArm))
                    //,new InstantCommand(() -> m_arm.TBD_EXTEND_ARM(ArmConstants.kAmpExtension))
                    )
        );
    }
}