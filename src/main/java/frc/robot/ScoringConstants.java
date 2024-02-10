
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ScoringConstants {

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static final class IntakeConstants {
        public static final int kIRPort = 1;
        public static final int kIntakeCanId = 21;
        public static final double speed = .5;
        public static final double outtakeSpeed = -.2;
    }

	public static final class ClimberConstants {
        public static final int kClimber1CanId = 000;
		public static final int kClimber2CanId = 000;
		public static final IdleMode kClimberMotor1IdleMode = IdleMode.kBrake;
		public static final IdleMode kClimberMotor2IdleMode = IdleMode.kBrake;
        public static final double speed = .25;
		public static final int kClimberMotor1CurrentLimit = 0;
		public static final int kClimberMotor2CurrentLimit = 0;
		public static double kClimber1MaxOutput = 0.8;
        public static double kClimber1MinOutput = -0.8;
		public static double kClimber2MaxOutput = 0.8;
        public static double kClimber2MinOutput = -0.8;
    }

	public static final class ShooterConstants {
        public static final int kShooter1CanId = 26;
        public static final int kShooter2CanId = 27;
        public static final int kShooterMotor2CurrentLimit = 0;
        public static final IdleMode kShooterMotor2IdleMode = IdleMode.kBrake;
        public static final int kShooterMotor1CurrentLimit = 0;
        public static IdleMode kShooterMotor1IdleMode = IdleMode.kBrake;
        public static double kShooter2MaxOutput = 0.8;
        public static double kShooter2MinOutput = -0.8;
        public static double kShooter2FF = 0;
        public static double kShooter2P = 1;
        public static double kShooter2I = 0.0001;
        public static double kShooter2D = 0;
        public static double kShooter1MaxOutput = 0.8;
        public static double kShooter1MinOutput = -0.8;
        public static double kShooter1FF = 0;
        public static double kShooter1P = 1;
        public static double kShooter1I = 0.0001;
        public static double kShooter1D = 0;
        public static double kShooterEncoder2VelocityFactor = (2 * Math.PI) / 60.0;
        public static double kShooterEncoder2PositionFactor = (2 * Math.PI);
        public static double kshooterEncoder1VelocityFactor = (2 * Math.PI) / 60.0;
        public static double kshooterEncoder1PositionFactor = (2 * Math.PI);
    }

	public static final class ArmConstants {
        public static final int kArmCanId = 23;

        public static final int kArmMotorCurrentLimit = 20; // amps

        // Invert the encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kArmEncoderInverted = true;

        public static final double kArmEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kArmEncoderPositionPIDMinInput = 0; // radians
        public static final double kArmEncoderPositionPIDMaxInput = kArmEncoderPositionFactor; // radians

        public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;

        public static final double kArmMaxSpeed = 0.3;

        // Manual Arm movement speeds
        public static final double kArmRaiseSpeed = .15;
        public static final double kArmLowerSpeed = .15;

        // PIDS
        // Arm PID coefficients
        public static final int kArmPIDSlot = 0;
        public static final double kArmP = 1;
        public static final double kArmI = 0.0001;
        public static final double kArmD = 0.1;
        public static final double kArmFF = 0;
        public static final double kArmMaxOutput = kArmMaxSpeed;
        public static final double kArmMinOutput = -kArmMaxSpeed;

        // SETPOINTS

        public static final double kAdjustmentFactor = 90;

        public enum Position {
            TRAVEL("Travel", 35 - kAdjustmentFactor),
            PODIUM("Podium", 46 - kAdjustmentFactor),
            LOW_SUBWOOFER("LowSubwoofer", 75 - kAdjustmentFactor);

            private static final Map<String, Position> BY_LABEL = new HashMap<>();
            private static final Map<Double, Position> BY_ARM_POSITION = new HashMap<>();

            static {
                for (Position e : values()) {
                    BY_LABEL.put(e.label, e);
                    BY_ARM_POSITION.put(e.armPosition, e);
                }
            }

            public final String label;
            public final double armPosition;

            private Position(String label, double armPosition) {
                this.label = label;
                this.armPosition = armPosition;
            }

            public static Position valueOfLabel(String label) {
                return BY_LABEL.get(label);
            }

            public static Position valueOfArmPosition(double armPosition) {
                return BY_ARM_POSITION.get(armPosition);
            }

        }

        public static final double kTravelArm = 35 - kAdjustmentFactor;
        public static final double kTravelShooter = 83;

        // podium
        public static final double kPodiumArm = 46 - kAdjustmentFactor;
        public static final double kPodiumShooter = 79;

        // low subwoofer
        public static final double kLowSubwooferArm = 75 - kAdjustmentFactor;
        public static final double kLowSubwooferShooter = 75;

        // amp
        public static final double kAmpArm = 114 - kAdjustmentFactor;
        public static final double kAmpShooter = -39;

        // trap approach
        public static final double kTrapApproachArm = 132 - kAdjustmentFactor;
        public static final double kTrapApproachShooter = -42;

        // trap climb
        public static final double kTrapClimbArm = 130 - kAdjustmentFactor;
        public static final double kTrapClimbShooter = -40;

        // trap score
        public static final double kTrapScoreArm = 130 - kAdjustmentFactor;
        public static final double kTrapScoreShooter = -55;

        // high podium
        public static final double kHighPodiumArm = 127 - kAdjustmentFactor;
        public static final double kHighPodiumShooter = -40;

        // back podium
        public static final double kBackPodiumArm = 127 - kAdjustmentFactor;
        public static final double kBackPodiumShooter = 166;

        // high subwoofer
        public static final double kHighSubwooferArm = 127 - kAdjustmentFactor;
        public static final double kHighSubwooferShooter = 23;

        // Estimates, fix this once we get exact measurements
        public static final double kArmTotalDegrees = 72.4; // TODO
        public static final double kArmTotalRevolutions = 5.488; // TODO

        // Convert angle of travel to encoder rotations, where encoder reading of .1 is
        // 0 degrees and reading of 5.5 is 90 degrees
        public static final double kArmRevolutionsPerDegree = -(kArmTotalRevolutions)
                / kArmTotalDegrees;

        // Estimates, fix this once we get exact measurements
        public static final double kShooterArmTotalDegrees = 72.4; // TODO
        public static final double kShooterArmTotalRevolutions = 5.488; // TODO

        // Convert angle of travel to encoder rotations, where encoder reading of .1 is
        // 0 degrees and reading of 5.5 is 90 degrees
        public static final double kShooterArmRevolutionsPerDegree = -(kShooterArmTotalRevolutions)
                / kShooterArmTotalDegrees;
	}

}
