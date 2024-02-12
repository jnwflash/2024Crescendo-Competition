// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.ControllerDevice;
import frc.robot.Devices.Controller;
import frc.robot.commands.ArmUp;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmStop;

//import frc.robot.commands.Autos;
import frc.robot.commands.ClimbersDown;
import frc.robot.commands.ClimbersStop;
import frc.robot.commands.ClimbersUp;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.TurnToAngleZeroHeadingCommand;
import frc.robot.commands.ZeroHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static Controller xboxController;
    public static final String kDefaultAuto = "1MeterForward";
    public static final String kCustomAuto = "SwiggleWiggle";
    public static final String kCustomAuto2 = "1Meter45Diag";
    public static final String kCustomAuto3 = "Test";
    public String ChosenAuto;
    public final SendableChooser<String> m_chooser = new SendableChooser<>();

    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    public static final IMUSubsystem imuSubsystem = new IMUSubsystem();
    public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem();
    public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static final ArmSubsystem armSubsystem = new ArmSubsystem();
    
    //private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandGenericHID m_operator1Controller = new CommandGenericHID(0);
    private final CommandGenericHID m_operator2Controller = new CommandGenericHID(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Define Auto Options
        m_chooser.setDefaultOption("1MeterForward", kDefaultAuto);
        m_chooser.addOption("SwiggleWiggle", kCustomAuto);
        m_chooser.addOption("1Meter45Diag", kCustomAuto2);
        m_chooser.addOption("Test", kCustomAuto3);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Configure driver interface - binding joystick objects to port numbers
        configureDriverInterface();

        // Configure the trigger bindings
        configureBindings();

        // This command should be for the teleop driving
        // Note that the first three of its parameters are DoubleSupplier, and the last one is a
        // BooleanSupplier
        
        driveSubsystem.setDefaultCommand(
                new DriveManuallyCommand(
                        () -> getDriverXAxis(),
                        () -> getDriverYAxis(),
                        () -> getDriverOmegaAxis(),
                        () -> getDriverFieldCentric()));
        }

    private void configureDriverInterface() {
        //driveStick = new Controller(ControllerDevice.DRIVESTICK);
        //turnStick = new Controller(ControllerDevice.TURNSTICK);
        xboxController = new Controller(ControllerDevice.XBOX_CONTROLLER);
        //System.out.println("Driver interface configured");
    }

    private void configureBindings() {
        
        //Climber Bindings
        new Trigger(m_operator2Controller.button(1))
            .onTrue(new ClimbersUp(climberSubsystem))
            .onFalse(new ClimbersStop(climberSubsystem));
        //new Trigger(m_operator2Controller.button(2))
        //    .onTrue(new ClimbersStop(climberSubsystem));
        new Trigger(m_operator2Controller.button(3))
            .onTrue(new ClimbersDown(climberSubsystem))
            .onFalse(new ClimbersStop(climberSubsystem));

        //Arm Bindings
        new Trigger(m_operator2Controller.button(4))
            .onTrue(new ArmUp(armSubsystem));
        new Trigger(m_operator2Controller.button(5))
            .onTrue(new ArmDown(armSubsystem));
        new Trigger(m_operator2Controller.button(6))
            .onTrue(new ArmStop(armSubsystem));
    }

    private double getDriverXAxis() {
        // return -driveStick.getLeftStickY();
        //System.out.println("***--- DX:"+-xboxController.getLeftStickY());
        return -xboxController.getLeftStickY();
    }

    private double getDriverYAxis() {
        // return -driveStick.getLeftStickX();
        return -xboxController.getLeftStickX();
    }

    private double getDriverOmegaAxis() {
        // return -turnStick.getLeftStickOmega();
        return -xboxController.getLeftStickOmega();
    }

    private boolean getDriverFieldCentric() {
        //return !turnStick.getRawButton(OIConstants.robotCentricButton);
        //return !xboxController.getRawButton(OIConstants.robotCentricButton);
        return true;
    }

/**
* Bindings to test simple swerve trajectories done in PathPlanner
*/
    public void trajectoryCalibration() {
        new Trigger(m_operator1Controller.button(1))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1MeterForward"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(2))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1MeterSideways"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(3))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("1Meter45Diag"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(4))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("MeterStraightTurn90"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(5))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("InPlaceTurn90"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(6))
            .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("SwiggleWiggle"))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(7))
            .whileTrue(new ZeroHeadingCommand())
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        new Trigger(m_operator1Controller.button(8))
            .whileTrue(new TurnToAngleZeroHeadingCommand(Rotation2d.fromDegrees(0)))
            .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem));
        //new JoystickButton(turnStick, 12)
        //    .whileTrue(new InstantCommand(RobotContainer.driveSubsystem::testOdometryUpdates));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        ChosenAuto = m_chooser.getSelected();
        return new RunTrajectorySequenceRobotAtStartPoint(ChosenAuto);
    }
}