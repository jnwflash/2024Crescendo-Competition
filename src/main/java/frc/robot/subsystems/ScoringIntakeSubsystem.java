package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/* 
import frc.robot.Constants.DriveConstants;

import frc.robot.Constants.ShooterConstants;
*/
import frc.robot.Constants.LoggingConstants;
import frc.robot.ScoringConstants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ScoringIntakeSubsystem extends SubsystemBase {
    // imports motor id
    private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
    DigitalInput breakBeam = new DigitalInput(IntakeConstants.kIRPort);
    DigitalInput breakMaxBeam = new DigitalInput(IntakeConstants.kIRPort);

    private double speed = IntakeConstants.speed;
    private double outakeSpeed = IntakeConstants.outtakeSpeed;

    /** ScoringIntakeSubsystem defines the motors of the intake portion of the robot. 
     * The intake is the portion of the robot picks up game pieces, but it does not include the arm it is attached to 
     * This subsytem defines the motors and motor controllers, limit switches methods for the intake. 
     * */
    public ScoringIntakeSubsystem() {
        SmartDashboard.putNumber("Intake speed", speed);
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */

    public void outtake() {
        m_intakeMotor.set(outakeSpeed);
    }

    public void forward() {
        m_intakeMotor.set(speed);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        log();
        NoteDetected();
        tuneSpeeds();
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putBoolean("Note detected", NoteDetected());
        }
    }

    public Boolean NoteDetected() {
        return !breakBeam.get();
    }

    public Boolean NoteTooFar(){
        return !breakMaxBeam.get();
    }

    public void tuneSpeeds() {
        speed = SmartDashboard.getNumber("Intake speed", IntakeConstants.speed);
        SmartDashboard.putNumber("Intake speed", speed);
    }

}