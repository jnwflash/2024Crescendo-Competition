package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoggingConstants;
import frc.robot.ScoringConstants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    // imports motor id
    private final CANSparkMax m_climber1Motor = new CANSparkMax(
            ClimberConstants.kClimber1CanId, MotorType.kBrushless);
    private final CANSparkMax m_climber2Motor = new CANSparkMax(
            ClimberConstants.kClimber2CanId, MotorType.kBrushless);

    // Encoders
    private RelativeEncoder m_climber1Encoder;
    private RelativeEncoder m_climber2Encoder;

    // PID Controllers
    private SparkPIDController m_climber1PIDController = m_climber1Motor.getPIDController();
    private SparkPIDController m_climber2PIDController = m_climber2Motor.getPIDController();
    

    /** Creates a new ExampleSubsystem. */
    public ClimberSubsystem() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.

        m_climber1Motor.restoreFactoryDefaults();
        m_climber2Motor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the Shooter1 and shooter Shooter1s.
        m_climber1Encoder = m_climber1Motor.getEncoder();
        m_climber1PIDController = m_climber1Motor.getPIDController();
        m_climber1PIDController.setFeedbackDevice(m_climber1Encoder);

        m_climber2Encoder = m_climber2Motor.getEncoder();
        m_climber2PIDController = m_climber2Motor.getPIDController();
        m_climber2PIDController.setFeedbackDevice(m_climber2Encoder);



        m_climber1Motor.setIdleMode(ClimberConstants.kClimberMotor2IdleMode);
        m_climber1Motor.setSmartCurrentLimit(ClimberConstants.kClimberMotor1CurrentLimit);

        m_climber2Motor.setIdleMode(ClimberConstants.kClimberMotor2IdleMode);
        m_climber2Motor.setSmartCurrentLimit(ClimberConstants.kClimberMotor2CurrentLimit);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        /*
        if (TuningModeConstants.kTuning) {
            tunePIDs();
        }
        */
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            // SmartDashboard.putNumber("xxxxx", sampleMethod());
            //
        }
    }

    // Stop the Climber
    public void stop() {
        m_climber1Motor.set(0);
        m_climber2Motor.set(0);
    }

}