package frc.robot.subsystems;

//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
//import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ScoringConstants.ArmConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.TuningModeConstants;

public class ScoringArmRotationSubsystem extends SubsystemBase {
    /* private boolean TUNING_MODE = TuningModeConstants.kTuning;

    // Motors
    private final CANSparkMax m_armMotor = new CANSparkMax(
            ArmConstants.kArmCanId, MotorType.kBrushless);

    private RelativeEncoder m_armEncoder;

    // PID Controllers

    private SparkPIDController m_armPIDController = m_armMotor.getPIDController();

    // PID Constants for tuning
    double kArmP = ArmConstants.kArmP;
    double kArmI = ArmConstants.kArmI;
    double kArmD = ArmConstants.kArmD;
    double kArmFF = ArmConstants.kArmFF;
    double kArmMaxOutput = ArmConstants.kArmMaxOutput;
    double kArmMinOutput = ArmConstants.kArmMinOutput;

    // limit switches
    private SparkLimitSwitch m_armLowerLimit;
    private SparkLimitSwitch m_armRaiseLimit;

    public boolean ArmLowerLimitHit() {
        return isArmLowerLimitHit();
    }

    public boolean ArmRaiseLimitHit() {
        return isArmRaiseLimitHit();
    }

    /** ScoringArmRotationSubsystem defines the motors of the arm portion of the robot including arm rotational and arm extension movement. 
     * The arm is the portion of the robot that holds the shooter and intake. It can raise or lower to change the shooting angle and it can extend or retract. 
     * This subsytem defines the motors and motor controllers, PID settings, limit switches and movement control methods for the arm. 
     * */
    /*
    public ScoringArmRotationSubsystem() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.

        m_armMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the arm and shooter arms.
        // temporarily use relative encoders
        // m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_armEncoder = m_armMotor.getEncoder();
        m_armPIDController = m_armMotor.getPIDController();
        m_armPIDController.setFeedbackDevice(m_armEncoder);

        // Apply position and velocity conversion factors for the encoders. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_armEncoder.setPositionConversionFactor(ArmConstants.kArmEncoderPositionFactor);
        m_armEncoder.setVelocityConversionFactor(ArmConstants.kArmEncoderVelocityFactor);

        // Invert the encoders, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        // m_armEncoder.setInverted(ArmConstants.kArmEncoderInverted);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_armPIDController.setPositionPIDWrappingEnabled(true);
        m_armPIDController.setPositionPIDWrappingMinInput(ArmConstants.kArmEncoderPositionPIDMinInput);
        m_armPIDController.setPositionPIDWrappingMaxInput(ArmConstants.kArmEncoderPositionPIDMaxInput);

        // Set the PID gains for the turning motor.
        m_armPIDController.setP(ArmConstants.kArmP);
        m_armPIDController.setI(ArmConstants.kArmI);
        m_armPIDController.setD(ArmConstants.kArmD);
        m_armPIDController.setFF(ArmConstants.kArmFF);
        m_armPIDController.setOutputRange(ArmConstants.kArmMinOutput,
                ArmConstants.kArmMaxOutput);
        m_armPIDController.setSmartMotionMaxAccel(0.5, 0);
        m_armPIDController.setSmartMotionMaxVelocity(0.5, 0);

        m_armMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_armMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);

        // Limit switches
        m_armLowerLimit = m_armMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        m_armRaiseLimit = m_armMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_armMotor.burnFlash();

        if (TUNING_MODE) {
            addPIDToDashboard();
        }
    }

    // Returns the arm
    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }

    // Maintain arm position in degrees
    public void setArmPositionDegrees(double degreesArm) {
        // set degrees for arm, convert to encoder value
        double positionArm = degreesArm * ArmConstants.kArmRevolutionsPerDegree;
        m_armPIDController.setReference(positionArm, ControlType.kPosition);
    }

    public void periodic() {
        // This method will be called once per scheduler run
        log();
        if (TUNING_MODE) {
            tunePIDs();
        }
    }

    public void log() {
        if (LoggingConstants.kLogging) {
            SmartDashboard.putNumber("Arm Position", getArmPosition());
        }
    }

    public void addPIDToDashboard() {
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
    }

    public void tunePIDs() {
        kArmP = SmartDashboard.getNumber("kArmP", 0);
        kArmI = SmartDashboard.getNumber("kArmI", 0);
        kArmD = SmartDashboard.getNumber("kArmD", 0);
        SmartDashboard.putNumber("kArmP", kArmP);
        SmartDashboard.putNumber("kArmI", kArmI);
        SmartDashboard.putNumber("kArmD", kArmD);
    }

    // Limit Switches

    public boolean isArmLowerLimitHit() {
        return m_armLowerLimit.isPressed();
    }

    public boolean isArmRaiseLimitHit() {
        return m_armRaiseLimit.isPressed();
    }

    // Arm Motor Movements

    public void armRaise() {
        m_armMotor.set(ArmConstants.kArmRaiseSpeed);
    }

    public void armLower() {
        m_armMotor.set(-ArmConstants.kArmLowerSpeed);
    }

    public void armStop() {
        m_armMotor.set(0);
    }

    public double getCurrentArmPosition() {
        return m_armEncoder.getPosition();
    }

    public void keepArmPosition(double armPosition) {
        m_armPIDController.setReference(armPosition, ControlType.kPosition);
        if (TUNING_MODE) {
            SmartDashboard.putNumber("Desired Arm Position", armPosition);
            System.out.println("Keep ARM Position " + armPosition);
        }
    } */
}