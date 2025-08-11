package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakePivotLeftMotor;
    private final SparkMax intakePivotRightMotor;
    private final SparkMax intakeWheelMotor;
    private final CANcoder intakeCANcoder;
    private final AnalogInput irSensor;


    private final SparkMaxConfig intakePivotLeftMotorConfig;
    private final SparkMaxConfig intakePivotRightMotorConfig;
    private final SparkMaxConfig intakeWheelMotorConfig;
    private final CANcoderConfiguration intakeCANcoderConfig;

    private final PIDController intakePID;
    private final ArmFeedforward intakeFeedforward;

    private double intakePivotSetpoint;
    private double intakePIDOutput;
    private double intakeFeedforwardOutput;
    private double intakeOutput;
    
    public IntakeSubsystem() {   
        intakePivotLeftMotor = new SparkMax(IntakeConstants.intakePivotLeftMotorID, MotorType.kBrushless);
        intakePivotRightMotor = new SparkMax(IntakeConstants.intakePivotRightMotorID, MotorType.kBrushless);
        intakeWheelMotor = new SparkMax(IntakeConstants.intakeWheelMotorID, MotorType.kBrushless);
        intakeCANcoder = new CANcoder(IntakeConstants.intakeCANcoderID);
        irSensor = new AnalogInput(IntakeConstants.IntakeIRSensorID);

        intakePivotLeftMotorConfig = new SparkMaxConfig();
        intakePivotRightMotorConfig = new SparkMaxConfig();
        intakeWheelMotorConfig = new SparkMaxConfig();
        intakeCANcoderConfig = new CANcoderConfiguration();

        intakePID = new PIDController(0, 0, 0);
        intakeFeedforward = new ArmFeedforward(0, 0, 0);

        intakePivotLeftMotorConfig.idleMode(IdleMode.kBrake);
        intakePivotRightMotorConfig.idleMode(IdleMode.kBrake);
        intakeWheelMotorConfig.idleMode(IdleMode.kCoast);

        intakePivotLeftMotorConfig.inverted(false);
        intakePivotRightMotorConfig.inverted(false);
        intakeWheelMotorConfig.inverted(false);

        intakePivotLeftMotorConfig.follow(intakePivotRightMotor.getDeviceId(), true);

        intakeCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        intakeCANcoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.intakeCANcoderOffset;

        intakePivotLeftMotor.configure(intakePivotLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakePivotRightMotor.configure(intakePivotRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeWheelMotor.configure(intakeWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeCANcoder.getConfigurator().apply(intakeCANcoderConfig);
    }
    
    public void intakeCoral_Pivot(){
        intakePivotSetpoint = IntakeConstants.intakeOutPosition;
    }
    public void intakePrimitive_Pivot(){
        intakePivotSetpoint = IntakeConstants.intakePrimitivePosition;
    }

    public void intakeCoral_Wheel(){
        intakeWheelMotor.setVoltage(IntakeConstants.intakeCoralVol);
    }
    public void rejectCoral_Wheel(){
        intakeWheelMotor.setVoltage(IntakeConstants.rejectCoralVol);
    }
    public void stopMotor(){
        intakeWheelMotor.stopMotor();
    }

    public double getAbsolutePosition(){
        return intakeCANcoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getAngle_Degrees(){
        return getAbsolutePosition() * 360;
    }
    public double getAngle_Raadians(){
        return Math.toRadians(getAngle_Degrees());
    }
    public double getAngularVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(intakeCANcoder.getVelocity().getValueAsDouble() * 60);
    }
    
    public double getDistance() {
        return irSensor.getValue();
    }
    public boolean hasGamePiece(){
        return getDistance() <= 600;
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intakedistance", getDistance());
        intakeFeedforwardOutput = intakeFeedforward.calculate(getAngle_Raadians(), getAngularVelocity());
        intakePIDOutput = intakePID.calculate(getAngle_Degrees(), intakePivotSetpoint);
        intakePIDOutput = Constants.setMaxOutput(intakePIDOutput, 0.4);
        intakeOutput = intakeFeedforwardOutput + intakePIDOutput;
        intakePivotRightMotor.setVoltage(intakeOutput);
    }
}
