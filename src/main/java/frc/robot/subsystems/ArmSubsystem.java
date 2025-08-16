package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armPivotMotor;
    private final SparkMax armWheelMotor;
    private final DutyCycleEncoder armAbsoluteEncoder;
    private final Encoder armRelativeEncoder;
    private final AnalogInput irSensor;

    private final SparkMaxConfig armWheelMotorConfig;
    private final TalonFXConfiguration armPivotMotorConfig;

    private final PIDController armPID;
    private final ArmFeedforward armFeedforward;

    private double armGoalPosition;
    private double armPIDOutput;
    private double armFeedforwardOutput;
    private double armOutput;
    
    // private static final Distance LEdSpacing =  Meters.of(1 / 120.0);
    
    

    public ArmSubsystem() {
        armPivotMotor = new TalonFX(ArmConstants.armPivotMotorID);
        armWheelMotor = new SparkMax(ArmConstants.armWheelMotorID, MotorType.kBrushless);
        armAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.armAbsoluteEncoderID, 1, ArmConstants.armAbsoluteEncoderOffset);
        armRelativeEncoder = new Encoder(ArmConstants.armRelativeEncoderFirstID, ArmConstants.armRelativeEncoderSecondID, true);
        irSensor = new AnalogInput(ArmConstants.armIRSensorID);

        armPivotMotorConfig = new TalonFXConfiguration();
        armWheelMotorConfig = new SparkMaxConfig();

        armPID = new PIDController(0, 0, 0);
        armFeedforward = new ArmFeedforward(0, 0, 0);

        armPivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armPivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armWheelMotorConfig.idleMode(IdleMode.kBrake);
        armWheelMotorConfig.inverted(false);


        armPivotMotor.getConfigurator().apply(armPivotMotorConfig);
        armWheelMotor.configure(armWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeCoral_Pivot();
    }
    
    //pivot
    public void intakeCoral_Pivot() {
        armGoalPosition = ArmConstants.coralIntakePosition;
    }
    public void readyL1_Pivot() {
        armGoalPosition = ArmConstants.coralL1ReadyPosition;
    }
    public void readyL2_Pivot() {
        armGoalPosition = ArmConstants.coralL2ReadyPosition;
    }
    public void readyL3_Pivot() {
        armGoalPosition = ArmConstants.coralL3ReadyPosition;
    }
    public void readyL4_Pivot() {
        armGoalPosition = ArmConstants.coralL4ReadyPosition;
    }
    public void putL1_Pivot() {
        armGoalPosition = ArmConstants.coralL1PutPosition;
    }
    public void putL2_Pivot() {
        armGoalPosition = ArmConstants.coralL2PutPosition;
    }
    public void putL3_Pivot() {
        armGoalPosition = ArmConstants.coralL3PutPosition;
    }
    public void putL4_Pivot() {
        armGoalPosition = ArmConstants.coralL4PutPosition;
    }
    public void intakeAlgaeHigh_Pivot(){
        armGoalPosition = ArmConstants.algaeHighPosition;
    }
    public void intakeAlgaeLow_Pivot(){
        armGoalPosition = ArmConstants.algaeLowPosition;
    }
    public void intakeAlgaeFloor_Pivot(){
        armGoalPosition = ArmConstants.algaeFloorPosition;
    }
    public void putNet_Pivot() {
        armGoalPosition = ArmConstants.algaeNetPosition;
    }
    public void putProcesser_Pivot() {
        armGoalPosition = ArmConstants.algaeProcessorPosition;
    }
    public void primitive_Algae_Pivot() {
        armGoalPosition = ArmConstants.algaePrimitivePosition;
    }

    //wheel
    public void intakeCoral_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.intakeCoralVol);
    }
    public void intakeAlgaeHih_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.intakeAlgaeHighVol);
    }
    public void intakeAlgaeLow_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.intakeAlgaeLowVol);
    }
    public void intakeAlgaeFloor_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.intakeAlgaeFloorVol);
    }
    public void putNet_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putNetVol);
    }
    public void putProcessor_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putProcessorVol);
    }
    public void holdAlgae_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.holdAlgaeVol);
    }
    public void putL1_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putL1Vol);
    }



    public double getAbsolutePosition(){
        return armAbsoluteEncoder.get();
    }
    public double getAngle_Degrees(){
        return getAbsolutePosition() * 360;
    }
    public double getAngle_Radians(){
        return Math.toRadians(getAngle_Degrees());
    }
    public double getAngularVelocity(){
        return Units.rotationsPerMinuteToRadiansPerSecond(armRelativeEncoder.getRate());
    }
    public boolean arriveSetpoint(){
        return Math.abs(armPID.getError()) <= 1;
    }
    public int getDistance() {
        return irSensor.getValue();
    }

    public void brake(){
        armWheelMotorConfig.idleMode(IdleMode.kBrake);
        armWheelMotor.configure(armWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void coast(){
        armWheelMotorConfig.idleMode(IdleMode.kCoast);
        armWheelMotor.configure(armWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    public boolean hasGamePiece_Arm(){
        return getDistance() <= 600;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armdistance", getDistance());
        armFeedforwardOutput = armFeedforward.calculate(getAngle_Radians(), getAngularVelocity());
        armPIDOutput = armPID.calculate(armGoalPosition, getAngle_Degrees());
        armPIDOutput = Constants.setMaxOutput(armPIDOutput, 0.4);
        armOutput = armFeedforwardOutput + armPIDOutput;
        armPivotMotor.setVoltage(armOutput);
    }
}
