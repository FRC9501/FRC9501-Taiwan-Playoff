package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armPivotMotor;
    private final TalonFX armWheelMotor;
    private final DutyCycleEncoder armAbsoluteEncoder;
    private final Encoder armRelativeEncoder;

    private final TalonFXConfiguration armWheelMotorConfig;
    private final TalonFXConfiguration armPivotMotorConfig;

    private final PIDController armPID;
    private final ArmFeedforward armFeedforward;

    private double armGoalPosition;
    private double armPIDOutput;
    private double armFeedforwardOutput;
    private double armOutput;
    private boolean back;
    private double armMaxOutput;
    private String mode = "start";
    
    // private static final Distance LEdSpacing =  Meters.of(1 / 120.0);
    
    

    public ArmSubsystem() {
        armPivotMotor = new TalonFX(ArmConstants.armPivotMotorID);
        armWheelMotor = new TalonFX(ArmConstants.armWheelMotorID);
        armAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.armAbsoluteEncoderID, 1, ArmConstants.armAbsoluteEncoderOffset);
        armRelativeEncoder = new Encoder(ArmConstants.armRelativeEncoderFirstID, ArmConstants.armRelativeEncoderSecondID, false);

        armPivotMotorConfig = new TalonFXConfiguration();
        armWheelMotorConfig = new TalonFXConfiguration();

        armPID = new PIDController(0.2, 0, 0);
        armFeedforward = new ArmFeedforward(0, 0.2, 0);

        armPivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armPivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armWheelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        back = false;


        armPivotMotor.getConfigurator().apply(armPivotMotorConfig);
        armWheelMotor.getConfigurator().apply(armWheelMotorConfig);
        primitive_Algae_Pivot();
    }
    
    //pivot
    public void intakeCoral_Pivot() {
        armGoalPosition = ArmConstants.coralIntakePosition;
        mode = "intakeCoral";
    }
    public void putL1_Pivot() {
        armGoalPosition = ArmConstants.coralL1PutPosition;
        mode = "putL1";
    }
    public void putL2_Pivot(){
        armGoalPosition = ArmConstants.coralL2PutPosition;
        mode = "putL2";
    }
    public void putL3_Pivot(){
        armGoalPosition = ArmConstants.coralL3PutPosition;
        mode = "putL3";
    }
    public void putL4_Pivot(){
        armGoalPosition = ArmConstants.coralL4PutPosition;
        mode = "putL4";
    }
    public void readyL2_Pivot() {
        armGoalPosition = ArmConstants.coralL2ReadyPosition;
        mode = "readyL2";
    }
    public void readyL3_Pivot() {
        armGoalPosition = ArmConstants.coralL3ReadyPosition;
        mode = "readyL3";
    }
    public void readyL4_Pivot() {
        armGoalPosition = ArmConstants.coralL4ReadyPosition;
        mode = "readyL4";
    }
    public void removeAlgaeHigh_Pivot(){
        armGoalPosition = ArmConstants.algaeHighPosition;
        back = false;
        mode = "removeAlgae_High";
    }
    public void removeAlgaeLow_Pivot(){
        armGoalPosition = ArmConstants.algaeLowPosition;
        back = false;
        mode = "removeAlgae_Low";
    }
    // public void removeAlgaeFloor_Pivot(){
    //     armGoalPosition = ArmConstants.algaeFloorPosition;
    //     back = false;
    //     mode = "removeAlgae_Low";
    // }
    public void putNet_Pivot() {
        armGoalPosition = ArmConstants.algaeNetPosition;
        back = true;
        mode = "putNet";
    }
    public void putProcesser_Pivot() {
        armGoalPosition = ArmConstants.algaeProcessorPosition;
        back = true;
        mode = "putProcessor";
    }
    public void primitive_Algae_Pivot() {
        armGoalPosition = ArmConstants.algaePrimitivePosition;
        back = true;
        mode = "primitiveAlgae";
    }

    //wheel
    public void intakeCoral_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.intakeCoralVol);
    }
    public void removeAlgaeHigh_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.removeAlgaeHighVol);
    }
    public void removeAlgaeLow_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.removeAlgaeLowVol);
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
    public void putL2_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putL2Vol);
    }
    public void putL3_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putL3Vol);
    }
    public void putL4_Wheel(){
        armWheelMotor.setVoltage(ArmConstants.putL4Vol);
    }
    public void stopWheel(){
        armWheelMotor.stopMotor();
    }



    public double getAbsolutePosition(){
        return armAbsoluteEncoder.get();
    }
    public double getAngle_Degrees(){
        return getAbsolutePosition() * 360;
    }
    public double getAngle_Degrees_PID(){
        return (getAngle_Degrees() + 180) % 360;
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
    public boolean hasAlgae(){
        return armWheelMotor.getSupplyCurrent().getValueAsDouble() >= 27;
    }

    public boolean hasCoral(){
        return armWheelMotor.getSupplyCurrent().getValueAsDouble() >= 1.8;
    }

    public String getMode(){
        return mode;
    }
    
    

    @Override
    public void periodic() {
        if(back){
            armMaxOutput = 2;
        }else{
            armMaxOutput = 3.6;
        }
        armFeedforwardOutput = armFeedforward.calculate(getAngle_Radians(), getAngularVelocity());
        armPIDOutput = armPID.calculate(getAngle_Degrees_PID(), armGoalPosition);
        armPIDOutput = Constants.setMaxOutput(armPIDOutput, armMaxOutput);
        armOutput = armFeedforwardOutput + armPIDOutput;
        armPivotMotor.setVoltage(armOutput);
        SmartDashboard.putNumber("Arm/ArmAngle", getAngle_Degrees());
        SmartDashboard.putNumber("Arm/ArmAngle_PID", getAngle_Degrees_PID());
        SmartDashboard.putNumber("Arm/ArmEncoderPosition", getAbsolutePosition());
        SmartDashboard.putNumber("Arm/ArmAngularVelocity", getAngularVelocity());
        SmartDashboard.putNumber("Arm/ArmPID", armPIDOutput);
        SmartDashboard.putNumber("Arm/ArmFeedforward", armFeedforwardOutput);
        SmartDashboard.putNumber("Arm/ArmSetpoint", armGoalPosition);
        SmartDashboard.putNumber("Arm/MotorPosition", armPivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Arm/HasAlgae", hasAlgae());
        SmartDashboard.putBoolean("Arm/ArmArriveSetpoint", arriveSetpoint());
        SmartDashboard.putString("Arm/ArmMode", mode);
        SmartDashboard.putBoolean("Arm/HasCoral", hasCoral());
    }
}
