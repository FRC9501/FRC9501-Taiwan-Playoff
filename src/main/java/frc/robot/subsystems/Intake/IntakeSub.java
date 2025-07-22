package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {      
    
    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
    private final TalonFXConfiguration armCFG = new TalonFXConfiguration();
    private final TalonFX armTalonFX = new TalonFX(1);
    private final PositionDutyCycle request = new PositionDutyCycle(0); 

    public IntakeSub() {
        armCFG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armCFG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armCFG.Slot0.kP = 0.05;
        armCFG.Slot0.kI = 0;
        armCFG.Slot0.kD = 0;
        armCFG.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armCFG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        armCFG.MotorOutput.PeakForwardDutyCycle = 0.2;
        armCFG.MotorOutput.PeakReverseDutyCycle = -0.2;
        armTalonFX.getConfigurator().apply(armCFG);
        resetEncoder();
        readyPosition();
    }
    
    public void readyPosition() {
        armTalonFX.setControl(request.withSlot(0).withPosition(0));
    }

    public void L0Position() {
        armTalonFX.setControl(request.withSlot(0).withPosition(1));
    }

    public void L1Position() {
        armTalonFX.setControl(request.withSlot(0).withPosition(2));
    }

    public void L2Position() {
        armTalonFX.setControl(request.withSlot(0).withPosition(3));
    }
    public void L3Position() {
        armTalonFX.setControl(request.withSlot(0).withPosition(4));
    }
    public void L4Position() {
        armTalonFX.setControl(request.withSlot(0).withPosition(5));
    }

    public void resetEncoder() {
        armTalonFX.setPosition(0);
    }

    public void takeIn() {
        motor.set(1);
    }

    public void shoot() {
        motor.set(-1);
    }

    @Override
    public void periodic() {
    }
}
