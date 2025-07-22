package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final TalonFX climberMotor;
    private final TalonFXConfiguration climberMotorCfg;
    private final PositionDutyCycle request = new PositionDutyCycle(0);

    public ClimberSubsystem() {
        climberMotorCfg = new TalonFXConfiguration();
        climberMotor = new TalonFX(1);
        climberMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberMotorCfg.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        climberMotorCfg.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        climberMotorCfg.Slot1.kP = 1;
        climberMotorCfg.Slot1.kD = 0;
        climberMotorCfg.Slot1.kI = 0;
        climberMotor.getConfigurator().apply(climberMotorCfg);
    }

    public void setAngle(DoubleSupplier Rotation) {
        climberMotor.setControl(request.withPosition(Rotation.getAsDouble()).withSlot(1));
    }
}
