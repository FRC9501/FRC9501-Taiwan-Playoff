package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import static edu.wpi.first.units.Units.Meters;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
    private final TalonFXConfiguration armCFG = new TalonFXConfiguration();
    private final TalonFX armTalonFX = new TalonFX(1);
    private final PositionDutyCycle request = new PositionDutyCycle(0); 
    private double position;
    // private static final Distance LEdSpacing =  Meters.of(1 / 120.0);
    private AnalogInput analog = new AnalogInput(0);
    private final SparkMaxConfig motorconfig;

    public IntakeSubsystem() {
        motorconfig = new SparkMaxConfig();
        motorconfig.idleMode(IdleMode.kCoast);
        armCFG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armCFG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armCFG.Slot0.kG = 0;
        armCFG.Slot0.kS = 0;
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
        position = 0;
    }
    public void L1Position() {
        position = 1;
    }
    public void L2Position() {
        position = 2;
    }
    public void L3Position() {
        position = 3;
    }
    public void L4Position() {
        position = 4;
    }
    public void netPosition() {
        position = 5;
    }
    public void processer() {
        position = 7;
    }

    public void resetEncoder() {
        armTalonFX.setPosition(0);
    }
    public double nowposition() {
        return armTalonFX.getPosition().getValueAsDouble();
    }
    public boolean setpoint() {
        return Math.abs(position - nowposition()) <= 1.0;
    }
    public void shoot() {
        motor.set(-1);
    }
    public void stop() {
        motor.set(0);
    }
    public double Distance() {
        return analog.getValue();
    }
    
    public void takeIn() {
        motor.set(1);
        if (Distance() > 650) {
            stop();
        } else {
            motor.set(1);
        }
    }

    @Override
    public void periodic() {
        armTalonFX.setControl(request.withPosition(position));
    }
}
