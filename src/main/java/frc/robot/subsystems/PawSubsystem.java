package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static frc.robot.Constants.PawConstants.*;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PawSubsystem extends SubsystemBase {
    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
    private final TalonFXConfiguration armCFG = new TalonFXConfiguration();
    private final TalonFX armTalonFX = new TalonFX(1);
    private final PositionDutyCycle request = new PositionDutyCycle(0); 
    private double position;
    
    // private static final Distance LEdSpacing =  Meters.of(1 / 120.0);
    private AnalogInput analog = new AnalogInput(0);
    private final SparkMaxConfig motorconfig;

    public PawSubsystem() {
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
        position = kreadyPosition;
    }
    public void L1Position() {
        position = kL1Position;
    }
    public void L2Position() {
        position = kL2Position;
    }
    public void L3Position() {
        position = kL3Position;
    }
    public void L4Position() {
        position = kL4Position;
    }
    public void netPosition() {
        position = knetPosition;
    }
    public void processer() {
        position = kprocesserPosition;
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
    public double distance() {
        return analog.getValue();
    }
    
    public void takeIn() {
        motor.set(1);
        if (distance() > 650) {
            stop();
        } else {
            motor.set(1);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance", distance());
        armTalonFX.setControl(request.withPosition(position));
    }
}
