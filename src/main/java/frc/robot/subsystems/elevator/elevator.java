package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private TalonFX LeftTalon;
    private TalonFX RightTalon;
    private TalonFXConfiguration RightTalonCfg = new TalonFXConfiguration();
    private TalonFXConfiguration LeftTalonCfg = new TalonFXConfiguration();
    private PositionDutyCycle request = new PositionDutyCycle(0);
    // private DutyCycleOut duty = new DutyCycleOut(0.2);

    public elevator() {
        LeftTalon = new TalonFX(13);
        RightTalon = new TalonFX(14);

        LeftTalonCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        RightTalonCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;             
        LeftTalonCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RightTalonCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        LeftTalonCfg.Slot1.withGravityType(GravityTypeValue.Elevator_Static);
        LeftTalonCfg.Slot1.kP = 0.02;
        LeftTalonCfg.Slot1.kI = 0;
        LeftTalonCfg.Slot1.kD = 0;
        LeftTalonCfg.Slot1.kA = 0;
        LeftTalonCfg.Slot1.kV = 0;
        LeftTalonCfg.MotorOutput.PeakForwardDutyCycle = 0.6;
        LeftTalonCfg.MotorOutput.PeakReverseDutyCycle = -0.6;
        LeftTalonCfg.Slot1.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        RightTalonCfg.Slot1.withGravityType(GravityTypeValue.Elevator_Static);
        RightTalonCfg.Slot1.kP = 0.02;
        RightTalonCfg.Slot1.kI = 0;
        RightTalonCfg.Slot1.kD = 0;
        RightTalonCfg.Slot1.kA = 0;
        RightTalonCfg.Slot1.kV = 0;
        RightTalonCfg.MotorOutput.PeakForwardDutyCycle = 0.6;
        RightTalonCfg.MotorOutput.PeakReverseDutyCycle = -0.6;
        RightTalonCfg.Slot1.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        RightTalon.getConfigurator().apply(RightTalonCfg);
        LeftTalon.getConfigurator().apply(LeftTalonCfg);

        resetEncoder();
    }

    // public void Duty() {
    //     RightTalon.set(0.3);
    //     LeftTalon.set(0.3);
    // }

    // public void Reverse() {
    //     RightTalon.set(-0.3);
    //     LeftTalon.set(-0.3);
    // }
    public double get() {
        return RightTalon.getPosition().getValueAsDouble();
    }

    public void stop() {
        RightTalon.set(0);
        LeftTalon.set(0);
    }

    public void L3() {
        RightTalon.setControl(request.withPosition(39).withSlot(1).withFeedForward(-0));
        LeftTalon.setControl(request.withPosition(39).withSlot(1).withFeedForward(-0));
    }

    public void L2() {
        RightTalon.setControl(request.withPosition(18).withSlot(1).withFeedForward(0.08));
        LeftTalon.setControl(request.withPosition(18).withSlot(1).withFeedForward(0.08));
    }

    public void L1() {
        RightTalon.setControl(request.withPosition(7).withSlot(1).withFeedForward(0.06));
        LeftTalon.setControl(request.withPosition(7).withSlot(1).withFeedForward(0.06));  
    }

    public void L0() {
        RightTalon.setControl(request.withPosition(0).withSlot(1).withFeedForward(0.04));
        LeftTalon.setControl(request.withPosition(0).withSlot(1).withFeedForward(0.04));  
    }

    @Override
    public void periodic() {
        get();
        SmartDashboard.putNumber("get", get());
    }

    // public void setAngle(Supplier<Double> Rotation) {
    //     LeftTalon.setControl(request.withPosition(Rotation.get()).withSlot(1));
    //     RightTalon.setControl(request.withPosition(Rotation.get()).withSlot(1));
    // }

    public void resetEncoder() {
        RightTalon.setPosition(0);
        LeftTalon.setPosition(0);
    }
}


