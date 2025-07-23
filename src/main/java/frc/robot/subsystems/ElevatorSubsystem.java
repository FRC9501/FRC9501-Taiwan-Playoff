package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX leftTalon;
    private TalonFX rightTalon;
    private TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
    private PositionDutyCycle request = new PositionDutyCycle(0);
    private MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();
    // private DutyCycleOut duty = new DutyCycleOut(0.2);
    private double elevatorGoalPosition;


    public ElevatorSubsystem() {
        leftTalon = new TalonFX(13);
        rightTalon = new TalonFX(14);

        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;             
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorConfig.Slot1.withGravityType(GravityTypeValue.Elevator_Static);
        elevatorMotorConfig.Slot1.kP = 0;
        elevatorMotorConfig.Slot1.kI = 0;
        elevatorMotorConfig.Slot1.kD = 0;
        elevatorMotorConfig.Slot1.kG = 0;
        elevatorMotorConfig.Slot1.kA = 0;
        elevatorMotorConfig.Slot1.kV = 0;
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = 90;
        elevatorMotionMagicConfigs.MotionMagicJerk = 400;
        // leftTalonCfg.MotorOutput.PeakForwardDutyCycle = 0.6;
        // leftTalonCfg.MotorOutput.PeakReverseDutyCycle = -0.6;
        // leftTalonCfg.Slot1.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        // elevatorMotorConfig.Slot1.withGravityType(GravityTypeValue.Elevator_Static);
        // elevatorMotorConfig.Slot1.kP = 0.02;
        // elevatorMotorConfig.Slot1.kG = 0.0;
        // elevatorMotorConfig.Slot1.kI = 0;
        // elevatorMotorConfig.Slot1.kD = 0;
        // elevatorMotorConfig.Slot1.kA = 0;
        // elevatorMotorConfig.Slot1.kV = 0;
        // elevatorMotorConfig.MotorOutput.PeakForwardDutyCycle = 0.6;
        // elevatorMotorConfig.MotorOutput.PeakReverseDutyCycle = -0.6;
        // elevatorMotorConfig.Slot1.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        rightTalon.getConfigurator().apply(elevatorMotorConfig);
        leftTalon.getConfigurator().apply(elevatorMotorConfig);
        rightTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        leftTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        leftTalon.setControl(new Follower(rightTalon.getDeviceID(), true));

        resetEncoder();

    }

    // public void Duty() {
    //     rightTalon.set(0.3);
    //     leftTalon.set(0.3);
    // }

    // public void Reverse() {
    //     rightTalon.set(-0.3);
    //     leftTalon.set(-0.3);
    // }
    // public double get() {
    //     return rightTalon.getPosition().getValueAsDouble();
    // }
    public boolean setpoint(){
        return Math.abs(nowposition() - elevatorGoalPosition) <= 0.1;
    }
    public double nowposition() {
        return rightTalon.getPosition().getValueAsDouble();
    }
    public void stop() {
        rightTalon.set(0);
        leftTalon.set(0);
    }


    public void readyPosition() {
        elevatorGoalPosition = 0.0;
    }
    public void takecoral() {
        elevatorGoalPosition = -1;
    }

    public void L4() {
        elevatorGoalPosition = 4.0;
    }

    public void L3() {
        elevatorGoalPosition = 3.0;
    }

    public void L2() {
        elevatorGoalPosition = 2.0;
    }

    public void L1() {
        elevatorGoalPosition = 1.0;
    }
    public void Net() {
        elevatorGoalPosition = 5.0;
    }
    public void processer() {
        elevatorGoalPosition = 7.0;
    }

    public void resetEncoder() {
        rightTalon.setPosition(0);
        leftTalon.setPosition(0);
    }

    @Override
    public void periodic() {
        rightTalon.setControl(request.withPosition(elevatorGoalPosition).withSlot(1));
    }

    // public void setAngle(Supplier<Double> Rotation) {
    //     leftTalon.setControl(request.withPosition(Rotation.get()).withSlot(1));
    //     rightTalon.setControl(request.withPosition(Rotation.get()).withSlot(1));
    // }

    
    
}


