package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX leftTalon;
    private TalonFX rightTalon;
    private TalonFXConfiguration elevatorMotorConfig;
    private PositionDutyCycle request;
    private MotionMagicConfigs elevatorMotionMagicConfigs;
    // private DutyCycleOut duty = new DutyCycleOut(0.2);
    private double elevatorGoalPosition;


    public ElevatorSubsystem() {
        leftTalon = new TalonFX(ElevatorConstants.elevatorLeftMotorID);
        rightTalon = new TalonFX(ElevatorConstants.elevatorRightMotorID);

        elevatorGoalPosition = ElevatorConstants.coralPrimitivePosition;

        elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotionMagicConfigs = new MotionMagicConfigs();

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

        rightTalon.getConfigurator().apply(elevatorMotorConfig);
        leftTalon.getConfigurator().apply(elevatorMotorConfig);
        rightTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        leftTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        leftTalon.setControl(new Follower(rightTalon.getDeviceID(), true));

        resetEncoder();
        primitive_Coral();

        request = new PositionDutyCycle(elevatorGoalPosition);
    }

    public void primitive_Coral() {
        elevatorGoalPosition = ElevatorConstants.coralPrimitivePosition;
    }
    public void intakeCoral() {
        elevatorGoalPosition = ElevatorConstants.coralIntakePosition;
    }
    public void putL4() {
        elevatorGoalPosition = ElevatorConstants.coralL4Position;
    }
    public void putL3() {
        elevatorGoalPosition = ElevatorConstants.coralL3Position;
    }
    public void putL2() {
        elevatorGoalPosition = ElevatorConstants.coralL2Position;
    }
    public void putL1() {
        elevatorGoalPosition = ElevatorConstants.coralL1Position;
    }
    public void putNet() {
        elevatorGoalPosition = ElevatorConstants.algaeNetPosition;
    }
    public void putProcesser() {
        elevatorGoalPosition = ElevatorConstants.algaeProcessorPosition;
    }
    public void primitive_Algae() {
        elevatorGoalPosition = ElevatorConstants.algaePrimitivePosition; 
    }
    public void intakeAlgaeHigh(){
        elevatorGoalPosition = ElevatorConstants.algaeHighPosition;
    }
    public void intakeAlgaeLow(){
        elevatorGoalPosition = ElevatorConstants.algaeLowPosition;
    }
    public void intakeAlgaeFloor(){
        elevatorGoalPosition = ElevatorConstants.algaeFloorPosition;
    }

    public void resetEncoder() {
        rightTalon.setPosition(0);
        leftTalon.setPosition(0);
    }
    public boolean arriveSetpoint(){
        return Math.abs(getPosition() - elevatorGoalPosition) <= 0.1;
    }
    public double getPosition() {
        return rightTalon.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        rightTalon.setControl(request.withPosition(elevatorGoalPosition).withSlot(1));
    }


    
    
}


