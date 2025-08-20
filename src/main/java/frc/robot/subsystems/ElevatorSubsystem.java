package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX leftTalon;
    private final TalonFX rightTalon;
    private final TalonFXConfiguration elevatorMotorConfig;
    private final MotionMagicVoltage request;
    private final MotionMagicConfigs elevatorMotionMagicConfigs;
    private double elevatorGoalPosition;
    private String mode = "start";
    


    public ElevatorSubsystem() {
        leftTalon = new TalonFX(ElevatorConstants.elevatorLeftMotorID);
        rightTalon = new TalonFX(ElevatorConstants.elevatorRightMotorID);


        elevatorGoalPosition = ElevatorConstants.coralPrimitivePosition;

        elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotionMagicConfigs = new MotionMagicConfigs();

        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;             
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorConfig.Slot1.withGravityType(GravityTypeValue.Elevator_Static);
        elevatorMotorConfig.Slot1.kP = 2;
        elevatorMotorConfig.Slot1.kI = 0;
        elevatorMotorConfig.Slot1.kD = 0;
        elevatorMotorConfig.Slot1.kG = 0.8;
        elevatorMotorConfig.Slot1.kA = 0;
        elevatorMotorConfig.Slot1.kV = 0;
        elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
        elevatorMotionMagicConfigs.MotionMagicAcceleration = 90;
        elevatorMotionMagicConfigs.MotionMagicJerk = 400;

        rightTalon.getConfigurator().apply(elevatorMotorConfig);
        leftTalon.getConfigurator().apply(elevatorMotorConfig);
        rightTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        leftTalon.getConfigurator().apply(elevatorMotionMagicConfigs);
        rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));

        resetEncoder();
        primitive_Coral();

        request = new MotionMagicVoltage(elevatorGoalPosition) ;
    }

    public void toSafePosition(){
        elevatorGoalPosition = ElevatorConstants.safePosition;
        mode = "toSafe";
    }
    public void primitive_Coral() {
        elevatorGoalPosition = ElevatorConstants.coralPrimitivePosition;
        mode = "toPrimitive_Coral";
    }
    public void intakeCoral() {
        elevatorGoalPosition = ElevatorConstants.coralIntakePosition;
        mode = "intakeCoral";
    }
    public void putL4() {
        elevatorGoalPosition = ElevatorConstants.coralL4Position;
        mode = "putL4";
    }
    public void putL3() {
        elevatorGoalPosition = ElevatorConstants.coralL3Position;
        mode = "putL3";
    }
    public void putL2() {
        elevatorGoalPosition = ElevatorConstants.coralL2Position;
        mode = "putL2";
    }
    public void putL1() {
        elevatorGoalPosition = ElevatorConstants.coralL1Position;
        mode = "putL1";
    }
    public void putNet() {
        elevatorGoalPosition = ElevatorConstants.algaeNetPosition;
        mode = "putNet";
    }
    public void putProcesser() {
        elevatorGoalPosition = ElevatorConstants.algaeProcessorPosition;
        mode = "putProcessor";
    }
    public void primitive_Algae() {
        elevatorGoalPosition = ElevatorConstants.algaePrimitivePosition; 
        mode = "toPrimitive_Algae";
    }
    public void intakeAlgaeHigh(){
        elevatorGoalPosition = ElevatorConstants.algaeHighPosition;
        mode = "intakeAlgae_High";
    }
    public void intakeAlgaeLow(){
        elevatorGoalPosition = ElevatorConstants.algaeLowPosition;
        mode = "intakeAlgae_Low";
    }
    public void intakeAlgaeFloor(){
        elevatorGoalPosition = ElevatorConstants.algaeFloorPosition;
        mode = "intakeAlgae_Floor";
    }

    public void resetEncoder() {
        rightTalon.setPosition(0);
        leftTalon.setPosition(0);
    }
    public boolean arriveSetpoint(){
        return Math.abs(getPosition() - elevatorGoalPosition) <= 0.1;
    }
    public boolean isSafe(){
        return getPosition() >= (ElevatorConstants.safePosition - 0.1);
    }
    public double getPosition() {
        return leftTalon.getPosition().getValueAsDouble();
    }

    public String getMode(){
        return mode;
    }



    @Override
    public void periodic() {
        if (getPosition() >= 12) {
            LEDConstants.isSlowMode = true;
        }else{
            LEDConstants.isSlowMode = false;
        }
        leftTalon.setControl(request.withPosition(elevatorGoalPosition).withSlot(1));
        SmartDashboard.putNumber("Elevator/Position", getPosition());
        SmartDashboard.putNumber("Elevator/Setpoint", elevatorGoalPosition);
        SmartDashboard.putString("Elevator/ElevatorMode", mode);
    }


    
    
}


