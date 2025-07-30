package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax intakeMotor = new SparkMax(10, MotorType.kBrushless);
    private SparkMax intakeMotor2 = new SparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    private CANcoder intakeCANcoder = new CANcoder(20);
    private PIDController intakePID = new PIDController(0.1, 0.01, 0.001);
    private double nowposition = 0.0;
    private double position = 0.0;

    public IntakeSubsystem() {      
    }
    
    public void intakePIDMove(double setpoint){
        intakeMotor.set(intakePID.calculate(nowposition,setpoint));
    }
    public double get(){
        return nowposition = intakeCANcoder.getPosition().getValueAsDouble();
    }
    public void suck(){
        intakeMotor2.set(1);
    }
    public void stop(){
        intakeMotor2.set(0);
    }

    @Override
    public void periodic() {
        position = encoder.getPosition();
        get();
    }
}
