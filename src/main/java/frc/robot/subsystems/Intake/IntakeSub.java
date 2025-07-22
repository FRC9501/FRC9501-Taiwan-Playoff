package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
    // private final SparkMax motor2 = new SparkMax(16, MotorType.kBrushless);
    private AnalogInput analog = new AnalogInput(0);
    private SparkMax motor;
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig motor2cfg = new SparkMaxConfig();


    public IntakeSub() {
        intakeConfig.inverted(true).idleMode(IdleMode.kBrake);
        motor = new SparkMax(15, MotorType.kBrushless);
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // motor2.configure(motor2cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double Distance() {
        return analog.getValue();
    }

    public void motorSet(double speed) {
        motor.set(speed);
        if (Distance() > 650) {
            motor.set(0);
        } else {
            motor.set(speed);
        }

    }

    public void shoot(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

        
    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance", Distance());
    }
}
