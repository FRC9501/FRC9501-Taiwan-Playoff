// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.management.MemoryType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Module_KrakenConstants;;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax turningMotor;
  private final SparkFlex driveMotor;

  private final SparkMaxConfig turningConfig;
  private final SparkFlexConfig driveConfig;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;

  private final PIDController turningPidController;
  private final SimpleMotorFeedforward driveFeedForward;

  public SwerveModule(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, double offset) {
    turningMotor = new SparkMax(turningMotor_ID,MotorType.kBrushless);
    driveMotor = new SparkFlex(driveMotor_ID, MotorType.kBrushless);

    turningConfig = new SparkMaxConfig();
    driveConfig = new SparkFlexConfig();

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();

    turningPidController = new PIDController(Module_KrakenConstants.turningPidController_Kp, Module_KrakenConstants.turningPidController_Ki, Module_KrakenConstants.turningPidController_Kd);
    turningPidController.enableContinuousInput(Module_KrakenConstants.pidRangeMin, Module_KrakenConstants.pidRangeMax);

    driveFeedForward = new SimpleMotorFeedforward(Module_KrakenConstants.driveFeedforward_Ks, Module_KrakenConstants.driveFeedforward_Kv);

    turningConfig.inverted(true);
    driveConfig.inverted(true);

    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint.Unsigned_0To1;
    cancoderConfig.MagnetSensor.MagnetOffset = offset;

    turningConfig.idleMode(IdleMode.kBrake);
    driveConfig.idleMode(IdleMode.kBrake);  
    
    turningMotor.configure(turningConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    absolutedEncoder.getConfigurator().apply(cancoderConfig);

    resetEncoder();
  }

  public void resetEncoder() {
    driveMotor.getEncoder().setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public double getDriveVelocity() {
    return driveMotor.getEncoder().getVelocity()*Module_KrakenConstants.driveEncoderRot2Meter;
  }

  public double getDrivePosition() {
    return driveMotor.getEncoder().getPosition();
  }

  public double getTurningPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurningMotorPosition(){
    return turningMotor.getEncoder().getPosition();
  }

  public double getTurningAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public void stopMotor() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    // Turn Motor
    state.optimize(getState().angle);
    double turningMotorOutput = turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
    turningMotor.set(turningMotorOutput);
    // Drive motor
    double driveMotorOutput = driveFeedForward.calculate(state.speedMetersPerSecond)/12;
    driveMotor.set(driveMotorOutput);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
