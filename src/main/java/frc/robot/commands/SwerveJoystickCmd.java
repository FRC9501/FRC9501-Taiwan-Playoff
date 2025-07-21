package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class SwerveJoystickCmd extends Command{
    private final DriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFuntion, ySpeedFunction, turningSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(DriveSubsystem swerveSubsystem, Supplier<Double> xSupplier, Supplier<Double>ySupplier, Supplier<Double> turningSupplier, Supplier<Boolean> fieldOrientedSupplier){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFuntion = xSupplier;
        this.ySpeedFunction = ySupplier;
        this.turningSpeedFunction = turningSupplier;
        this.fieldOrientedFunction = fieldOrientedSupplier;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double xSpeed = xSpeedFuntion.get();
        double ySpeed = ySpeedFunction.get();
        double turningSpeed = turningSpeedFunction.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        if(xSpeed == 0 && ySpeed == 0 && turningSpeed == 0){
            swerveSubsystem.stopModules();
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        if(fieldOrientedFunction.get()){    
            chassisSpeeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed,swerveSubsystem.getRotation2d()),0.02);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

// 宣告要使用的Subsystem，運算 