// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LeftVisionSubsystem;

public class TrackLeftReef_Auto extends Command {
    
    private final LeftVisionSubsystem m_LeftVisionSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;
    
  public TrackLeftReef_Auto(LeftVisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.m_LeftVisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_LeftVisionSubsystem, m_SwerveSubsystem);

  }


  @Override
  public void initialize() {
    LEDConstants.arriveSetpoint_Base = false;
  }

  @Override
  public void execute() {
    m_LeftVisionSubsystem.LEDDD();

    if(m_LeftVisionSubsystem.hastarget()) {
      xSpeed = m_LeftVisionSubsystem.getXOutput_Leftreef();
      ySpeed = -m_LeftVisionSubsystem.getYOutput_Leftreef();
      zSpeed = -m_LeftVisionSubsystem.getZOutput_Leftreef();
      if(m_LeftVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
      if(m_LeftVisionSubsystem.arriveXposition()&&m_LeftVisionSubsystem.arriveYposition()&&m_LeftVisionSubsystem.arriveRotationPosition()) {
        LEDConstants.arriveSetpoint_Base = true;
      }
    }else{
      xSpeed = 0;
      ySpeed = 0;
      zSpeed = 0;
    } 
    m_SwerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);

  }

  @Override
  public void end(boolean interrupted) {
    m_LeftVisionSubsystem.arriveSetpoint();
    LEDConstants.arriveSetpoint_Base = false;
  }


  @Override
  public boolean isFinished() {
    return LEDConstants.arriveSetpoint_Base;
  }
}
