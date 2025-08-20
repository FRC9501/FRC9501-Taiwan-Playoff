// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LeftVisionSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackMiddleReef_Auto extends Command {    
    private final SwerveSubsystem m_SwerveSubsystem;
    private final RightVisionSubsystem m_RightVisionSubsystem;
    private final LeftVisionSubsystem m_LeftVisionSubsystem;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;
    
  public TrackMiddleReef_Auto(RightVisionSubsystem rightVisionSubsystem ,LeftVisionSubsystem leftVisionSubsystem , SwerveSubsystem swerveSubsystem) {
    this.m_LeftVisionSubsystem = leftVisionSubsystem;
    this.m_RightVisionSubsystem = rightVisionSubsystem; 
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_LeftVisionSubsystem, m_RightVisionSubsystem, m_SwerveSubsystem);
  }


  @Override
  public void initialize() {
    LEDConstants.arriveSetpoint_Base = false;
  }

  @Override
  public void execute() {
    m_LeftVisionSubsystem.LEDDD();
    m_RightVisionSubsystem.LEDDD();
    if(m_LeftVisionSubsystem.hastarget()) {
      xSpeed = m_LeftVisionSubsystem.getXOutput_MiddleReef();
      ySpeed = -m_LeftVisionSubsystem.getYOutput_MiddleReef();
      zSpeed = -m_LeftVisionSubsystem.getZOutput_MiddleReef();

      if(m_LeftVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
      if (m_LeftVisionSubsystem.arriveXposition() && m_LeftVisionSubsystem.arriveYposition() && m_LeftVisionSubsystem.arriveRotationPosition()) {
        LEDConstants.arriveSetpoint_Base = true;
      }
    } else if(m_RightVisionSubsystem.hastarget()) {
      xSpeed = m_RightVisionSubsystem.getXOutput_MiddleReef();
      ySpeed = -m_RightVisionSubsystem.getYOutput_MiddleReef();
      zSpeed = -m_RightVisionSubsystem.getZOutput_MiddleReef();

      if(m_RightVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_RightVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_RightVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
      if (m_RightVisionSubsystem.arriveXposition() && m_RightVisionSubsystem.arriveYposition() && m_RightVisionSubsystem.arriveRotationPosition()) {
        LEDConstants.arriveSetpoint_Base = true;
      }
    } else {
      xSpeed = 0;
      ySpeed = 0;
      zSpeed = 0;
    }
    m_SwerveSubsystem.drive(xSpeed, ySpeed, zSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_RightVisionSubsystem.arriveSetpoint();
  }


  @Override
  public boolean isFinished() {
    return LEDConstants.arriveSetpoint_Base;
  }
}
