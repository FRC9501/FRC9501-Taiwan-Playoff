package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrackRightReef_Auto extends Command {    
    private final RightVisionSubsystem m_RightVisionSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;
    
  public TrackRightReef_Auto(RightVisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
    this.m_RightVisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_RightVisionSubsystem, m_SwerveSubsystem);

  }


  @Override
  public void initialize() {
    LEDConstants.arriveSetpoint_Base = false;
  }

  @Override
  public void execute() {
    m_RightVisionSubsystem.LEDDD();
    if(m_RightVisionSubsystem.hastarget()) {
      xSpeed = m_RightVisionSubsystem.getXOutput_RightReef();
      ySpeed = -m_RightVisionSubsystem.getYOutput_RightReef();
      zSpeed = -m_RightVisionSubsystem.getZOutput_RightReef();

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
