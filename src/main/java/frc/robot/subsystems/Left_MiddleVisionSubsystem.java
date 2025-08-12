// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class Left_MiddleVisionSubsystem extends SubsystemBase {
  private PIDController xPID; 
  private PIDController yPID;
  private PIDController zPID;
  private final NetworkTable table;
  private final DoubleArraySubscriber botPose;

  public Left_MiddleVisionSubsystem() {
      xPID = new PIDController(0.15, 0, 0.001);
      yPID = new PIDController(0.17, 0, 0.005);
      zPID = new PIDController(0.01, 0, 0);

      table = NetworkTableInstance.getDefault().getTable("limelight-left");
      botPose = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{0, 0, 0, 0, 0, 0});

      arriveSetpoint();
    }

    public double getXOutput() {
        return Constants.setMaxOutput(xPID.calculate(getZ(), VisionConstants.left_middleReefXSetpoint), 0.2);
    }
    public double getYOutput() {
        return Constants.setMaxOutput(yPID.calculate(getX(), VisionConstants.left_middleReefYSetpoint), 0.2) ;
    }
    public double getZOutput() {
        return Constants.setMaxOutput(zPID.calculate(getRY(), VisionConstants.left_middleReefZSetpoint), 0.1);
    }
  public double getX(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left").getX();
  }
  public double getZ(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left").getZ();
  }
  public double getRY(){
    return botPose.get()[4];
  }

  public boolean lefthastarget(){
    return LimelightHelpers.getTV("limelight-left");
  }

  public boolean arriveXposition(){
    return Math.abs(xPID.getError())<0.05;
  }
  public boolean arriveYposition(){
    return Math.abs(yPID.getError())<0.05;
  }
  public boolean arriveRotationPosition(){
    return Math.abs(zPID.getError())<1;
  }

  public void tracking_hasTarget(){
    LimelightHelpers.setLEDMode_ForceOn("limelight-left");
  }

  public void tracking_noTarget(){
    LimelightHelpers.setLEDMode_ForceBlink("limelight-left");
  }

  public void arriveSetpoint(){
    LimelightHelpers.setLEDMode_ForceOff("limelight-left");
  }

  public void LEDDD(){
    if(arriveXposition() & arriveYposition() && arriveRotationPosition()){
      arriveSetpoint();
    }
    else{
        if(lefthastarget()){
          tracking_hasTarget();
        }
        else{
          tracking_noTarget();
      }
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tx", getX());
    SmartDashboard.putNumber("Tz", getZ());
    SmartDashboard.putNumber("RY", getRY());
    SmartDashboard.putBoolean("hastarget", lefthastarget());
    SmartDashboard.putNumber("xoutput", getXOutput());
    SmartDashboard.putNumber("youtput", getYOutput());
    SmartDashboard.putNumber("RYoutput", getZOutput());
    
  }
}