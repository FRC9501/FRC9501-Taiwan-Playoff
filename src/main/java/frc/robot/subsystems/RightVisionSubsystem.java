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

public class RightVisionSubsystem extends SubsystemBase {
  private PIDController xPID; 
  private PIDController yPID;
  private PIDController zPID;
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-right");
  private final DoubleArraySubscriber doubleArray = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{0, 0, 0, 0, 0, 0});
  
  // private final LEDPattern rainbow = LEDPattern.rainbow(255,255);
  // private static final Distance LEDSpacing = Meters.of(1/120.0);
  // private final LEDPattern scrollingRainbow =  rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDSpacing);
  // private AddressableLED led = new AddressableLED(0);
  // private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(100);
  // private final LEDPattern Aqua = LEDPattern.solid(Color.kAqua);
  // private final LEDPattern Green = LEDPattern.solid(Color.kDarkGreen);


  public RightVisionSubsystem() {
      xPID = new PIDController(0.17, 0, 0.001);
      yPID = new PIDController(0.19, 0, 0.005);
      zPID = new PIDController(0.01, 0, 0);

      
      // led.setLength(ledBuffer.getLength());
      // led.start();
      arriveSetpoint();
    }

  public double getXOutput_RightReef() {
        return Constants.setMaxOutput(xPID.calculate(getTZ(), VisionConstants.rightReefXSetpoint), 0.2);
  }
  public double getYOutput_RightReef() {
        return Constants.setMaxOutput(yPID.calculate(getTX(), VisionConstants.rightReefYSetpoint), 0.2) ;
  }
  public double getZOutput_RightReef() {
        return Constants.setMaxOutput(zPID.calculate(getRY(), VisionConstants.rightReefZSetpoint), 0.1);
  }
  public double getXOutput_MiddleReef() {
    return Constants.setMaxOutput(xPID.calculate(getTZ(), VisionConstants.right_MiddleReefXSetpoint), 0.2);
}
public double getYOutput_MiddleReef() {
    return Constants.setMaxOutput(yPID.calculate(getTX(), VisionConstants.right_MiddleReefYSetpoint), 0.2) ;
}
public double getZOutput_MiddleReef() {
    return Constants.setMaxOutput(zPID.calculate(getRY(), VisionConstants.right_MiddleReefZSetpoint), 0.1);
}
  public double getTX(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-right").getX();
  }
  public double getTZ(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-right").getZ();
  }
  public double getRY(){
    return doubleArray.get()[4];
  }
  public boolean hastarget(){
    return LimelightHelpers.getTV("limelight-right");
  }

  public boolean arriveXposition(){
    return Math.abs(xPID.getError()) <= 0.01;
  }
  public boolean arriveYposition(){
    return Math.abs(yPID.getError()) <= 0.01;
  }
  public boolean arriveRotationPosition(){
    return Math.abs(zPID.getError()) <= 1;
  }

  public void tracking_hasTarget(){
    LimelightHelpers.setLEDMode_ForceOn("limelight-right");
  }

  public void tracking_noTarget(){
    LimelightHelpers.setLEDMode_ForceBlink("limelight-right");
  }

  public void arriveSetpoint(){
    LimelightHelpers.setLEDMode_ForceOff("limelight-right");
  }

  public void LEDDD(){
    if(arriveXposition() && arriveYposition() && arriveRotationPosition()){
      arriveSetpoint();
    }else{
      if(hastarget()){
        tracking_hasTarget();
      }else{
        tracking_noTarget();
      }
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tx", getTX());
    SmartDashboard.putNumber("Tz", getTZ());
    SmartDashboard.putNumber("RY", getRY());
    SmartDashboard.putBoolean("hastarget", hastarget());
    SmartDashboard.putNumber("xoutput", getXOutput_RightReef());
    SmartDashboard.putNumber("youtput", getYOutput_RightReef());
    SmartDashboard.putNumber("RYoutput", getZOutput_RightReef());
    
  }
}