// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

public class LeftVisionSubsystem extends SubsystemBase {
  private PIDController xPID; 
  private PIDController yPID;
  private PIDController zPID;
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
  private final DoubleArraySubscriber doubleArray = table.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{0, 0, 0, 0, 0, 0});
  
  private final LEDPattern rainbow = LEDPattern.rainbow(255,255);
  private static final Distance LEDSpacing = Meters.of(1/120.0);
  private final LEDPattern scrollingRainbow =  rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2), LEDSpacing);
  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(100);
  private final LEDPattern Aqua = LEDPattern.solid(Color.kAqua);
  private final LEDPattern Green = LEDPattern.solid(Color.kDarkGreen);


  public LeftVisionSubsystem() {
      xPID = new PIDController(0.15, 0, 0.001);
      yPID = new PIDController(0.17, 0, 0.005);
      zPID = new PIDController(0.01, 0, 0);

      
      led.setLength(ledBuffer.getLength());
      led.start();
      arriveSetpoint();
    }

    public double getXOutput(double xSetpoint) {
        return Constants.setMaxOutput(xPID.calculate(getZ(),xSetpoint), 0.2);
    }
    public double getYOutput(double zSetpoint) {
        return Constants.setMaxOutput(yPID.calculate(getX(), zSetpoint), 0.2) ;
    }
    public double getZOutput(double zSetpoint) {
        return Constants.setMaxOutput(zPID.calculate(getRY(), zSetpoint), 0.1);
    }
  public double getX(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left").getX();
  }
  public double getZ(){
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left").getZ();
  }
  public double getRY(){
    return doubleArray.get()[4];
  }
  public boolean hastarget(){
    return LimelightHelpers.getTV("limelight-left");
  }

  public boolean Xposition(){
    return Math.abs(xPID.getError())<0.05;
  }
  public boolean Yposition(){
    return Math.abs(yPID.getError())<0.05;
  }
  public boolean ryPosition(){
    return Math.abs(zPID.getError())<1;
  }

  public void LED(){
    scrollingRainbow.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }
  public void AQLED(){
      Aqua.applyTo(ledBuffer);
      led.setData(ledBuffer);
  }
  public void GLED(){
    Green.applyTo(ledBuffer);
    led.setData(ledBuffer);
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
    if(Xposition()&Yposition()&&ryPosition()){
      AQLED();
      arriveSetpoint();
    }
    else{
        if(hastarget()){
          GLED();
          tracking_hasTarget();
        }
        else{
          LED();
          tracking_noTarget();
      }
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tx", getX());
    SmartDashboard.putNumber("Tz", getZ());
    SmartDashboard.putNumber("RY", getRY());
    SmartDashboard.putBoolean("hastarget", hastarget());
    SmartDashboard.putNumber("xoutput", getXOutput(VisionConstants.rightXSetpoint));
    SmartDashboard.putNumber("youtput", getYOutput(VisionConstants.rightYSetpoint));
    SmartDashboard.putNumber("RYoutput", getZOutput(VisionConstants.rightZSetpoint));
    
  }
}