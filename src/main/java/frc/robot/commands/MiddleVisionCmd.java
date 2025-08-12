package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Left_MiddleVisionSubsystem;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.Right_MiddleVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class MiddleVisionCmd extends Command {    
    private final SwerveSubsystem m_SwerveSubsystem;
    private final Right_MiddleVisionSubsystem m_RightMiddleVisionSubsystem;
    private final Left_MiddleVisionSubsystem m_LeftMiddleVisionSubsystem;

    private final DoubleSupplier xSpeedFunc;
    private final DoubleSupplier ySpeedFunc;
    private final DoubleSupplier zSpeedFunc;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;

    private double xSpeed;
    private double ySpeed;
    private double zSpeed;
    private boolean fieldOrient;
    
  public MiddleVisionCmd(Right_MiddleVisionSubsystem rightMiddleVisionSubsystem ,Left_MiddleVisionSubsystem leftMiddleVisionSubsystem , SwerveSubsystem swerveSubsystem,DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
    this.m_LeftMiddleVisionSubsystem = leftMiddleVisionSubsystem;
    this.m_RightMiddleVisionSubsystem = rightMiddleVisionSubsystem; 
    this.m_SwerveSubsystem = swerveSubsystem;

    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;

    this.xLimiter = new SlewRateLimiter(4.6);
    this.yLimiter = new SlewRateLimiter(4.6);
    this.zLimiter = new SlewRateLimiter(4.6);

    addRequirements(m_LeftMiddleVisionSubsystem,m_RightMiddleVisionSubsystem, m_SwerveSubsystem);

  }


  @Override
  public void initialize() {
    LEDConstants.arriveSetpoint_Base = false;
  }

  @Override
  public void execute() {
    m_LeftMiddleVisionSubsystem.LEDDD();
    if(m_LeftMiddleVisionSubsystem.lefthastarget()) {
      fieldOrient = false;

      xSpeed = m_LeftMiddleVisionSubsystem.getXOutput();
      ySpeed = -m_LeftMiddleVisionSubsystem.getYOutput();
      zSpeed = -m_LeftMiddleVisionSubsystem.getZOutput();

      if(m_LeftMiddleVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_LeftMiddleVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_LeftMiddleVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
      if (m_LeftMiddleVisionSubsystem.arriveXposition() && m_LeftMiddleVisionSubsystem.arriveYposition() && m_LeftMiddleVisionSubsystem.arriveRotationPosition()) {
        LEDConstants.arriveSetpoint_Base = true;
      } else {
        xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.kJoystickDeadBand);
        ySpeed = MathUtil.applyDeadband(ySpeed, OperatorConstants.kJoystickDeadBand);
        zSpeed = MathUtil.applyDeadband(zSpeed, OperatorConstants.kJoystickDeadBand);

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        zSpeed = zLimiter.calculate(zSpeed);
        
      }
    } 
    else {
      fieldOrient = true;

      this.xSpeed = -xSpeedFunc.getAsDouble() * 0.4;
      this.ySpeed = -ySpeedFunc.getAsDouble() * 0.4;
      this.zSpeed = -zSpeedFunc.getAsDouble() * 0.2;

      this.xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
      this.ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);
      this.zSpeed = MathUtil.applyDeadband(this.zSpeed, OperatorConstants.kJoystickDeadBand);

      this.xSpeed = xLimiter.calculate(this.xSpeed);
      this.ySpeed = yLimiter.calculate(this.ySpeed);
      this.zSpeed = zLimiter.calculate(this.zSpeed);
    }

    m_RightMiddleVisionSubsystem.LEDDD();
    if(m_RightMiddleVisionSubsystem.righthastarget()) {
      fieldOrient = false;

      xSpeed = m_RightMiddleVisionSubsystem.getXOutput();
      ySpeed = -m_RightMiddleVisionSubsystem.getYOutput();
      zSpeed = -m_RightMiddleVisionSubsystem.getZOutput();

      if(m_RightMiddleVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_RightMiddleVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_RightMiddleVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
      if (m_RightMiddleVisionSubsystem.arriveXposition() && m_RightMiddleVisionSubsystem.arriveYposition() && m_RightMiddleVisionSubsystem.arriveRotationPosition()) {
        LEDConstants.arriveSetpoint_Base = true;
      } else {
        xSpeed = MathUtil.applyDeadband(xSpeed, OperatorConstants.kJoystickDeadBand);
        ySpeed = MathUtil.applyDeadband(ySpeed, OperatorConstants.kJoystickDeadBand);
        zSpeed = MathUtil.applyDeadband(zSpeed, OperatorConstants.kJoystickDeadBand);

        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        zSpeed = zLimiter.calculate(zSpeed);
        
      }
    } 
    else {
      fieldOrient = true;

      this.xSpeed = -xSpeedFunc.getAsDouble() * 0.4;
      this.ySpeed = -ySpeedFunc.getAsDouble() * 0.4;
      this.zSpeed = -zSpeedFunc.getAsDouble() * 0.2;

      this.xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
      this.ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);
      this.zSpeed = MathUtil.applyDeadband(this.zSpeed, OperatorConstants.kJoystickDeadBand);

      this.xSpeed = xLimiter.calculate(this.xSpeed);
      this.ySpeed = yLimiter.calculate(this.ySpeed);
      this.zSpeed = zLimiter.calculate(this.zSpeed);
    }
    m_SwerveSubsystem.drive(xSpeed, ySpeed, zSpeed, fieldOrient);
  }

  @Override
  public void end(boolean interrupted) {
    LEDConstants.arriveSetpoint_Base = false;
    m_RightMiddleVisionSubsystem.arriveSetpoint();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
