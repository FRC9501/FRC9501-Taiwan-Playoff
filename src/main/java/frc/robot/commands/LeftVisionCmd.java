package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LeftVisionSubsystem;

public class LeftVisionCmd extends Command {
    public double turnOutput;
    public double moveOutput;
    public double frontOutput;
    
    private final LeftVisionSubsystem m_LeftVisionSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;

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
    
  public LeftVisionCmd(LeftVisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
    this.m_LeftVisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;

    this.xLimiter = new SlewRateLimiter(4.6);
    this.yLimiter = new SlewRateLimiter(4.6);
    this.zLimiter = new SlewRateLimiter(4.6);

    addRequirements(m_LeftVisionSubsystem, m_SwerveSubsystem);

  }


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_LeftVisionSubsystem.LEDDD();
    if(m_LeftVisionSubsystem.hastarget()) {
      fieldOrient = false;
      xSpeed = m_LeftVisionSubsystem.getXOutput(leftXSetpoint);
      ySpeed = -m_LeftVisionSubsystem.getYOutput(leftYSetpoint);
      zSpeed = -m_LeftVisionSubsystem.getZOutput(leftZSetpoint);
      if(m_LeftVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_LeftVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
      }
    } 
    else {
      fieldOrient = true;
      this.xSpeed = -xSpeedFunc.getAsDouble();
      this.ySpeed = -ySpeedFunc.getAsDouble();
      this.zSpeed = -zSpeedFunc.getAsDouble();

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
    m_LeftVisionSubsystem.arriveSetpoint();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
