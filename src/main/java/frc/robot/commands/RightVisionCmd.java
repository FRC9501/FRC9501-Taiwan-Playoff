package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.RightVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RightVisionCmd extends Command {
    public double turnOutput;
    public double moveOutput;
    public double frontOutput;
    
    private final RightVisionSubsystem m_RightVisionSubsystem;
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
    
  public RightVisionCmd(RightVisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem,DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed) {
    this.m_RightVisionSubsystem = visionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;

    this.xLimiter = new SlewRateLimiter(4.6);
    this.yLimiter = new SlewRateLimiter(4.6);
    this.zLimiter = new SlewRateLimiter(4.6);

    addRequirements(m_RightVisionSubsystem, m_SwerveSubsystem);

  }


  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_RightVisionSubsystem.LEDDD();
    if(m_RightVisionSubsystem.hastarget()) {
      fieldOrient = false;

      xSpeed = m_RightVisionSubsystem.getXOutput(VisionConstants.rightXSetpoint);
      ySpeed = -m_RightVisionSubsystem.getYOutput(VisionConstants.rightYSetpoint);
      zSpeed = -m_RightVisionSubsystem.getZOutput(VisionConstants.rightZSetpoint);

      if(m_RightVisionSubsystem.arriveXposition()){
        xSpeed = 0;
      }
      if(m_RightVisionSubsystem.arriveYposition()){
        ySpeed = 0;
      }
      if(m_RightVisionSubsystem.arriveRotationPosition()){
        zSpeed=0;
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
    m_RightVisionSubsystem.arriveSetpoint();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
