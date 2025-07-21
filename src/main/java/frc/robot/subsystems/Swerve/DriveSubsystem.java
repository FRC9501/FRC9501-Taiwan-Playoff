package frc.robot.subsystems.Swerve;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule[] modules = new MAXSwerveModule[4];
  private final AHRS gyro;
  private final SwerveDriveOdometry m_odometry;
  private final SwerveDrivePoseEstimator m_poseEstimator;

  public DriveSubsystem() {
       gyro = new AHRS(NavXComType.kMXP_SPI);

        modules[0] = new MAXSwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.FrontLeftdriveMotorReverse,
        DriveConstants.FrontLeftTurningMotorReverse,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        modules[1] = new MAXSwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.FrontRightdriveMotorReverse,
        DriveConstants.FrontRightTurningMotorReverse,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        modules[2] = new MAXSwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.BackLeftdriveMotorReverse,
        DriveConstants.BackLeftdTurningMotorReverse,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        modules[3] = new MAXSwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.BackRightdriveMotorReverse,
        DriveConstants.BackRightTurningMotorReverse,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    resetEncoders();
    zeroHeading();
        
    m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
          modules[0].getPosition(),
          modules[1].getPosition(),
          modules[2].getPosition(),
          modules[3].getPosition()
      });

        RobotConfig config = null;
        try{
          config = RobotConfig.fromGUISettings();
        }catch(Exception e) {
          e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getdriveRobotRelative, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                    new PIDConstants(2.8, 0.001, 0.006),  // 6,0,0
                    new PIDConstants(3.0, 0.005, 0.0)  // 5,0,0
            ),
            config,
            () -> {

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
     getRotation2d(),
     new SwerveModulePosition[] {
         modules[0].getPosition(),
         modules[1].getPosition(),
         modules[2].getPosition(),
         modules[3].getPosition()},
    getPose());
    }

  @Override
  public void periodic() {
    getVelocity();
    getTurning();
    updateToAdvantageScope();
    m_poseEstimator.update(getRotation2d(),      
    new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()});

    // System.out.println(gyro.getRotation2d().getDegrees());
    m_odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        });
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void getTurning() {
    SmartDashboard.putNumber("FL", modules[0].getTurningPosition());
    SmartDashboard.putNumber("FR", modules[1].getTurningPosition());
    SmartDashboard.putNumber("BL", modules[2].getTurningPosition());
    SmartDashboard.putNumber("BR", modules[3].getTurningPosition());
  }

  public void getVelocity() {
    SmartDashboard.putNumber("FL V", modules[0].getDriveVelocity());
    SmartDashboard.putNumber("FR V", modules[1].getDriveVelocity());
    SmartDashboard.putNumber("BL V", modules[2].getDriveVelocity());
    SmartDashboard.putNumber("BR V", modules[3].getDriveVelocity());

  }

  public ChassisSpeeds getdriveRobotRelative() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState());

  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
      setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)); 
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(0),//gyro.getAngle()),
        new SwerveModulePosition[] {
            modules[0].getPosition(),
            modules[1].getPosition(),
            modules[2].getPosition(),
            modules[3].getPosition()
        },
        pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        modules[0].setDesiredState(desiredStates[0]);
        modules[1].setDesiredState(desiredStates[1]);
        modules[2].setDesiredState(desiredStates[2]);
        modules[3].setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    modules[0].resetEncoders();
    modules[1].resetEncoders();
    modules[2].resetEncoders();
    modules[3].resetEncoders();
  }

  public void zeroHeading() {
    gyro.reset();
    //System.out.println(m_odometry.getPoseMeters().getRotation().getDegrees());
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }


  public void stopModules() {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();
  }

  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void get() {
    m_poseEstimator.getEstimatedPosition();
  }

  public void updateToAdvantageScope() {
    Pose2d poseA = new Pose2d();
    poseA = m_odometry.getPoseMeters();//m_poseEstimator.getEstimatedPosition();
    Logger.recordOutput("MyPose", m_odometry.getPoseMeters());

    SwerveModuleState[] states = new SwerveModuleState[]{
    modules[0].getState(),
    modules[1].getState(),
    modules[2].getState(),
    modules[3].getState()
  };

  Logger.recordOutput("SwerveModuleState", states);
  }
}