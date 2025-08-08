package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {
  public static final class DriveConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 2.8;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 1.8 * Math.PI;//最大旋轉速度

    public static final double kTeleDriveMaxSpeedMetersPerSecond = (kPhysicalMaxSpeedMetersPerSecond / 4) * 3.2;//最大加速度 3.2
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.278, 0.278),
        new Translation2d(0.278, -0.278),
        new Translation2d(-0.278, 0.278),
        new Translation2d(-0.278, -0.278));

        public static final boolean FrontLeftdriveMotorReverse = false;
        public static final boolean BackLeftdriveMotorReverse = false;
        public static final boolean FrontRightdriveMotorReverse = true;
        public static final boolean BackRightdriveMotorReverse = true;
    
        public static final boolean FrontLeftTurningMotorReverse = true;
        public static final boolean BackLeftdTurningMotorReverse = true;
        public static final boolean FrontRightTurningMotorReverse = true;
        public static final boolean BackRightTurningMotorReverse = true;
    
        public static final int kFrontLeftDriveMotorPort = 5;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kFrontRightDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 7;
    
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 3;
    
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 11;
    
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final double WilliamConstant = 1.042;
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.10068;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
    public static final double kTurningGearRaitio = 1/19.6;
  }

  public static final class OIConstants {
          
    public static final int kDriverControllerPort = 0;
    public static final int kControlPort = 1;

    public static final int kDriverYAxis = 0;
    public static final int kDriverXAxis = 1;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 2;
    
    public static final double kDeadband = 0.1;
}

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }





  public static final class IntakeConstants{
    public static final double ksetpoint = 1;
    public static final double kresetpoint = 0;
  }

  public static final class ElevatorConstants{
    public static final double kreadyPosition = 0.0;
    public static final double ktakecoral = -1.0;
    public static final double kL4 = 4.0;
    public static final double kL3 = 3.0;
    public static final double kL2 = 2.0;
    public static final double kL1 = 1.0;
    public static final double knet = 5.0;
    public static final double kprocesser = 6.0;
    public static final double khaveAlgae = 7.0;
  }
  
  public static final class PawConstants{
    public static final double kreadyPosition = 0.0;
    public static final double kL1Position = 1.0;
    public static final double kL2Position = 2.0;
    public static final double kL3Position = 3.0;
    public static final double kL4Position = 4.0;
    public static final double knetPosition = 5.0;
    public static final double kprocesserPosition = 6.0;
    public static final double khaveAlgae = 7.0;
  }
}
