package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }

  // public static Double[] optimate(double currentAngle, double goalAngle, double speedMetersPerSecond){
  //   Double[] goal = new Double[2];
  //   double delta = Math.abs(goalAngle - currentAngle);
  //   if (delta > (Math.PI / 2)) {
  //     goalAngle = goalAngle - Math.PI;
  //     speedMetersPerSecond = speedMetersPerSecond * -1;
  //   }
  //   goal[0] = goalAngle;
  //   goal[1] = speedMetersPerSecond;
  //   return goal;
  // }

  public static class Module_KrakenConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2RadPerSec = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;
    public static final double driveEncoderRot2RadPerMin = turningEncoderRot2RadPerSec*60;

    public static final double kModuleDistance = 0.546;
    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double turningPidController_Kp = 0.008;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0.0001;

    public static final double drivePidController_Kp = 0;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2;

  }

  public class Swerve_KrakenConstants {
    public static final int leftFrontDrive_ID = 7;
    public static final int leftBackDrive_ID = 8;
    public static final int rightFrontDrive_ID = 6;
    public static final int rightBackDrive_ID = 5;

    public static final int leftFrontTurning_ID = 3;
    public static final int leftBackTurning_ID = 4;
    public static final int rightFrontTurning_ID = 2;
    public static final int rightBackTurning_ID = 1;

    public static final int leftFrontAbsolutedEncoder_ID = 11;
    public static final int leftBackAbsolutedEncoder_ID = 12;
    public static final int rightFrontAbsolutedEncoder_ID = 10;
    public static final int rightBackAbsolutedEncoder_ID = 9;

    public static final double leftFrontOffset = 0.4714;
    public static final double leftBackOffset = -0.0515;
    public static final double rightFrontOffset = -0.2717;
    public static final double rightBackOffset = 0.0376;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;

    public static final double kModuleDistance = 22.24*0.0254;


    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2), 
      new Translation2d(kModuleDistance/2, -kModuleDistance/2), 
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;

    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 850;
  
  }

  public class PhotonConstants {
    public static final double xPidController_Kp = 0.6;
    public static final double xPidController_Ki = 0;
    public static final double xPidController_Kd = 0;

    public static final double yPidController_Kp = 0.6;
    public static final double yPidController_Ki = 0;
    public static final double yPidController_Kd = 0.001;

    public static final double rotationPidController_Kp = 0.004;
    public static final double rotationPidController_Ki = 0;
    public static final double rotationPidController_Kd = 0.0001;

    public static final double xPidMaxOutput = 0.4;
    public static final double yPidMaxOutput = 0.4;
    public static final double rotationPidMaxOutput = 0.4;
    public static final double xPidMaxOutput_NeedSlow = 0.2;
    public static final double yPidMaxOutput_NeedSlow = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow = 0.2;

    public static final double xPidMaxOutput_Reef = 0.4;
    public static final double yPidMaxOutput_Reef = 0.4;
    public static final double rotationPidMaxOutput_Reef = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Reef= 0.2;
    public static final double yPidMaxOutput_NeedSlow_Reef = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Reef = 0.2;

    public static final double xPidMaxOutput_CoralStation = 0.4;
    public static final double yPidMaxOutput_CoralStation = 0.4;
    public static final double rotationPidMaxOutput_CoralStation = 0.4;
    public static final double xPidMaxOutput_NeedSlow_CoralStation = 0.2;
    public static final double yPidMaxOutput_NeedSlow_CoralStation = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_CoralStation = 0.2;

    public static final double xPidMaxOutput_Cage = 0.4;
    public static final double yPidMaxOutput_Cage = 0.4;
    public static final double rotationPidMaxOutput_Cage = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Cage = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Cage = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Cage = 0.2;

    public static final double xPidMaxOutput_Net = 0.4;
    public static final double yPidMaxOutput_Net = 0.4;
    public static final double rotationPidMaxOutput_Net = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Net = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Net = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Net = 0.2;

    public static final double xPidMaxOutput_Processor = 0.4;
    public static final double yPidMaxOutput_Processor = 0.4;
    public static final double rotationPidMaxOutput_Processor = 0.4;
    public static final double xPidMaxOutput_NeedSlow_Processor = 0.2;
    public static final double yPidMaxOutput_NeedSlow_Processor = 0.2;
    public static final double rotationPidMaxOutput_NeedSlow_Processor = 0.2;

    public static final double xPidSetPoint_RightReef = 0.45; 
    public static final double yPidSetPoint_RightReef = -0.14;
    public static final double rotationPidSetPoint_RightReef = 183;

    public static final double xPidSetPoint_LeftReef = 0.446;
    public static final double yPidSetPoint_LeftReef = 0.1102;
    public static final double rotationPidSetPoint_LeftReef = 183.4;

    public static final double xPidSetPoint_MiddleReef_FrontRight = 0;
    public static final double yPidSetPoint_MiddleReef_FrontRight = 0;
    public static final double rotationPidSetPoint_MiddleReef_FrontRight = 182;

    public static final double xPidSetPoint_MiddleReef_FrontLeft = 0;
    public static final double yPidSetPoint_MiddleReef_FrontLeft = 0;
    public static final double rotationPidSetPoint_MiddleReef_FrontLeft = 0;

    public static final double xPidSetPoint_CoralStation_Back = 0;
    public static final double yPidSetPoint_CoralStation_Back = 0;
    public static final double rotationPidSetPoint_CoralStation_Back = 0;

    public static final double xPidSetPoint_CoralStation_FrontRight = 0;
    public static final double yPidSetPoint_CoralStation_FrontRight = 0;
    public static final double rotationPidSetPoint_CoralStation_FrontRight = 0;

    public static final double xPidSetPoint_Cage_FrontRight = 0;
    public static final double yPidSetPoint_Cage_FrontRight = 0;
    public static final double rotationPidSetPoint_Cage_FrontRight = 0;

    public static final double xPidSetPoint_Cage_FrontLeft = 0;
    public static final double yPidSetPoint_Cage_FrontLeft = 0;
    public static final double rotationPidSetPoint_Cage_FrontLeft = 0;

    public static final double xPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double yPidSetPoint_Cage_Back_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID20_ID11 = 0;

    public static final double xPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double yPidSetPoint_Cage_Back_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Cage_Back_ID21_ID10 = 0;

    public static final double xPidSetPoint_Processor_FrontRight = 0;
    public static final double yPidSetPoint_Processor_FrontRight = 0;
    public static final double rotationPidSetPoint_Processor_FrontRight = 0;

    public static final double xPidSetPoint_Processor_FrontLeft = 0;
    public static final double yPidSetPoint_Processor_FrontLeft = 0;
    public static final double rotationPidSetPoint_Processor_FrontLeft = 0;

    public static final double xPidSetPoint_Net_FrontRight = 0;
    public static final double yPidSetPoint_Net_FrontRight = 0;
    public static final double rotationPidSetPoint_Net_FrontRight = 0;

    public static final double xPidSetPoint_Net_FrontLeft = 0;
    public static final double yPidSetPoint_Net_FrontLeft = 0;
    public static final double rotationPidSetPoint_Net_FrontLeft = 0;

    public static final double xPidSetPoint_Net_Back_ID20_ID11 = 0;
    public static final double yPidSetPoint_Net_Back_ID20_ID11 = 0;
    public static final double rotationPidSetPoint_Net_Back_ID20_ID11 = 0;

    public static final double xPidSetPoint_Net_Back_ID21_ID10 = 0;
    public static final double yPidSetPoint_Net_Back_ID21_ID10 = 0;
    public static final double rotationPidSetPoint_Net_Back_ID21_ID10 = 0;

    public static final double arriveXPosition_Reef = 0;
    public static final double arriveXPosition_Cage = 0;
    public static final double arrivePosition_Net = 0;

    public static final double tooClosePosition_Reef = 0;
    public static final double tooClosePosition_Cage = 0;
    public static final double tooClosePosition_Net = 0;

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
  public static class VisionConstants{
    public static final double rightXSetpoint = 0.4233;
    public static final double rightYSetpoint = -0.3216;
    public static final double rightZSetpoint = -8;

    public static final double leftXSetpoint = 0;
    public static final double leftYSetpoint = 1;
    public static final double leftZSetpoint = 0;
  }
}
