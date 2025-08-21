package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kpanelPort = 1;
  }

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }

  public static class ModuleConstants {

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

    public static final double turningPidController_Kp = 0.008;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0.0001;

    public static final double drivePidController_Kp = 0;
    public static final double drivePidController_Ki = 0;
    public static final double drivePidController_Kd = 0;

    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2;

  }

  public class SwerveConstants {
    public static final int leftFrontDrive_ID = 2;//2
    public static final int leftBackDrive_ID = 3;//3
    public static final int rightFrontDrive_ID = 1;//1
    public static final int rightBackDrive_ID = 4;//4

    public static final int leftFrontTurning_ID = 6;
    public static final int leftBackTurning_ID = 7;
    public static final int rightFrontTurning_ID = 5;
    public static final int rightBackTurning_ID = 8;

    public static final int leftFrontAbsolutedEncoder_ID = 10;
    public static final int leftBackAbsolutedEncoder_ID = 11;
    public static final int rightFrontAbsolutedEncoder_ID = 9;
    public static final int rightBackAbsolutedEncoder_ID = 12;

    public static final double leftFrontOffset = 0.480224;
    public static final double leftBackOffset = 0.041503;
    public static final double rightFrontOffset = -0.410644;
    public static final double rightBackOffset = -0.140625;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/5.95;
    public static final double turningGearRatio = 1.0/21;

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;

    public static final double kModuleDistance = 0.546;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
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

    public static final double maxDriveSpeed_MeterPerSecond = 5;
    public static final double maxAngularVelocity_Angle = 850;
  
  }

  public static final class ElevatorConstants{
    public static final int elevatorRightMotorID = 13;
    public static final int elevatorLeftMotorID = 14;

    public static final double safePosition = 9.12;
    public static final double coralPrimitivePosition = 8.78;
    public static final double coralIntakePosition = 7.83;
    public static final double coralL4Position  = 19.23;//19.03
    public static final double coralL4RealPosition = 19.03;
    public static final double coralL3Position  = 6.98;
    public static final double coralL2Position  = 0;
    public static final double coralL1Position  = 10.59;
    public static final double algaePrimitivePosition = 0;
    public static final double algaeHighPosition = 20.54;
    public static final double algaeLowPosition = 9.77;
    public static final double algaeFloorPosition = 0;
    public static final double algaeNetPosition  = 21.41;
    public static final double algaeProcessorPosition  = 0;
  }

  
  public static final class ArmConstants{
    public static final int armPivotMotorID = 15;
    public static final int armWheelMotorID = 16;
    public static final int armAbsoluteEncoderID = 0;
    public static final int armRelativeEncoderFirstID = 1;
    public static final int armRelativeEncoderSecondID = 2;

    public static final double armAbsoluteEncoderOffset = -0.169;

    public static final double coralPrimitivePosition = 92.9;
    public static final double coralIntakePosition = 92.9;
    public static final double coralL1PutPosition = 140.76;
    public static final double coralL2PutPosition = 232.11;
    public static final double coralL3PutPosition = 228.46;
    public static final double coralL4PutPosition = 226.47;
    public static final double algaeHighPosition = 167.8;
    public static final double algaeLowPosition = 179.36;
    public static final double algaeFloorPosition = 270;
    public static final double algaeNetPosition = 298.18;
    public static final double algaeProcessorPosition = 186.36;
    public static final double algaePrimitivePosition = 255;

    public static final double putL1Vol = 0;
    public static final double putL2Vol = 0;
    public static final double putL3Vol = 0;
    public static final double putL4Vol = -0.6;
    public static final double intakeCoralVol = 3;
    public static final double intakeAlgaeFloorVol = 12;
    public static final double intakeAlgaeHighVol = 12;
    public static final double intakeAlgaeLowVol = 12;
    public static final double holdAlgaeVol = 12;
    public static final double putProcessorVol = 0;
    public static final double putNetVol = 0;
  }
  public static class VisionConstants{
    public static final double rightReefXSetpoint = 0.4233;
    public static final double rightReefYSetpoint = -0.3216;
    public static final double rightReefZSetpoint = -8;

    public static final double leftReefXSetpoint = 0;
    public static final double leftReefYSetpoint = 0;
    public static final double leftReefZSetpoint = 0;

    public static final double right_MiddleReefXSetpoint = 0;
    public static final double right_MiddleReefYSetpoint = 0;
    public static final double right_MiddleReefZSetpoint = 0;
    public static final double left_MiddleReefXSetpoint = 0;
    public static final double left_MiddleReefYSetpoint = 0;
    public static final double left_MiddleReefZSetpoint = 0;

  }
  public static class LEDConstants{
    public static boolean arriveSetpoint_Base = false;
    public static boolean arriveSetpoint_Arm = false;
    public static boolean arriveSetpoint_Elevator = false;
    public static boolean hasGamePiece = false;
    public static boolean hasTarget = false;
    public static boolean isAlgaeMode = false;
    public static boolean isSlowMode = false;
  }
}
