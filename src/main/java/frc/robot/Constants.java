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
    public static final int elevatorRightMotorID = 0;
    public static final int elevatorLeftMotorID = 0;

    public static final double coralPrimitivePosition = 0;
    public static final double coralIntakePosition = 0;
    public static final double coralL4Position  = 0;
    public static final double coralL3Position  = 0;
    public static final double coralL2Position  = 0;
    public static final double coralL1Position  = 0;
    public static final double algaePrimitivePosition = 0;
    public static final double algaeHighPosition = 0;
    public static final double algaeLowPosition = 0;
    public static final double algaeFloorPosition = 0;
    public static final double algaeNetPosition  = 0;
    public static final double algaeProcessorPosition  = 0;
  }

  public static final class IntakeConstants{
    public static final int intakePivotRightMotorID = 0;
    public static final int intakePivotLeftMotorID = 0;
    public static final int intakeWheelMotorID = 0;
    public static final int intakeCANcoderID = 0;

    public static final double intakeCANcoderOffset = 0;

    public static final double intakeOutPosition = 0;
    public static final double intakePrimitivePosition = 0;

    public static final double intakeCoralVol = 0;
    public static final double rejectCoralVol = 0;
  }
  
  public static final class ArmConstants{
    public static final int armPivotMotorID = 0;
    public static final int armWheelMotorID = 0;
    public static final int armCANcoderID = 0;
    public static final int armIRSensorID = 0;

    public static final double paeCANcoderOffset = 0;

    public static final double coralPrimitivePosition = 0;
    public static final double coralIntakePosition = 0;
    public static final double coralL1Position = 0;
    public static final double coralL2Position = 0;
    public static final double coralL3Position = 0;
    public static final double coralL4Position = 0;
    public static final double algaeHighPosition = 0;
    public static final double algaeLowPosition = 0;
    public static final double algaeFloorPosition = 0;
    public static final double algaeNetPosition = 0;
    public static final double algaeProcessorPosition = 0;
    public static final double algaePrimitivePosition = 0;

    public static final double intakeCoralVol = 0;
    public static final double intakeAlgaeFloorVol = 0;
    public static final double intakeAlgaeHighVol = 0;
    public static final double intakeAlgaeLowVol = 0;
    public static final double holdAlgaeVol = 0;
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
  }
}
