package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveModuleConstants {
  public static final double kTrackWidth = 0.55; // Distance between right and left wheels
  public static final double kWheelBase = 0.55; // Distance between front and back wheels
  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

  public static final double freeSpeedMetersPerSecond = 4;
  public static final double driveRatio = 6.75;
  public static final double steeringRatio = 12.5;
  public static final double wheelRadiusMeters = 0.0508; // 2 inches (in meters)
  public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI;
  public static final double driveDPRMeters = wheelCircumferenceMeters * driveRatio;
  public static final double steeringPositionConversionFactor = 1 / steeringRatio * 360; // degrees / rotation
  public static final double steeringVelocityConversionFactor = steeringPositionConversionFactor / 60; // degrees / (rotations * seconds/minute) 
  // public final static double cancoderTLOffset = 0;
  // public final static double cancoderTROffset = 0;
  // public final static double cancoderBLOffset = 0;
  // public final static double cancoderBROffset = 0;

  public final static double cancoderTLOffset = 143.701;
  public final static double cancoderTROffset = 274.921;
  public final static double cancoderBLOffset = 235.283;
  public final static double cancoderBROffset = 338.642;

  public final int idDrive;
  public final PIDFGains driveGains;
  public final int idSteering;
  public final PIDFGains steeringGains;
  public final double cancoderZeroAngle;
  public final int canCoderId;
  public final boolean isSteeringInverted;
  public final boolean isDriveInverted;

  public SwerveModuleConstants(int idDrive, int idSteering, double cancoderZeroAngle,
      int canCoderId, boolean isSteeringInverted, boolean isDriveInverted) {
    this(idDrive, idSteering, new PIDFGains(0.05, 0, 0, 0, 1, 0),
        new PIDFGains(0.2, 0, 0, 0, 1, 0), cancoderZeroAngle, canCoderId, isSteeringInverted, isDriveInverted);
  }

  public SwerveModuleConstants(int idDrive, int idSteering, PIDFGains driveGains,
      PIDFGains steeringGains, double cancoderZeroAngle, int canCoderId, boolean isSteeringInverted, boolean isDriveInverted) {
    this.idDrive = idDrive;
    this.driveGains = driveGains;
    this.idSteering = idSteering;
    this.steeringGains = steeringGains;
    this.cancoderZeroAngle = cancoderZeroAngle;
    this.canCoderId = canCoderId;
    this.isSteeringInverted = isSteeringInverted;
    this.isDriveInverted = isDriveInverted;
  }
}
