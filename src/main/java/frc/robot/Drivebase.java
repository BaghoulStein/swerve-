package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private static Drivebase instance;

  public enum controlMode {
    robot_oriented, field_oriented
  };

  public enum wheels {
    left_front, right_front,
    left_back, right_back
  };

  private SwerveModule[] swerveModules;
  private SwerveModuleState[] moduleStates;

  private AHRS NavX;

  private controlMode drive_mode;

  private SwerveDriveKinematics swerveKinematics;

  private ChassisSpeeds targetSpeeds;
  private SwerveDriveOdometry odometry;

  private Rotation2d angle;

  private Drivebase() {
    swerveModules = new SwerveModule[4];
    swerveModules[wheels.left_front.ordinal()] = new SwerveModule(RobotContainer.FLModule);
    swerveModules[wheels.right_front.ordinal()] = new SwerveModule(RobotContainer.FRModule);
    swerveModules[wheels.left_back.ordinal()] = new SwerveModule(RobotContainer.BLModule);
    swerveModules[wheels.right_back.ordinal()] = new SwerveModule(RobotContainer.BRModule);

    moduleStates = new SwerveModuleState[4];

    odometry = new SwerveDriveOdometry(swerveKinematics, angle,
        new SwerveModulePosition[] { swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
            swerveModules[2].getModulePosition(),
            swerveModules[3].getModulePosition() });


    NavX = new AHRS();

    swerveKinematics = SwerveModuleConstants.swerveKinematics;

    drive_mode = controlMode.field_oriented;

    calibrate();

    reset(new Pose2d());
    NavX.reset();
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public Pose2d getPose() {
    // return odometry.getPoseMeters();
    return new Pose2d();
  }

  public void resetOdometry(Pose2d initalPose2d) {
    resetOdometry(angle,
        getModulePositions(),
        initalPose2d);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
        swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition() };
  }

  public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  public void calibrate() {
    NavX.calibrate();
  }

  public void reset(Pose2d position) {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].reset();
      moduleStates[wheel.ordinal()] = new SwerveModuleState();
    }

    // prototype.reset();

    NavX.reset();
    NavX.setAngleAdjustment(position.getRotation().getDegrees());

    targetSpeeds = new ChassisSpeeds();

    angle = new Rotation2d();

  }

  public static Drivebase getInstance() {
    if (instance == null)
      instance = new Drivebase();
    return instance;
  }

  public void update() {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].update();
      moduleStates[wheel.ordinal()] = swerveModules[wheel.ordinal()].getCurrentState();
    }

    angle = NavX.getRotation2d();
    odometry.update(angle, getModulePositions());
    SmartDashboard.putNumber("chassis angle", angle.getDegrees());
  }

  public void setControlMode(controlMode drive_mode) {
    this.drive_mode = drive_mode;
  }

  public void setChassisSpeeds(ChassisSpeeds target_speeds) {
    this.targetSpeeds = target_speeds;

    SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(target_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    SwerveModuleState[] desiredStates;
    switch (drive_mode) {
      case field_oriented:
        this.targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle);
        desiredStates = swerveKinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle));
        break;
      default:
        this.targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        desiredStates = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;
    }
    System.out.println(this.targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
    // SwerveModuleState[] desiredStates =
    // swerve_kinematics.toSwerveModuleStates(field_oriented_target_speeds)
  }

  public void stop() {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].stop();
    }

    // prototype.stop();
  }

  public double getAngle() {
    return angle.getDegrees();
  }

  public void resetGyro() {
    NavX.reset();
  }

  public Rotation2d getAngleRotation() {
    return angle;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
