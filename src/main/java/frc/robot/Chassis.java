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

public class Chassis extends SubsystemBase {
  private static Chassis instance;

  public enum control_mode {
    robot_oriented, field_oriented
  };

  public enum wheels {
    left_front, right_front,
    left_back, right_back
  };

  private SwerveModule[] swerve_modules;
  private SwerveModuleState[] current_states;

  private AHRS navx;

  private control_mode drive_mode;

  private SwerveDriveKinematics swerve_kinematics;

  private ChassisSpeeds target_speeds;
  private ChassisSpeeds field_oriented_target_speeds;
  private SwerveDriveOdometry odometry;

  private Rotation2d angle;

  private Chassis() {
    swerve_modules = new SwerveModule[4];
    swerve_modules[wheels.left_front.ordinal()] = new SwerveModule(RobotContainer.FLModule);
    swerve_modules[wheels.right_front.ordinal()] = new SwerveModule(RobotContainer.FRModule);
    swerve_modules[wheels.left_back.ordinal()] = new SwerveModule(RobotContainer.BLModule);
    swerve_modules[wheels.right_back.ordinal()] = new SwerveModule(RobotContainer.BRModule);

    current_states = new SwerveModuleState[4];

    // odometry = new SwerveDriveOdometry(swerve_kinematics, angle,
        // new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(),
            // new SwerveModulePosition() });


    navx = new AHRS();

    swerve_kinematics = SwerveModuleConstants.swerveKinematics;

    drive_mode = control_mode.field_oriented;

    calibrate();

    reset(new Pose2d());
    navx.reset();
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public Pose2d getPose() {
    // return odometry.getPoseMeters();
    return new Pose2d();
  }

  public void resetOdometry(Pose2d initalPose2d) {
    resetOdometry(navx.getRotation2d(),
        getModulePositions(),
        initalPose2d);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { swerve_modules[0].getModulePosition(), swerve_modules[1].getModulePosition(),
        swerve_modules[2].getModulePosition(), swerve_modules[3].getModulePosition() };
  }

  public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  public void calibrate() {
    navx.calibrate();
  }

  public void reset(Pose2d position) {
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].reset();
      current_states[wheel.ordinal()] = new SwerveModuleState();
    }

    // prototype.reset();

    navx.reset();
    navx.setAngleAdjustment(position.getRotation().getDegrees());

    target_speeds = new ChassisSpeeds();
    field_oriented_target_speeds = new ChassisSpeeds();

    angle = new Rotation2d();

  }

  public static Chassis getInstance() {
    if (instance == null)
      instance = new Chassis();
    return instance;
  }

  public void update() {
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].update();
      current_states[wheel.ordinal()] = swerve_modules[wheel.ordinal()].getState();
    }

    angle = navx.getRotation2d();
    // odometry.update(angle, getModulePositions());
    SmartDashboard.putNumber("chassis angle", angle.getDegrees());
  }

  public void setControlMode(control_mode drive_mode) {
    this.drive_mode = drive_mode;
  }

  public void setChassisSpeeds(ChassisSpeeds target_speeds) {
    this.target_speeds = target_speeds;

    SwerveModuleState[] desiredStates = swerve_kinematics.toSwerveModuleStates(target_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
  }

  public void setSpeeds(ChassisSpeeds target_speeds) {
    this.target_speeds = target_speeds;
    // constraintTargetSpeedsVelocity();
    // constraintTargetSpeedsAngularVelocity();
    // constraintTargetSpeedsAcceleration();
    // constraintTargetSpeedsAngularAcceleration();

    SwerveModuleState[] targets = getTargetModuleStates();
    SwerveDriveKinematics.desaturateWheelSpeeds(targets, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].set(targets[wheel.ordinal()]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    SwerveModuleState[] desiredStates;
    switch (drive_mode) {
      case field_oriented:
        this.target_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle);
        desiredStates = swerve_kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, angle));
        break;
      default:
        this.target_speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        desiredStates = swerve_kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;
    }
    System.out.println(this.target_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
    // SwerveModuleState[] desiredStates =
    // swerve_kinematics.toSwerveModuleStates(field_oriented_target_speeds)
  }

  public void stop() {
    for (wheels wheel : wheels.values()) {
      swerve_modules[wheel.ordinal()].stop();
    }

    // prototype.stop();
  }

  public double getAngle() {
    return angle.getDegrees();
  }

  public void resetGyro() {
    navx.reset();
  }

  public Rotation2d getAngleRotation() {
    return angle;
  }

  private SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] states;
    switch (drive_mode) {
      case robot_oriented:
        states = swerve_kinematics.toSwerveModuleStates(target_speeds);
        break;

      case field_oriented:
        field_oriented_target_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            target_speeds.vxMetersPerSecond, target_speeds.vyMetersPerSecond,
            target_speeds.omegaRadiansPerSecond, getAngleRotation());
        states = swerve_kinematics.toSwerveModuleStates(field_oriented_target_speeds);
        break;

      default:
        states = null;
        break;
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModuleConstants.freeSpeedMetersPerSecond);

    return states;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
