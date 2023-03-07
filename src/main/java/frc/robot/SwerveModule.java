package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The periodic function for the SwerveModule.
 */
public class SwerveModule extends SubsystemBase {
  private TalonFX m_driveMotor;
  private CANSparkMax m_steeringMotor;
  private CANCoder absEncoder;

  private double m_steerRotations;
  public String moduleName;
  private SwerveModuleState targetState;
  private SwerveModuleState currentState;

  public SwerveModule(SwerveModuleConstants cModuleConstants) {
    absEncoder = configCANCoder(cModuleConstants.canCoderId, cModuleConstants.cancoderZeroAngle);
    m_driveMotor = configTalonFX(cModuleConstants.idDrive, cModuleConstants.driveGains, cModuleConstants.isDriveInverted);
    m_steeringMotor = configSparkMax(cModuleConstants.idSteering, cModuleConstants.steeringGains,
        cModuleConstants.isSteeringInverted);

    m_steerRotations = 0;
    targetState = new SwerveModuleState();
    currentState = new SwerveModuleState();

    switch (cModuleConstants.idDrive) {
      case 2:
        moduleName = "FRONT LEFT ";
        break;
      case 4:
        moduleName = "FRONT RIGHT ";
        break;
      case 6:
        moduleName = "BACK LEFT ";
        break;
      case 8:
        moduleName = "BACK RIGHT ";
        break;
      default:
        moduleName = "undefined ";
        break;
    }
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Updates the state of the module.
   */
  public void update() {
    SmartDashboard.putNumber(moduleName + "Cancoder position", getAbsolutePosition());
    SmartDashboard.putNumber(moduleName + "Neo encoder position",
        m_steeringMotor.getEncoder().getPosition() * SwerveModuleConstants.steeringPositionConversionFactor);
    m_steerRotations = m_steeringMotor.getEncoder().getPosition() / SwerveModuleConstants.steeringRatio;
    currentState.angle = Rotation2d.fromDegrees(m_steerRotations * 360.0);
    currentState.speedMetersPerSecond = getRPS() * SwerveModuleConstants.wheelCircumferenceMeters
        / SwerveModuleConstants.driveRatio;
  }

  /**
   * Configures a CANSparkMax with the given gains and inversion.
   * @param id The ID of the CANSparkMax.
   * @param gains The PIDFGains to use.
   * @param isInverted Whether the CANSparkMax should be inverted.
   * @return The configured CANSparkMax.
   */
  private CANSparkMax configSparkMax(int id,
      PIDFGains gains, boolean isInverted) {
    CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
    sparkMax.getPIDController().setP(gains.getP());
    sparkMax.getPIDController().setI(gains.getI());
    sparkMax.getPIDController().setD(gains.getD());
    sparkMax.getPIDController().setIZone(gains.getIZone());
    sparkMax.setInverted(isInverted);
    sparkMax.getPIDController().setOutputRange(-1, 1);
    sparkMax.setSmartCurrentLimit(40);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setClosedLoopRampRate(0.01);
    sparkMax.enableVoltageCompensation(12);
    sparkMax.getEncoder().setPosition(absEncoder.getAbsolutePosition() /
        SwerveModuleConstants.steeringPositionConversionFactor);
    // sparkMax.getEncoder().setPosition(0);

    return sparkMax;
  }

  /**
  * Configures the CANCoder with the given ID.
  * @param id The ID of the CANCoder to configure.
  * @param zeroAngle The zero angle of the CANCoder.
  * @return The configured CANCoder.
   */
  private CANCoder configCANCoder(int id, double zeroAngle) {

    CANCoder canCoder = new CANCoder(id);
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    // Configure the offset angle of the magnet
    canCoder.configMagnetOffset(360 - zeroAngle);

    return canCoder;
  }

  /**
   * Resets the motors to their default position.
   */
  public void resetMotors() {
    m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_steeringMotor.set(0);
  }

  /**
   * Resets the motors and the state of the swerve drivetrain.
   */
  public void reset() {
    resetMotors();

    m_steerRotations = 0;
    targetState = new SwerveModuleState();
    currentState = new SwerveModuleState();
  }

  /**
   * Configures TalonFX with the gains from the config file.
   * @param id The ID of the TalonFX to configure.
   * @param gains The PIDFGains to use.
   * @param isInverted Whether the TalonFX is inverted.
   * @return The configured TalonFX.
   */
  private TalonFX configTalonFX(int id, PIDFGains gains, boolean isInverted) {
    TalonFX talon = new TalonFX(id);
    talon.config_kP(0, gains.getP());
    talon.config_kI(0, gains.getI());
    talon.config_kD(0, gains.getD());
    talon.config_kF(0, gains.getF());
    talon.config_IntegralZone(0, gains.getIZone());
    talon.configClosedloopRamp(0.1);
    talon.configOpenloopRamp(0.1);
    talon.setInverted(isInverted);
    talon.setNeutralMode(NeutralMode.Coast);
    return talon;
  }

  /**
  * Sets the drivetrain to brake or coast mode.
  * @param isBrake whether to set the drivetrain to brake mode or not.
   */
  public void setNeutralMode(boolean isBrake) {
    if (isBrake) {
      m_driveMotor.setNeutralMode(NeutralMode.Brake);
      m_steeringMotor.setIdleMode(IdleMode.kBrake);
      return;
    }
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_steeringMotor.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Generates parameters for the targetState and sets it
   * @param speed the desired module speed in meters/s
   * @param angle the desired module ange in degrees
   */
  public void set(double angle, double speed) {
    targetState.angle = Rotation2d.fromDegrees(angle);
    targetState.speedMetersPerSecond = speed;
    set(targetState);
  }

  /**
   * Sets the target state of the module.
   * @param target The target state of the module.
   */
  public void set(SwerveModuleState target) {
    target = SwerveModuleState.optimize(target, currentState.angle);
    this.targetState = target;

    m_driveMotor.set(ControlMode.Velocity, meterPerSecToRPS(this.targetState.speedMetersPerSecond) / 10 * 2048);
    SmartDashboard.putNumber(moduleName + "desired angle", minChangeInSteerAngle(this.targetState.angle.getDegrees()));
    m_steeringMotor.getPIDController().setReference(degreesToRotations(minChangeInSteerAngle(this.targetState.angle.getDegrees())),
        CANSparkMax.ControlType.kPosition);
  }

  /**
   * Converts a speed in meters per second to rotations per second.
   * @param speed The speed in meters per second.
   * @return The speed in rotations per second.
   */
  private double meterPerSecToRPS(double speed) {
    return speed * SwerveModuleConstants.driveRatio / SwerveModuleConstants.wheelCircumferenceMeters;
  }

  /**
   * Returns the current RPS of the drive motor.
   * @return The current RPS of the drive motor.
   */
  public double getRPS() {
    return m_driveMotor.getSelectedSensorVelocity() * 10 / 2048;
  }

  /**
   * Converts an angle in degrees to rotations.
   * @param angle The angle in degrees.
   * @return The angle in rotations.
   */
  private double degreesToRotations(double angle) {
    return angle * SwerveModuleConstants.steeringRatio / 360.0;
  }

  /**
  * Returns the minimum angle that the steer can be set to, given the current angle and the
  * number of rotations that the steer has been rotated.
  * @param angle the current angle of the steer, in degrees.
  * @return the minimum angle that the steer can be set to, in degrees.
   */
  private double minChangeInSteerAngle(double angle) {
    double full_rotations = (int) m_steerRotations;
    double close_angle = angle + 360.0 * full_rotations;
    double angle_plus = close_angle + 360;
    double angle_minus = close_angle - 360;

    double minAngle = close_angle;
    if (Math.abs(minAngle - getAngle()) > Math.abs(angle_plus - getAngle()))
      minAngle = angle_plus;
    if (Math.abs(minAngle - getAngle()) > Math.abs(angle_minus - getAngle()))
      minAngle = angle_minus;

    return minAngle;
  }

  /**
   * Returns the current angle of the player.
   * @return the current angle of the player.
   */
  public double getAngle() {
    return currentState.angle.getDegrees();
  }

  /**
   * Locks the steering motor to the current state's angle.
   */
  public void lockPosition() {
    m_steeringMotor.getPIDController().setReference(degreesToRotations(currentState.angle.getDegrees()), ControlType.kPosition);
  }

  /**
   * Returns the current position of the module.
   * @return The current position of the module.
   */
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition() / 2048 / SwerveModuleConstants.driveRatio,
        currentState.angle);
  }

  /**
   * Stops the drivetrain and steering motor.
   */
  public void stop() {
    m_driveMotor.set(ControlMode.PercentOutput, 0);
    m_steeringMotor.set(0);
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getCurrentState() {
    return currentState;
  }

  /**
   * Returns the speed of the player in meters per second.
   * @return the speed of the player in meters per second.
   */
  public double getSpeed() {
    return currentState.speedMetersPerSecond;
  }

  /**
   * Returns the absolute position of the stream in milliseconds.
   * @return the absolute position of the stream in milliseconds.
   */
  public double getAbsolutePosition() {
    return absEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
  }
}
