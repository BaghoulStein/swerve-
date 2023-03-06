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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private TalonFX spin;
  private CANSparkMax steer;
  private CANCoder absEncoder;

  private double steer_rotations;
  public String moduleName;
  private SwerveModuleState target;
  private SwerveModuleState state;

  public SwerveModule(SwerveModuleConstants cModuleConstants) {
    absEncoder = configCANCoder(cModuleConstants.canCoderId, cModuleConstants.cancoderZeroAngle);
    spin = configTalonFX(cModuleConstants.idDrive, cModuleConstants.driveGains, cModuleConstants.isDriveInverted);
    steer = configSparkMax(cModuleConstants.idSteering, cModuleConstants.steeringGains,
        cModuleConstants.isSteeringInverted);

    steer_rotations = 0;
    target = new SwerveModuleState();
    state = new SwerveModuleState();

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
  }

  public void update() {
    SmartDashboard.putNumber(moduleName + "Cancoder position", getAbsolutePosition());
    SmartDashboard.putNumber(moduleName + " Ceo encoder position",
        steer.getEncoder().getPosition() * SwerveModuleConstants.steeringPositionConversionFactor);
    steer_rotations = steer.getEncoder().getPosition() / SwerveModuleConstants.steeringRatio;
    state.angle = Rotation2d.fromDegrees(steer_rotations * 360.0);
    state.speedMetersPerSecond = getRPS() * SwerveModuleConstants.wheelCircumferenceMeters
        / SwerveModuleConstants.driveRatio;
  }

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
    sparkMax.setIdleMode(IdleMode.kCoast);
    sparkMax.setClosedLoopRampRate(0.01);
    sparkMax.enableVoltageCompensation(12);
    sparkMax.getEncoder().setPosition(absEncoder.getAbsolutePosition() /
        SwerveModuleConstants.steeringPositionConversionFactor);
    // sparkMax.getEncoder().setPosition(0);

    return sparkMax;
  }

  private CANCoder configCANCoder(int id, double zeroAngle) {

    CANCoder canCoder = new CANCoder(id);
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    // canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    // Configure the offset angle of the magnet
    canCoder.configMagnetOffset(360 - zeroAngle);

    return canCoder;
  }

  public void resetMotors() {
    spin.set(ControlMode.PercentOutput, 0);
    steer.set(0);
  }

  public void reset() {
    resetMotors();

    steer_rotations = 0;
    target = new SwerveModuleState();
    state = new SwerveModuleState();
  }

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

  public void set(double angle, double speed) {
    target.angle = Rotation2d.fromDegrees(angle);
    target.speedMetersPerSecond = speed;
    set(target);
  }

  public void set(SwerveModuleState target) {
    target = SwerveModuleState.optimize(target, state.angle);
    this.target = target;

    spin.set(ControlMode.Velocity, meterPerSecToRPS(this.target.speedMetersPerSecond) / 10 * 2048);
    SmartDashboard.putNumber(moduleName + " desired angle", this.target.angle.getDegrees());
    steer.getPIDController().setReference(degreesToRotations(minChangeInSteerAngle(this.target.angle.getDegrees())),
        ControlType.kPosition);
  }

  private double meterPerSecToRPS(double speed) {
    return speed * SwerveModuleConstants.driveRatio / SwerveModuleConstants.wheelCircumferenceMeters;
  }

  public double getRPS() {
    return spin.getSelectedSensorVelocity() * 10 / 2048;
  }

  private double degreesToRotations(double angle) {
    return angle * SwerveModuleConstants.steeringRatio / 360.0;
  }

  private double minChangeInSteerAngle(double angle) {
    double full_rotations = (int) steer_rotations;
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

  public double getAngle() {
    return state.angle.getDegrees();
  }

  public void lockPosition() {
    steer.getPIDController().setReference(degreesToRotations(state.angle.getDegrees()), ControlType.kPosition);
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        spin.getSelectedSensorPosition() / 2048 / SwerveModuleConstants.driveRatio,
        state.angle);
  }

  public void stop() {
    spin.set(ControlMode.PercentOutput, 0);
    steer.set(0);
  }

  public SwerveModuleState getState() {
    return state;
  }

  public double getSpeed() {
    return state.speedMetersPerSecond;
  }

  public double getAbsolutePosition() {
    return absEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
  }
}
