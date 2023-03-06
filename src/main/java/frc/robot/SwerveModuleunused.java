// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class SwerveModuleunused extends SubsystemBase {
  private CANSparkMax m_steeringMotor;
  private TalonFX m_driveMotor;
  private CANCoder m_absoluteEncoder;
  private String moduleName;
  private double moduleRotations;

  public SwerveModuleunused(SwerveModuleConstants cModuleConstants) {
    m_absoluteEncoder = configCANCoder(cModuleConstants.canCoderId, cModuleConstants.cancoderZeroAngle);
    m_steeringMotor = configSparkMax(cModuleConstants.idSteering, cModuleConstants.steeringGains, cModuleConstants.isSteeringInverted);
    m_driveMotor = new TalonFX(cModuleConstants.idDrive);
    m_driveMotor = configTalonFX(cModuleConstants.idDrive, cModuleConstants.driveGains, cModuleConstants.isDriveInverted);
    moduleRotations = 0;
    switch (cModuleConstants.idDrive) {
      case 2:
          moduleName = "TOP LEFT";
        break;

      case 4:
        moduleName = "TOP RIGHT";
        break;

      case 6:
        moduleName = "BOTTOM LEFT";
        break;

      case 8:
        moduleName = "BOTTOM RIGHT";
        break;
    
      default:
        moduleName = "undefined";
        break;
    }
  }

  public double getAbsPosition() {
    return m_absoluteEncoder.getAbsolutePosition();
  }

  public void setModuleState(SwerveModuleState desiredState) {
    
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(this.m_steeringMotor.getEncoder().getPosition()));

    if (Math.abs(desiredState.speedMetersPerSecond) >= 0.2) {
      m_steeringMotor.getPIDController().setReference(optimizeAngle(desiredState.angle.getDegrees()), ControlType.kPosition);
      SmartDashboard.putNumber(moduleName + " desired angle", optimizeAngle(desiredState.angle.getDegrees()));
      SmartDashboard.putNumber(moduleName + " angle error",
          SmartDashboard.getNumber(moduleName + " desired angle", m_steeringMotor.getEncoder().getPosition())
              - m_steeringMotor.getEncoder().getPosition());
      // m_driveMotor.set(ControlMode.Velocity, metersPerSecToRPS(desiredState.speedMetersPerSecond) * 2048 / 10);
      m_driveMotor.set(ControlMode.PercentOutput,desiredState.speedMetersPerSecond / 2);
    }
    else 
      this.stop();
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
      rpsToMetersPerSec(getDriveMotorRPS()),
      new Rotation2d().rotateBy(Rotation2d.fromDegrees(m_steeringMotor.getEncoder().getPosition()))
    );
  }

  public static double metersPerSecToRPS(double MPS) {
    return MPS * SwerveModuleConstants.driveRatio / SwerveModuleConstants.wheelCircumferenceMeters;
  }

  public static double rpsToMetersPerSec(double RPS) {
    return SwerveModuleConstants.driveRatio / (SwerveModuleConstants.wheelCircumferenceMeters * RPS);
  }

  public static double degreesToRotations(double degrees) {
    return degrees * SwerveModuleConstants.steeringRatio / 360;
  }

  public double getDriveMotorRPS() { return m_driveMotor.getSelectedSensorVelocity() * 10 / 2048; }

  public double getDriveMeters() { return m_driveMotor.getSelectedSensorPosition() * (SwerveModuleConstants.wheelCircumferenceMeters / (SwerveModuleConstants.driveRatio * 2048)); }


  private CANSparkMax configSparkMax(int id,
      PIDFGains gains, boolean isInverted) {
    CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);
    sparkMax.getPIDController().setP(gains.getP());
    sparkMax.getPIDController().setI(gains.getI());
    sparkMax.getPIDController().setD(gains.getD()); 
    sparkMax.getPIDController().setIZone(gains.getIZone());
    sparkMax.setInverted(isInverted);
    sparkMax.getPIDController().setOutputRange(-1, 1);
    sparkMax.getEncoder().setPositionConversionFactor(SwerveModuleConstants.steeringPositionConversionFactor);
    sparkMax.getEncoder().setVelocityConversionFactor(SwerveModuleConstants.steeringVelocityConversionFactor);
    sparkMax.setSmartCurrentLimit(40);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setClosedLoopRampRate(0.01);
    sparkMax.enableVoltageCompensation(12);
    sparkMax.getEncoder().setPosition(getAbsPosition());

    return sparkMax;
  }

  private CANCoder configCANCoder(int id, double zeroAngle) {

    CANCoder canCoder = new CANCoder(id);
    // Always set CANCoder relative encoder to 0 on boot
    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    // Configure the offset angle of the magnet
    canCoder.configMagnetOffset(360 - zeroAngle);

    return canCoder;
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
    return talon;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        getDriveMeters(),
        new Rotation2d().rotateBy(Rotation2d.fromDegrees(m_steeringMotor.getEncoder().getPosition()))
    );
  }

  private double optimizeAngle(double angle) {
    double full_rotations = (int)moduleRotations;
    double close_angle = angle + 360.0 * full_rotations;
    double angle_plus = close_angle + 360;
    double angle_minus = close_angle - 360;

    double minAngle = close_angle;
    if (Math.abs(minAngle - getModuleState().angle.getDegrees()) > Math.abs(angle_plus - getModuleState().angle.getDegrees()))
      minAngle = angle_plus;
    if(Math.abs(minAngle - getModuleState().angle.getDegrees()) > Math.abs(angle_minus - getModuleState().angle.getDegrees())) minAngle = angle_minus;

    return minAngle;
  }

  public double getDriveVelocity() {
    return m_driveMotor.getSelectedSensorVelocity() * 10 / 2048 / SwerveModuleConstants.driveRatio * SwerveModuleConstants.wheelCircumferenceMeters;
  }

  public void calibrateSteering() {
    this.m_steeringMotor.getEncoder().setPosition(getAbsPosition());
  }

  public void stop() {
    this.m_driveMotor.set(ControlMode.PercentOutput, 0);
    this.m_steeringMotor.set(0);
  }

  @Override
  public String toString() {
      return moduleName;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(moduleName + " current angle", m_steeringMotor.getEncoder().getPosition());
    moduleRotations = m_steeringMotor.getEncoder().getPosition() / 360;
  }

}
