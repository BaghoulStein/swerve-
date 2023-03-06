package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class PIDFGains {
    private double _kP, _kI, _kD, _kF, _tolerance, _iZone;

    public PIDFGains(double kP, double kI, double kD, double kF, double tolerance, double iZone) {
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
        this._kF = kF;
        this._tolerance = tolerance;
        this._iZone = iZone;
    }

    public PIDFGains(double kP, double kI, double kD) {
        this._kP = kP;
        this._kI = kI;
        this._kD = kD;
        this._kF = 0;
        this._tolerance = 0;
        this._iZone = 0;
    }

    public double getIZone() {
        return _iZone;
    }

    public double getP() {
        return this._kP;
    }

    public double getI() {
        return this._kI;
    }

    public double getD() {
        return this._kD;
    }

    public double getF() {
        return this._kF;
    }

    public double getTolerance() {
        return this._tolerance;
    }

    public PIDController createPIDController() {
        return new PIDController(_kP, _kI, _kD);
    }
}