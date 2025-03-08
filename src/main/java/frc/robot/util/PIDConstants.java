package frc.robot.util;

import java.util.Optional;

import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.util.LoggedTunableSparkPID.InternalPIDConstants;

/** A set of PID constants with tunable numbers for each for logged tunable PIDs. */
public class PIDConstants {
    public Optional<Double> p;
    public Optional<Double> i;
    public Optional<Double> iZone;
    public Optional<Double> d;
    public Optional<Double> f;
    public ClosedLoopSlot slot;

    public PIDConstants(Optional<Double> p, Optional<Double> i, Optional<Double> iZone, Optional<Double> d,
        Optional<Double> f, ClosedLoopSlot slot) {
        this.p = p;
        this.i = i;
        this.i = i;
        this.iZone = iZone;
        this.d = d;
        this.f = f;
        this.slot = slot;
    }

    public PIDConstants(double p, double i, double iZone, double d, double f, ClosedLoopSlot slot) {
        this(Optional.of(p), Optional.of(i), Optional.of(iZone), Optional.of(d), Optional.of(f), slot);
    }

    public PIDConstants(double p, double i, double d, double f, ClosedLoopSlot slot) {
        this(Optional.of(p), Optional.of(i), Optional.empty(), Optional.of(d), Optional.of(f), slot);
    }

    public PIDConstants(double p, double i, double d, ClosedLoopSlot slot) {
        this(Optional.of(p), Optional.of(i), Optional.empty(), Optional.of(d), Optional.empty(), slot);
    }

    public PIDConstants(double p, double i, double iZone, double d, double f) {
        this(p, i, iZone, d, f, ClosedLoopSlot.kSlot0);
    }

    public PIDConstants(double p, double i, double d, double f) {
        this(p, i, d, f, ClosedLoopSlot.kSlot0);
    }

    public PIDConstants(double p, double i, double d) {
        this(p, i, d, ClosedLoopSlot.kSlot0);
    }

    public PIDConstants iZone(double iZone) {
        this.iZone = Optional.of(iZone);
        return this;
    }

    public InternalPIDConstants toInternal(LoggedTunableSparkPID tunablePID) {
        return tunablePID.new InternalPIDConstants(this.p, this.i, this.iZone, this.d, this.f, slot);
    }
}
