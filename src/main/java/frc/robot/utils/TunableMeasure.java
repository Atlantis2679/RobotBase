package frc.robot.utils;

import java.util.function.Supplier;

import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public final class TunableMeasure<M extends Measure<?>> implements Supplier<M> {
    private final TunableDouble raw;
    private final Unit unit;

    private TunableMeasure(double rawInitial, Unit unit) {
        this.unit = unit;
        raw = TunableDouble.createConfig(
            rawInitial,
            new TunableConfig().withProperty("unit", "\"" + unit.symbol() + "\""));
    }

    public static <U extends Unit, M extends Measure<U>> TunableMeasure<M> of(M initial, U unit) {
        return new TunableMeasure<>(initial.in(unit), unit);
    }

    @SuppressWarnings("unchecked")
    @Override
    public M get() {
        return (M) unit.of(raw.get());
    }

    public TunableBase tunable() { return raw; }
}
