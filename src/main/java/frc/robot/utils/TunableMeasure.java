package frc.robot.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public final class TunableMeasure<M extends Measure<?>> implements Supplier<M> {
    private M value;
    private final TunableBase tunable;

    @SuppressWarnings("unchecked")
    public <U extends Unit> TunableMeasure(M initial, U unit) {
        value = initial;
        tunable = bind(() -> (Measure<U>) value, v -> value = (M) v, unit);
    }

    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> TunableBase bind(
            Supplier<M> getter, Consumer<M> setter, U unit) {
        return TunableDouble.createConfig(
            () -> getter.get().in(unit),
            setter == null ? null : v -> setter.accept((M) unit.of(v)),
            new TunableConfig().withProperty("unit", "\"" + unit.symbol() + "\""));
    }

    @Override public M get() { return value; }
    public TunableBase tunable() { return tunable; }
}