package autopilot.core;

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * Unit tests for {@link GearBox}.
 * Naming convention: methodName_stateUnderTest_expectedBehavior.
 */
class GearBoxTest {

    private static final double STANDSTILL_KMH = 0.0;

    /** A speed sensor whose reading a test can change between calls. */
    private static final class AdjustableSpeedSensor implements SpeedSensor {
        private double speedKmh = STANDSTILL_KMH;

        @Override
        public double getSpeedKmh() {
            return speedKmh;
        }
    }

    private final AdjustableSpeedSensor speedSensor = new AdjustableSpeedSensor();
    private final GearBox gearBox = new GearBox(speedSensor);

    /**
     * Drives the gearbox from its initial P position to {@code gear} using only
     * transitions that are legal at standstill (P -> R -> N -> D).
     */
    private void driveTo(Gear gear) {
        speedSensor.speedKmh = STANDSTILL_KMH;
        for (Gear step : List.of(Gear.R, Gear.N, Gear.D)) {
            if (gearBox.getCurrentGear() == gear) {
                return;
            }
            gearBox.changeTo(step);
        }
    }

    @Test
    void changeTo_sameGear_keepsGearWithoutReadingSensor() {
        SpeedSensor brokenSensor = () -> {
            throw new AssertionError("sensor must not be read for a no-op change");
        };
        GearBox parkedGearBox = new GearBox(brokenSensor);

        parkedGearBox.changeTo(Gear.P);

        assertEquals(Gear.P, parkedGearBox.getCurrentGear());
    }

    @Test
    void changeTo_nullGear_throwsNullPointerException() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void changeTo_invalidSpeedReading_throwsIllegalArgumentException(double speedKmh) {
        speedSensor.speedKmh = speedKmh;

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
    }

    @ParameterizedTest(name = "{0} -> {1} at {2} km/h")
    @CsvFileSource(resources = "/gearbox-allowed-transitions.csv", numLinesToSkip = 1)
    void changeTo_allowedTransition_switchesGear(Gear from, Gear to, double speedKmh) {
        driveTo(from);
        speedSensor.speedKmh = speedKmh;

        gearBox.changeTo(to);

        assertEquals(to, gearBox.getCurrentGear());
    }

    @ParameterizedTest(name = "{0} -> {1} at {2} km/h")
    @CsvFileSource(resources = "/gearbox-rejected-transitions.csv", numLinesToSkip = 1)
    void changeTo_rejectedTransition_throwsIllegalGearChangeException(Gear from, Gear to, double speedKmh) {
        driveTo(from);
        speedSensor.speedKmh = speedKmh;

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(to));
    }

    @Test
    void changeTo_rejectedTransition_keepsCurrentGear() {
        driveTo(Gear.D);

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(Gear.N));

        assertEquals(Gear.D, gearBox.getCurrentGear());
    }
}
