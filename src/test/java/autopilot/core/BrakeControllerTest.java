package autopilot.core;

import autopilot.core.GearBox.Gear;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link BrakeController}.
 * Naming convention: methodName_stateUnderTest_expectedBehavior.
 */
class BrakeControllerTest {

    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    /** Above the 80 % ABS pressure threshold. */
    private static final double HARD_PRESSURE = 95.0;
    /** Below the 80 % ABS pressure threshold. */
    private static final double GENTLE_PRESSURE = 40.0;
    /** Above the 30 km/h ABS speed threshold. */
    private static final double HIGHWAY_SPEED_KMH = 50.0;
    private static final Gear DRIVING_GEAR = Gear.D;

    private final BrakeController brakes = new BrakeController();

    @ParameterizedTest(name = "request {0} % at {1} km/h -> {2} %")
    @CsvSource({
            "40.0,  50.0,  40.0",   // ordinary request is applied as-is
            "0.0,   50.0,  0.0",    // releasing the brake is a valid request
            "80.0,  50.0,  80.0",   // exactly at ABS pressure threshold: no ABS
            "80.1,  30.0,  80.1",   // exactly at ABS speed threshold: no ABS
            "80.1,  30.1,  90.0",   // just past both thresholds: first ABS pulse (high)
            "100.0, 20.0,  100.0",  // maximum pressure, too slow for ABS
            "150.0, 20.0,  100.0",  // request above maximum is capped
            "150.0, 50.0,  90.0"    // capped and fast: ABS engages
    })
    void apply_validRequest_returnsCappedOrPulsedPressure(double requested, double speedKmh, double expected) {
        assertEquals(expected, brakes.apply(requested, speedKmh, DRIVING_GEAR));
    }

    @Test
    void apply_validRequest_storesAppliedPressure() {
        brakes.apply(GENTLE_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR);

        assertEquals(GENTLE_PRESSURE, brakes.getPressure());
    }

    @Test
    void apply_hardBrakingAtSpeed_alternatesAbsPulses() {
        double[] pulses = {
                brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR),
                brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR),
                brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR)
        };

        assertArrayEquals(new double[] {ABS_HIGH_PULSE, ABS_LOW_PULSE, ABS_HIGH_PULSE}, pulses);
    }

    @Test
    void apply_gentleBrakingAfterAbsPulse_restartsPulseSequenceAtHigh() {
        brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR);   // high pulse, next would be low
        brakes.apply(GENTLE_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR); // leaves ABS

        assertEquals(ABS_HIGH_PULSE, brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR));
    }

    @Test
    void apply_ordinaryBraking_doesNotRaiseWarning() {
        brakes.apply(GENTLE_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR);

        assertFalse(brakes.isWarningActive());
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_invalidPressure_throwsIllegalArgumentException(double requested) {
        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(requested, HIGHWAY_SPEED_KMH, DRIVING_GEAR));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_invalidSpeed_throwsIllegalArgumentException(double speedKmh) {
        assertThrows(IllegalArgumentException.class,
                () -> brakes.apply(GENTLE_PRESSURE, speedKmh, DRIVING_GEAR));
    }

    @Test
    void apply_inPark_throwsIllegalStateException() {
        assertThrows(IllegalStateException.class,
                () -> brakes.apply(GENTLE_PRESSURE, HIGHWAY_SPEED_KMH, Gear.P));
    }

    @Test
    void apply_nullGear_throwsNullPointerException() {
        assertThrows(NullPointerException.class,
                () -> brakes.apply(GENTLE_PRESSURE, HIGHWAY_SPEED_KMH, null));
    }

    @ParameterizedTest
    @EnumSource(value = Gear.class, mode = EnumSource.Mode.EXCLUDE, names = "P")
    void emergencyBrake_outsidePark_appliesMaximumPressure(Gear gear) {
        brakes.emergencyBrake(gear);

        assertEquals(MAX_PRESSURE, brakes.getPressure());
    }

    @Test
    void emergencyBrake_outsidePark_raisesWarning() {
        brakes.emergencyBrake(DRIVING_GEAR);

        assertTrue(brakes.isWarningActive());
    }

    @Test
    void emergencyBrake_afterAbsPulse_restartsPulseSequenceAtHigh() {
        brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR); // high pulse, next would be low
        brakes.emergencyBrake(DRIVING_GEAR);

        assertEquals(ABS_HIGH_PULSE, brakes.apply(HARD_PRESSURE, HIGHWAY_SPEED_KMH, DRIVING_GEAR));
    }

    @Test
    void emergencyBrake_inPark_throwsIllegalStateException() {
        assertThrows(IllegalStateException.class, () -> brakes.emergencyBrake(Gear.P));
    }

    @Test
    void emergencyBrake_inPark_doesNotRaiseWarning() {
        assertThrows(IllegalStateException.class, () -> brakes.emergencyBrake(Gear.P));

        assertFalse(brakes.isWarningActive());
    }

    @Test
    void emergencyBrake_nullGear_throwsNullPointerException() {
        assertThrows(NullPointerException.class, () -> brakes.emergencyBrake(null));
    }
}
