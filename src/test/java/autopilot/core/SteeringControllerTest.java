package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * Unit tests for {@link SteeringController}.
 * Naming convention: methodName_stateUnderTest_expectedBehavior.
 */
class SteeringControllerTest {

    /** At or below the 120 km/h threshold the full +/-450 degree range is available. */
    private static final double LOW_SPEED_KMH = 50.0;
    /** Above the 120 km/h threshold the range shrinks to +/-90 degrees. */
    private static final double HIGH_SPEED_KMH = 150.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double MAX_STEP_DEGREES = 60.0;
    private static final double CENTER_DEGREES = 0.0;
    private static final double ANGLE_BEYOND_ONE_STEP = 100.0;
    private static final int TICKS_TO_REACH_ANY_ANGLE = 10;

    private final SteeringController steering = new SteeringController();

    /** Turns the wheel tick by tick at low speed until it rests at {@code angleDegrees}. */
    private void steerTo(double angleDegrees) {
        for (int tick = 0; tick < TICKS_TO_REACH_ANY_ANGLE; tick++) {
            steering.update(angleDegrees, LOW_SPEED_KMH);
        }
        assertEquals(angleDegrees, steering.getCurrentAngleDegrees(), "test setup failed");
    }

    @ParameterizedTest(name = "request {0} deg at {1} km/h -> target {2} deg")
    @CsvSource({
            "30.0,   50.0,   30.0",    // inside every limit
            "450.0,  50.0,   450.0",   // exactly the physical limit
            "-451.0, 50.0,   -450.0",  // beyond the physical limit is clamped
            "500.0,  120.0,  450.0",   // exactly at the speed threshold: full range
            "500.0,  120.1,  90.0",    // just above the threshold: reduced range
            "-100.0, 150.0,  -90.0",   // reduced range applies on the negative side too
            "90.0,   150.0,  90.0"     // exactly the reduced limit
    })
    void update_anyRequest_clampsTargetToSpeedDependentLimit(double requested, double speedKmh, double expectedTarget) {
        steering.update(requested, speedKmh);

        assertEquals(expectedTarget, steering.getTargetAngleDegrees());
    }

    @ParameterizedTest(name = "request {0} deg from center -> {1} deg after one tick")
    @CsvSource({
            "0.0,    0.0",    // nothing to do
            "30.0,   30.0",   // less than one step: reached immediately
            "60.0,   60.0",   // exactly one step: reached immediately
            "61.0,   60.0",   // more than one step: rate limited
            "-100.0, -60.0"   // rate limit applies in the negative direction too
    })
    void update_fromCenter_movesTowardTargetByAtMostOneStep(double requested, double expectedAngle) {
        assertEquals(expectedAngle, steering.update(requested, LOW_SPEED_KMH));
    }

    @Test
    void update_repeatedTicks_convergesOnTargetThenStays() {
        double[] angles = {
                steering.update(ANGLE_BEYOND_ONE_STEP, LOW_SPEED_KMH),
                steering.update(ANGLE_BEYOND_ONE_STEP, LOW_SPEED_KMH),
                steering.update(ANGLE_BEYOND_ONE_STEP, LOW_SPEED_KMH)
        };

        assertArrayEquals(new double[] {MAX_STEP_DEGREES, ANGLE_BEYOND_ONE_STEP, ANGLE_BEYOND_ONE_STEP}, angles);
    }

    @Test
    void update_speedRisesAboveThresholdWhileTurnedSharply_unwindsTowardReducedLimit() {
        double sharpAngle = HIGH_SPEED_LIMIT_DEGREES + MAX_STEP_DEGREES + MAX_STEP_DEGREES; // 210 deg
        steerTo(sharpAngle);

        double angleAfterOneTick = steering.update(sharpAngle, HIGH_SPEED_KMH);

        assertEquals(sharpAngle - MAX_STEP_DEGREES, angleAfterOneTick);
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_nonFiniteAngle_throwsIllegalArgumentException(double angleDegrees) {
        assertThrows(IllegalArgumentException.class, () -> steering.update(angleDegrees, LOW_SPEED_KMH));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_invalidSpeed_throwsIllegalArgumentException(double speedKmh) {
        assertThrows(IllegalArgumentException.class, () -> steering.update(CENTER_DEGREES, speedKmh));
    }

    @ParameterizedTest(name = "from {0} deg -> {1} deg after one tick")
    @CsvSource({
            "0.0,    0.0",    // already centered
            "45.0,   0.0",    // within one step: centered immediately
            "-60.0,  0.0",    // exactly one step: centered immediately
            "100.0,  40.0",   // more than one step: rate limited
            "-100.0, -40.0"   // rate limit applies from the negative side too
    })
    void selfCenter_fromAnyAngle_movesTowardCenterByAtMostOneStep(double startAngle, double expectedAngle) {
        steerTo(startAngle);

        assertEquals(expectedAngle, steering.selfCenter());
    }

    @Test
    void selfCenter_whileTurned_setsTargetToCenter() {
        steerTo(ANGLE_BEYOND_ONE_STEP);

        steering.selfCenter();

        assertEquals(CENTER_DEGREES, steering.getTargetAngleDegrees());
    }
}
