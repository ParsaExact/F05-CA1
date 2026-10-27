package autopilot.core;

import autopilot.core.AdaptiveCruise.Decision;
import autopilot.core.AdaptiveCruise.Mode;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * Unit tests for {@link AdaptiveCruise}.
 * Naming convention: methodName_stateUnderTest_expectedBehavior.
 */
class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 100.0;
    /** Cruising at 80 km/h gives: safe distance 40 m, emergency distance 20 m, release distance 48 m. */
    private static final double CRUISING_SPEED_KMH = 80.0;
    private static final double SAFE_DISTANCE_M = 40.0;
    private static final double EMERGENCY_DISTANCE_M = 20.0;
    private static final double RELEASE_DISTANCE_M = 48.0;
    private static final double INSIDE_SAFE_DISTANCE_M = 30.0;
    private static final double BETWEEN_SAFE_AND_RELEASE_M = 44.0;
    private static final double CLEAR_ROAD_M = 100.0;

    private static final double EMERGENCY_PRESSURE = 100.0;
    private static final double HYSTERESIS_PRESSURE = 10.0;
    private static final double MAX_ACCELERATION_STEP_KMH = 5.0;

    private static final Decision EMERGENCY = new Decision(Mode.EMERGENCY_BRAKE, EMERGENCY_PRESSURE);
    private static final Decision HOLD = new Decision(Mode.HOLD, 0.0);
    private static final Decision FULL_STEP_ACCELERATE = new Decision(Mode.ACCELERATE, MAX_ACCELERATION_STEP_KMH);
    private static final Decision LIGHT_BRAKE = new Decision(Mode.BRAKE, HYSTERESIS_PRESSURE);

    private final AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

    /** Feeds the given obstacle distances tick by tick at cruising speed and returns the last decision. */
    private Decision decideAfter(double... distancesMeters) {
        Decision last = null;
        for (double distance : distancesMeters) {
            last = cruise.update(CRUISING_SPEED_KMH, distance);
        }
        return last;
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void constructor_invalidTargetSpeed_throwsIllegalArgumentException(double targetSpeedKmh) {
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(targetSpeedKmh));
    }

    @ParameterizedTest(name = "{0} km/h -> {1} m")
    @CsvSource({
            "0.0,   0.0",
            "1.0,   0.5",
            "80.0,  40.0",
            "120.0, 60.0"
    })
    void safeDistanceFor_validSpeed_returnsHalfTheSpeedInMeters(double speedKmh, double expectedMeters) {
        assertEquals(expectedMeters, cruise.safeDistanceFor(speedKmh));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void safeDistanceFor_invalidSpeed_throwsIllegalArgumentException(double speedKmh) {
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(speedKmh));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_invalidSpeed_throwsIllegalArgumentException(double speedKmh) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(speedKmh, CLEAR_ROAD_M));
    }

    @ParameterizedTest
    @ValueSource(doubles = {-1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_invalidDistance_throwsIllegalArgumentException(double distanceMeters) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(CRUISING_SPEED_KMH, distanceMeters));
    }

    @ParameterizedTest(name = "{0} km/h, obstacle at {1} m -> {2}")
    @MethodSource("decisionsWithoutBrakingHistory")
    void update_withoutBrakingHistory_decidesFromDistanceAndSpeed(double speedKmh, double distanceMeters,
                                                                  Decision expected) {
        assertEquals(expected, cruise.update(speedKmh, distanceMeters));
    }

    static Stream<Arguments> decisionsWithoutBrakingHistory() {
        return Stream.of(
                Arguments.of(CRUISING_SPEED_KMH, 0.0, EMERGENCY),                     // touching the obstacle
                Arguments.of(CRUISING_SPEED_KMH, EMERGENCY_DISTANCE_M, EMERGENCY),    // exactly at emergency distance
                Arguments.of(CRUISING_SPEED_KMH, INSIDE_SAFE_DISTANCE_M,              // 10 m short of 40 m -> 25 %
                        new Decision(Mode.BRAKE, 25.0)),
                Arguments.of(CRUISING_SPEED_KMH, SAFE_DISTANCE_M, HOLD),              // exactly at safe distance
                Arguments.of(CRUISING_SPEED_KMH, BETWEEN_SAFE_AND_RELEASE_M,          // no hysteresis without history
                        FULL_STEP_ACCELERATE),
                Arguments.of(CRUISING_SPEED_KMH, CLEAR_ROAD_M, FULL_STEP_ACCELERATE), // far below target
                Arguments.of(97.0, CLEAR_ROAD_M, new Decision(Mode.ACCELERATE, 3.0)), // close to target: only the gap
                Arguments.of(TARGET_SPEED_KMH, CLEAR_ROAD_M, HOLD),                   // at target
                Arguments.of(110.0, CLEAR_ROAD_M, HOLD),                              // above target: hold, never brake
                Arguments.of(0.0, 1.0, FULL_STEP_ACCELERATE),                         // standstill with room ahead
                Arguments.of(0.0, 0.0, EMERGENCY)                                     // standstill touching obstacle
        );
    }

    @Test
    void update_betweenSafeAndReleaseDistanceWhileBraking_keepsLightBrake() {
        assertEquals(LIGHT_BRAKE, decideAfter(INSIDE_SAFE_DISTANCE_M, BETWEEN_SAFE_AND_RELEASE_M));
    }

    @Test
    void update_stillInsideSafeDistanceWhileBraking_keepsProportionalBrake() {
        double tenMetersShort = SAFE_DISTANCE_M - 10.0;

        assertEquals(new Decision(Mode.BRAKE, 25.0), decideAfter(INSIDE_SAFE_DISTANCE_M, tenMetersShort));
    }

    @Test
    void update_reachesReleaseDistanceWhileBraking_stopsBraking() {
        assertEquals(FULL_STEP_ACCELERATE, decideAfter(INSIDE_SAFE_DISTANCE_M, RELEASE_DISTANCE_M));
    }

    @Test
    void update_afterEmergencyBrake_keepsLightBrakeUntilReleaseDistance() {
        assertEquals(LIGHT_BRAKE, decideAfter(EMERGENCY_DISTANCE_M, BETWEEN_SAFE_AND_RELEASE_M));
    }

    @Test
    void update_afterBrakeReleased_hysteresisNoLongerApplies() {
        assertEquals(FULL_STEP_ACCELERATE,
                decideAfter(INSIDE_SAFE_DISTANCE_M, RELEASE_DISTANCE_M, BETWEEN_SAFE_AND_RELEASE_M));
    }
}
