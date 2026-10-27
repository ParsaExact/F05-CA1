package autopilot.core;

import autopilot.core.AdaptiveCruise.Mode;
import autopilot.core.AutoDriver.DriveResult;
import autopilot.core.GearBox.Gear;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SensorSnapshot;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Unit tests for {@link AutoDriver}.
 * Naming convention: methodName_stateUnderTest_expectedBehavior.
 */
class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    /** Already at target speed, so the cruise control has nothing to do on a clear road. */
    private static final double CRUISING_SPEED_KMH = TARGET_SPEED_KMH;
    /** Far beyond the 40 m safe distance that 80 km/h requires. */
    private static final double CLEAR_ROAD_DISTANCE_M = 100.0;
    private static final double CENTERED_LANE_OFFSET_M = 0.0;
    private static final double CENTERED_STEERING_DEGREES = 0.0;
    private static final double NO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    /** One healthy snapshot that can stand in for all three sensors at once. */
    private static final SensorSnapshot CLEAR_ROAD =
            SensorSnapshot.of(CRUISING_SPEED_KMH, CLEAR_ROAD_DISTANCE_M, CENTERED_LANE_OFFSET_M);
    private static final SensorSnapshot FAILED_SENSOR = SensorSnapshot.failed("sensor failed");

    private static final DriveResult HOLDING_COURSE =
            new DriveResult(CENTERED_STEERING_DEGREES, NO_BRAKE_PRESSURE, Mode.HOLD, false, false);
    private static final DriveResult SAFE_MODE =
            new DriveResult(CENTERED_STEERING_DEGREES, SAFE_MODE_BRAKE_PRESSURE, Mode.BRAKE, false, true);

    @Test
    void tick_clearRoadWithHealthySensors_holdsCourseWithoutBraking() {
        AutoDriver driver = new AutoDriver(CLEAR_ROAD, CLEAR_ROAD, CLEAR_ROAD, TARGET_SPEED_KMH);

        assertEquals(HOLDING_COURSE, driver.tick(Gear.D));
    }

    @ParameterizedTest(name = "lane offset {0} m")
    @ValueSource(doubles = {1.0, -1.0}) // both beyond the 0.5 m deviation threshold
    void tick_laneOffsetBeyondThreshold_steersAgainstTheOffset(double laneOffsetMeters) {
        LaneSensor driftedLane = () -> laneOffsetMeters;
        AutoDriver driver = new AutoDriver(CLEAR_ROAD, CLEAR_ROAD, driftedLane, TARGET_SPEED_KMH);

        DriveResult result = driver.tick(Gear.D);

        assertEquals(-Math.signum(laneOffsetMeters), Math.signum(result.steeringAngleDegrees()));
    }

    @ParameterizedTest(name = "{0} sensor fails")
    @MethodSource("singleSensorFailures")
    void tick_anySensorFails_entersSafeMode(String failedSensor, SpeedSensor speedSensor,
                                            ObstacleSensor obstacleSensor, LaneSensor laneSensor) {
        AutoDriver driver = new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);

        assertEquals(SAFE_MODE, driver.tick(Gear.D));
    }

    static Stream<Arguments> singleSensorFailures() {
        return Stream.of(
                Arguments.of("speed", FAILED_SENSOR, CLEAR_ROAD, CLEAR_ROAD),
                Arguments.of("obstacle", CLEAR_ROAD, FAILED_SENSOR, CLEAR_ROAD),
                Arguments.of("lane", CLEAR_ROAD, CLEAR_ROAD, FAILED_SENSOR)
        );
    }
}
