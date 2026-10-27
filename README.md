# F05-CA1: Autonomous Vehicle Project

## Architecture

The project architecture is divided into three main packages:

- `sensor`: Defines interfaces for speed, obstacle distance, and lane deviation sensors. `SensorSnapshot` stores real-time sensor readings at a specific moment and can report sensor failures.
- `core`: Contains the main vehicle control logic.
- `exception`: Defines exceptions related to invalid gear shifts and sensor failures.

`AutoDriver` acts as the main coordinator. It reads sensor data, manages steering, adaptive cruise control, and braking, and switches to a safe mode in case of sensor failure.

## Class Logic

- **`GearBox`**: Implements a state machine for the `P`, `R`, `N`, and `D` gear positions. Gear transitions depend on the vehicle's current speed, and invalid transitions throw exceptions.
- **`SteeringController`**: Maintains the steering angle within `-450°` to `+450°`, limits changes to `60°` per tick, reduces the steering range to `±90°` at speeds above `120 km/h`, and supports automatic steering centering.
- **`BrakeController`**: Manages brake pressure, caps it at `100%`, simulates ABS using alternating `60%` and `90%` pressure pulses, and supports emergency braking.
- **`AdaptiveCruise`**: Calculates the safe following distance as half the vehicle's speed in km/h, expressed in meters. Depending on the distance to the vehicle ahead, it gradually accelerates, maintains speed, brakes, or applies emergency braking. Hysteresis prevents rapid switching between acceleration and braking.
- **`AutoDriver`**: Corrects lane deviations greater than `0.5 meters` through steering adjustments. In case of sensor failure, it centers the steering wheel and applies gentle braking.
## Tests

Unit tests live in `src/test/java/autopilot/core`, one `*Test` class per production class.

- **Naming convention**: `methodName_stateUnderTest_expectedBehavior`
  (e.g. `changeTo_rejectedTransition_throwsIllegalGearChangeException`).
- **No magic numbers**: every literal is a named constant or carries a comment explaining the boundary it probes.
- **One behaviour per test**: each test has a single logical assertion; related inputs are grouped in parameterized tests.
- **Independence**: every test builds its own fresh object (field initializers run per test method in JUnit 5).

Parameterized sources, each exercising a different method:

| Source           | Method under test                | Test class               |
|------------------|----------------------------------|--------------------------|
| `@CsvFileSource` | `GearBox.changeTo`               | `GearBoxTest`            |
| `@EnumSource`    | `BrakeController.emergencyBrake` | `BrakeControllerTest`    |
| `@CsvSource`     | `SteeringController.update`      | `SteeringControllerTest` |
| `@MethodSource`  | `AdaptiveCruise.update`          | `AdaptiveCruiseTest`     |
| `@ValueSource`   | `AdaptiveCruise.safeDistanceFor` | `AdaptiveCruiseTest`     |

The CSV data for `@CsvFileSource` lives in `src/test/resources/`.

Run everything with `mvn test`.
