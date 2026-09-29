# C++ Style Guide and Project Requirements

This document outlines the coding standards, naming conventions, and architectural requirements for our C++ codebase. Adhering to these guidelines ensures maintainability, readability, and consistency across all robotic subsystems.

---

## 1. Naming Conventions

Consistent naming is critical for codebase navigation. Follow these rules for all new and refactored code:

| Type                   | Convention         | Example                         | Description                             |
| :--------------------- | :----------------- | :------------------------------ | :-------------------------------------- |
| **Variables**          | `camelCase`        | `currentPose`, `targetSpeed`    | Standard local and parameter variables. |
| **Functions**          | `camelCase`        | `getPose()`, `updateState()`    | All global and class member functions.  |
| **Files**              | `camelCase`        | `pathFollower.cpp`, `utils.hpp` | Header and source files.                |
| **Classes / Structs**  | `PascalCase`       | `Odometry`, `PidController`     | Custom data structures and objects.     |
| **Constants / Macros** | `UPPER_SNAKE_CASE` | `TURN_KP`, `MAX_VOLTAGE`        | `constexpr` and `const` values.         |
| **Class Members**      | `m_camelCase`      | `m_config`, `m_leftMotor`       | Private and protected class variables.  |

### Code Example

```cpp
class PoseEstimator {
public:
    void updateHeading(double sensorInput);

private:
    const double MAX_TOLERANCE = 0.05;
    double m_currentHeading;
};
```

---

## 2. Header and Include Conventions

To prevent header dependency hell and make dependencies clear at a glance, all `#include` directives must be grouped logically and separated by blank lines.

Order your includes strictly from most specific to most generic:

```cpp
// 1. Primary header declaration (The header this .cpp implements)
#include "daydream/motion/control/purePursuit.hpp"

// 2. C++ Standard Library headers
#include <cmath>
#include <iostream>
#include <vector>

// 3. Third-party / Vendor headers (e.g., PROS API, Eigen)
#include "main.h"

// 4. Project-level utilities & subsystems
#include "daydream/config/subsystems.hpp"
#include "daydream/motion/odometry.hpp"
#include "daydream/utils/helpers.hpp"
#include "daydream/utils/sd_card_logging.hpp"
```

---

## 3. Function Conventions

### Namespace Management (`using` Keyword)

Never use global `using namespace` directives in headers or source files. If the `using` keyword is necessary for brevity, it **must** be locally scoped inside the function body and placed at the very top lines of that function.

```cpp
// ❌ BAD: Global namespace pollution
using namespace std;

// ✅ GOOD: Locally scoped using statement
void calculateTrajectory() {
    using namespace daydream::math;
    using std::vector;

    vector<Point> path;
    // ... Function logic ...
}
```

### Documentation (Doxygen)

Every function and method declaration must be documented using standard [Doxygen format](https://www.doxygen.nl/manual/docblocks.html) blocks in the header file.

```cpp
/**
 * @brief Calculates the shortest angle to the target heading.
 *
 * @param currentHeading The robot's current angle in radians.
 * @param targetHeading The desired target angle in radians.
 * @return double The optimized turn error in the range [-pi, pi].
 */
double calculateAngleError(double currentHeading, double targetHeading);
```

---

## 4. Class Conventions

Classes and structs must follow a consistent layout, access hierarchy, and member declaration pattern to maintain code readability across all subsystems.

### Structure and Layout Order

Organize class declarations logically with explicit access modifiers. Public interfaces must be placed first so callers can immediately see the API, followed by internal implementations:

1. **Public Interface (`public:`):** Constructors, destructors, public methods, and public type aliases.
2. **Protected Members (`protected:`):** Internal state or helper methods intended for derived classes.
3. **Private State (`private:`):** Private member variables and private helper functions.

### Guidelines

- **Class Documentation:** Every class must have a top-level Doxygen header comment explaining its responsibility and scope.
- **Member Variable Naming:** All non-public member variables must use the `m_camelCase` prefix (e.g., `m_targetVelocity`).
- **Explicit Constructors:** Single-argument constructors must use the `explicit` keyword to prevent implicit type conversions.
- **In-Class Initialization:** Provide default member initializers at the variable declaration where practical.
- **Const Correctness:** Member functions that do not modify class state must be marked `const`.

### Example

```cpp
/**
 * @brief Handles closed-loop PID control calculations for robot motion.
 */
class PidController {
public:
    /**
     * @brief Constructs a PidController with gains.
     *
     * @param kP Proportional gain coefficient.
     * @param kI Integral gain coefficient.
     * @param kD Derivative gain coefficient.
     */
    explicit PidController(double kP, double kI = 0.0, double kD = 0.0);

    ~PidController() = default;

    /**
     * @brief Calculates output command based on measured and target state.
     *
     * @param current Actual sensor reading.
     * @param target Desired setpoint value.
     * @return double Calculated output command.
     */
    double compute(double current, double target);

    /**
     * @brief Resets accumulated error and previous error values.
     */
    void reset();

private:
    double m_kP{0.0};
    double m_kI{0.0};
    double m_kD{0.0};

    double m_accumulatedError{0.0};
    double m_previousError{0.0};
};
```

---

---

## 5. Namespace Conventions

Namespaces visually separate project subsystems (e.g., motion control, hardware interfaces, telemetry) and eliminate symbol collisions across translation units.

### Guidelines

- **Naming:** Use concise, all-lowercase names for namespaces (e.g., `daydream`, `motion`, `utils`). Avoid `CamelCase` or `UPPERCASE` namespace names.
- **Nested Namespaces:** Prefer C++17 nested namespace syntax (`namespace daydream::motion { ... }`) over multi-level nested opening tags.
- **No `using namespace` in Headers:** NEVER place `using namespace` or namespace aliases in header files (`.hpp`). This pollutes the global namespace for any file that includes the header.
- **Scoped Usage in Source Files:** Avoid top-level `using namespace` even in `.cpp` files. Keep `using` directives localized inside specific functions where needed.
- **Anonymous Namespaces:** Use internal (unnamed/anonymous) namespaces inside `.cpp` files to declare helper constants or internal utility functions restricted to that single source file.

### Example

```cpp
// Header file definition (daydream/motion/odometry.hpp)
namespace daydream::motion {

class Odometry {
public:
    void update();
};

} // namespace daydream::motion
```

```cpp

// Implementation file (daydream/motion/odometry.cpp)
#include "daydream/motion/odometry.hpp"
#include "daydream/utils/sd_card_logging.hpp"

namespace daydream::motion {

// Anonymous namespace for translation-unit local constants
namespace {
    constexpr double ENCODER_TICKS_PER_REV = 360.0;
}

void Odometry::update() {
    // Local scope using directive if needed for brevity
    using daydream::utils::SDLogger;

    SDLogger::log("Odometry", "Sensors polled successfully.");
}

} // namespace daydream::motion
```

---

## 6. Error Handling Protocol

### Code / System Errors

Routine system processes should operate silently under normal circumstances.

- **SD Card Logging:** All errors, regardless of severity, must be logged to the SD card for post-run analysis.
- **LCD Printing:** Only print to the physical LCD display on **catastrophic errors**. A catastrophic error is defined as a failure that prevents the robot from safely or accurately continuing its current routine (e.g., sensor failure breaking odometry).

### Physical / Hardware Errors

Physical errors directly impact driver performance and require immediate human feedback (e.g., motor disconnection, motor thermal throttling).

- **Tactile Feedback:** Alert the driver with a **single controller rumble**.
- **Visual Alert:** Print a concise warning directly to the **controller screen**.

```cpp
// Example: Hardware Error Handling
if (m_driveMotor.isOverheating()) {
    // 1. Driver Notification
    controller.rumble("."); // Single rumble pattern
    controller.print(0, 0, "ERR: Motor Hot");

    // 2. SD Log
    SDLogger::log("Hardware", "Drive motor thermal limit exceeded");
}
```

---

## 7. Testing and Debugging Conventions

Keep terminal output clean and focus on data retention for post-run evaluation.

- **Dedicated File Naming:** When writing test outputs, log specifically to an explicitly named test file on the SD card (e.g., `pure_pursuit_tuning_run1.csv`).
- **No Generic Names:** Never use placeholder file names such as `test.txt`, `temp.txt`, or `log.txt`.
- **Terminal & Console Output:** Terminal output (`std::cout`, `printf`) is permitted during testing for temporary debugging, provided it is not printed to the physical LCD display. In final production code, all terminal output must be completely removed or disabled. Because high-frequency serial output causes communication delays and log pollution, write persistent telemetry and test data directly to the SD card instead.
