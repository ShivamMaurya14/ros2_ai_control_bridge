# Contributing to AI Control Bridge

Thank you for interest in contributing! This document provides guidelines for contributing to the AI Control Bridge project.

## Code of Conduct

Be respectful, inclusive, and professional in all interactions.

## Getting Started

### 1. Fork & Clone

```bash
git clone https://github.com/yourusername/ai-control-bridge.git
cd ai-control-bridge
```

### 2. Create Development Branch

```bash
git checkout -b feature/your-feature-name
git checkout -b fix/your-fix-name
```

### 3. Build & Test

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
colcon test
```

## Code Style Guide

### C++ Standards

- **Standard**: C++17
- **Naming**: snake_case for variables/functions, CamelCase for classes
- **Documentation**: Doxygen-style comments for public APIs

```cpp
/// @brief Brief description of the function.
/// 
/// Detailed description of what the function does, including any
/// important implementation details or caveats.
/// 
/// @param[in] param1 Description of input parameter
/// @param[out] param2 Description of output parameter
/// @return Description of return value
/// 
/// @note Important notes about usage
/// @see Related functions
bool my_function(const std::vector<double>& param1, 
                 std::vector<double>& param2) {
    // Implementation
}
```

### Python Standards

- **Style**: PEP 8
- **Docstring**: Google-style docstrings
- **Type Hints**: Use for all public functions

```python
def process_observations(obs: List[float], 
                        normalize: bool = True) -> np.ndarray:
    """Process sensor observations for inference.
    
    Args:
        obs: Sensor observations as list of floats
        normalize: Whether to normalize observations
        
    Returns:
        Processed observations as numpy array
        
    Raises:
        ValueError: If observation dimensions are incorrect
    """
    pass
```

### File Documentation

Every file should start with:

```cpp
// Copyright 2026 GSoC Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file filename.hpp
/// @brief Purpose of this file in one sentence
/// 
/// More detailed description of what this file contains and why.
```

## Commit Messages

Follow conventional commit format:

```
type(scope): subject

body - explain what and why, not how

footer: references #123
```

**Types:**
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation
- `refactor:` Code refactoring
- `test:` Test additions/changes
- `chore:` Build/tooling changes

**Examples:**
```
feat(trajectory): add cubic spline trajectory generation

Implement cubic spline trajectory generator as alternative to quintic
for faster computation. Reduces memory overhead by 40%.

Fixes #42
```

```
fix(safety): prevent jerk limit overflow

Add bounds checking in jerk calculation to prevent integer overflow
when acceleration changes are large.

Fixes #99
```

## Pull Request Process

### 1. Before Creating PR

- [ ] Code follows style guide
- [ ] All comments and docstrings added
- [ ] README updated if behavior changed
- [ ] All tests pass: `colcon test`
- [ ] No compiler warnings: `--cmake-args -DCMAKE_BUILD_TYPE=Debug`

### 2. Create PR Description

```markdown
## Description

Brief description of what this PR changes and why.

## Type of Change

- [ ] Bug fix (non-breaking)
- [ ] New feature (non-breaking)
- [ ] Breaking change
- [ ] Documentation update

## How Has This Been Tested

Describe tests you ran and how to reproduce.

## Checklist

- [ ] Code follows style guidelines
- [ ] Self-review completed
- [ ] Comments added for complex logic
- [ ] Documentation updated
- [ ] Tests added/updated
- [ ] All tests pass
- [ ] No new compiler warnings
```

### 3. Code Review

- Respond to reviewer comments
- Update PR based on feedback
- Re-request review after updates

## Testing

### Unit Tests

```cpp
#include "gtest/gtest.h"
#include "trajectory.hpp"

TEST(TrajectoryTest, ComputesCorrectPositions) {
    auto traj = MinimumJerkTrajectory::compute(0.0, 1.0, 1.0);
    double pos = traj.evaluate(0.5);
    EXPECT_NEAR(pos, 0.8125, 1e-6);
}

TEST(TrajectoryTest, EnforcesZeroVelocityBoundaries) {
    auto traj = MinimumJerkTrajectory::compute(0.0, 1.0, 1.0);
    EXPECT_NEAR(traj.velocity(0.0), 0.0, 1e-6);
    EXPECT_NEAR(traj.velocity(1.0), 0.0, 1e-6);
}
```

### Integration Tests

Test multiple components together:

```bash
# Run integration test suite
ros2 launch ai_control_bridge_core ai_bridge_test.launch.py

# Test with real robot (if available)
ros2 launch my_robot_bringup.launch.py
ros2 launch ai_control_bridge_core ai_bridge_test.launch.py
```

### Performance Tests

Measure critical paths:

```bash
# Run with profiling
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
ros2 launch ai_control_bridge_core performance_test.launch.py
```

## Documentation

### README Files

Each package should have a comprehensive README with:

- **Overview**: What the package does
- **Architecture**: How it's structured
- **Usage**: How to use it
- **Configuration**: Parameter description
- **Examples**: Code examples
- **Testing**: How to test it

### API Documentation

Generate Doxygen docs:

```bash
cd ai_control_bridge_core
doxygen Doxyfile
# Open docs/html/index.html in browser
```

## Reporting Bugs

Use GitHub Issues with template:

```markdown
**Describe the bug**
Clear and concise description of the bug.

**To Reproduce**
Steps to reproduce the behavior:
1. Build with...
2. Run...
3. Observe...

**Expected behavior**
What should happen

**Actual behavior**
What actually happened

**Environment**
- OS: Ubuntu 24.04
- ROS: Jazzy
- Compiler: GCC 13

**Additional context**
Any other context
```

## Feature Requests

Use GitHub Issues with template:

```markdown
**Feature description**
Clear description of the desired feature.

**Motivation**
Why is this feature needed? What problem does it solve?

**Proposed solution**
How should this feature be implemented?

**Alternatives**
Any alternative solutions?

**Additional context**
Any other relevant information
```

## Questions?

- Join our ROS Discourse forum
- Check existing GitHub Issues
- See documentation in `/docs`

## License

By contributing, you agree that your contributions will be licensed under Apache 2.0.

---

Happy contributing! 🎉
