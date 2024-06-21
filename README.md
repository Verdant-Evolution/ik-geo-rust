# Ik Geo

This is a Rust implementation of the analytic inverse kinematics algorithms found in the [IK-Geo](https://arxiv.org/abs/2211.05737) paper. This implemenation has been adapted from the original Rust implementation by [RPI Robotics](https://github.com/rpiRobotics/ik-geo).

## Usage

To use this package, you either need to choose a an implemented model by name, or specify the kinematics as a Product of Exponentials and select the appropriate problem decomposition.

```rust
use ik_geo::{irb6640, spherical_bot, three_parallel_bot, two_parallel_bot, ur5, Robot};
fn main() {
    let robot = ur5();

    let R: Matrix3<f64> = ...
    let t: Vector3<f64> = ...

    // Solutions is a list of IK solutions
    // and whether they are least squares approximations.
    // Depending on the specific decomposition, a least squares solution might not be available.
    let solns = robot.ik(R, t);

    for (q, is_ls) in solns {
        if !is_ls {
            // This is an exact solution
        } else {
            // This one is close, but not exact
        }
    }
}
```

## Performance

While this implementation can be used on a wide range of manipulators, it performs much better on when the solution can be found entirely analytically. The following table shows which method is used for each type of kinematics:

| Solution Type | Robot Kinematic Family                             | Example                            |
| ------------- | -------------------------------------------------- | ---------------------------------- |
| Closed-form   | Spherical joint                                    | Franka Production 3, fixed $q_5$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | KUKA LBR iiwa 7 R800 , fixed $q_3$ |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | ABB IRB 6640                       |
|               | Three parallel axes                                | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | Universal Robots UR5               |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 1D search     | Two intersecting axes                              | Kassow Robots KR810, fixed $q_7$   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | FANUC CRX-10iA/L                   |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | Kawasaki KJ125                     |
|               | Two parallel axes                                  | N/A                                |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
|               | Two intersecting axes $k, k+2$                     | ABB YuMi, fixed $q_3$              |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two intersecting axes | RRC K-1207i, fixed $q_6$           |
|               | &nbsp;&nbsp;&nbsp;&nbsp; and two parallel axes     | N/A                                |
| 2D search     | General 6R                                         | Kassow Robots KR810, fixed $q_6$   |

### Testing

#### Correctness Tests

```
$ cargo test --release
```

For diagnostic info:

```
$ cargo test --release -- --nocapture
```
