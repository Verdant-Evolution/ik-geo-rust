use crate::{inverse_kinematics::hardcoded::setups::{Irb6640, KukaR800FixedQ3, SphericalBot, ThreeParallelBot, TwoParallelBot, Ur5}, robot::{
    irb6640, spherical, spherical_bot, spherical_two_intersecting, spherical_two_parallel, three_parallel, three_parallel_bot, three_parallel_two_intersecting, two_parallel, two_parallel_bot, ur5, IKSolver, Robot
}};

use std::f64::consts::PI;
use rand::prelude::*;
use rand_pcg::Pcg64;
use rand_seeder::Seeder;

const TEST_ITERATIONS: usize = 1000;
const TOLERANCE: f64 = 1e-3;

fn test_robot(robot: Robot) {
    let mut rng: Pcg64 = Seeder::from("robot").make_rng();
    for _ in 0..TEST_ITERATIONS {
        let mut q = rng.gen::<[f64; 6]>();
        for i in 0..6 {
            q[i] = q[i] * 2.0 * PI - PI;
        }

        let (rot, translation) = robot.fk(&q);
        let solutions = robot.ik(rot, translation);

        let mut found = false;
        for (q, _) in solutions {
            let q = [q[0], q[1], q[2], q[3], q[4], q[5]];
            let (rot_test, translation_test) = robot.fk(&q);

            if (rot - rot_test).norm() < TOLERANCE
                && (translation - translation_test).norm() < TOLERANCE
            {
                found = true;
                break;
            }
        }

        if !found {
            panic!("Could not find a solution for q: {:?}", q);
        }
    }
}

#[test]
fn test_irb6640() {
    test_robot(irb6640());
}

#[test]
fn test_spherical_bot() {
    test_robot(spherical_bot());
}

#[test]
fn test_three_parallel_bot() {
    test_robot(three_parallel_bot());
}

#[test]
fn test_two_parallel_bot() {
    test_robot(two_parallel_bot());
}

#[test]
fn test_ur5() {
    test_robot(ur5());
}

#[test]
fn test_spherical() {
    let kin = SphericalBot::get_kin();
    test_robot(spherical(kin.h, kin.p));
}

#[test]
fn test_spherical_two_intersecting() {
    let (kin, _) = KukaR800FixedQ3::get_kin_partial();
    test_robot(spherical_two_intersecting(kin.h, kin.p));
}

#[test]
fn test_spherical_two_parallel() {
    let kin = Irb6640::get_kin();
    test_robot(spherical_two_parallel(kin.h, kin.p));
}

#[test]
fn test_three_parallel() {
    let kin = ThreeParallelBot::get_kin();
    test_robot(three_parallel(kin.h, kin.p));
}

#[test]
fn test_three_parallel_two_intersecting() {
    let kin = Ur5::get_kin();
    test_robot(three_parallel_two_intersecting(kin.h, kin.p));
}

#[test]
fn test_two_parallel() {
    let kin = TwoParallelBot::get_kin();
    test_robot(two_parallel(kin.h, kin.p));
}
