use crate::robot::{irb6640, spherical_bot, three_parallel_bot, two_parallel_bot, ur5, Robot};

use std::f64::consts::PI;

use rand::prelude::*;
use rand_pcg::Pcg64;
use rand_seeder::Seeder;

const TEST_ITERATIONS: usize = 1000;
const TOLERANCE: f64 = 1e-8;

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
