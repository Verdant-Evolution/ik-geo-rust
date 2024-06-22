use crate::inverse_kinematics::{
    auxiliary::Kinematics, gen_six_dof, hardcoded::*, setups::calculate_ik_error, spherical,
    spherical_two_intersecting, spherical_two_parallel, three_parallel,
    three_parallel_two_intersecting, two_intersecting, two_parallel,
};

use nalgebra::{Matrix3, Vector3, Vector6};
use setups::{Irb6640, SphericalBot, ThreeParallelBot, TwoParallelBot, Ur5};

pub type IKFunction =
    fn(&Matrix3<f64>, &Vector3<f64>, &Kinematics<6, 7>) -> (Vec<Vector6<f64>>, Vec<bool>);

pub struct Robot {
    sub_problem_solver: IKFunction,
    kinematics: Kinematics<6, 7>,
}

pub trait IKSolver {
    fn ik(&self, rot: Matrix3<f64>, translation: Vector3<f64>) -> Vec<(Vector6<f64>, bool)>;
}

impl Robot {
    pub fn spherical_two_parallel(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: spherical_two_parallel,
            kinematics: kinematics,
        }
    }

    pub fn spherical_two_intersecting(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: spherical_two_intersecting,
            kinematics: kinematics,
        }
    }

    pub fn spherical(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: spherical,
            kinematics: kinematics,
        }
    }

    pub fn three_parallel_two_intersecting(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: three_parallel_two_intersecting,
            kinematics: kinematics,
        }
    }

    pub fn three_parallel(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: three_parallel,
            kinematics: kinematics,
        }
    }

    pub fn two_parallel(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: two_parallel,
            kinematics: kinematics,
        }
    }

    pub fn two_intersecting(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: two_intersecting,
            kinematics: kinematics,
        }
    }

    pub fn gen_six_dof(kinematics: Kinematics<6, 7>) -> Self {
        Robot {
            sub_problem_solver: gen_six_dof,
            kinematics: kinematics,
        }
    }

    // Get inverse kinematics and errors, sorted by error
    pub fn get_ik_sorted(
        &mut self,
        rot: Matrix3<f64>,
        translation: Vector3<f64>,
    ) -> Vec<([f64; 6], f64, bool)> {
        // First, get all the solutions
        let solutions = self.ik(rot, translation);

        // Get the errors of each solution
        let mut solutions_with_errors: Vec<([f64; 6], f64, bool)> =
            Vec::<([f64; 6], f64, bool)>::new();

        for (q, is_ls) in solutions {
            // Calculate the error, assuming it's 0 if it's not a least squares solution
            let mut error: f64 = 0.0;
            if is_ls {
                error = calculate_ik_error(&self.kinematics, &rot, &translation, &q);
            }

            // Convert q to a 6 element array
            let mut q_vals = [0.0; 6];
            for j in 0..6 {
                q_vals[j] = q[j];
            }
            solutions_with_errors.push((q_vals, error, is_ls));
        }

        // Sort the solutions by error, asecending
        solutions_with_errors.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        solutions_with_errors
    }

    pub fn fk(&self, q: &[f64; 6]) -> (Matrix3<f64>, Vector3<f64>) {
        self.kinematics
            .forward_kinematics(&Vector6::from_row_slice(q))
    }
}

impl IKSolver for Robot {
    fn ik(&self, rot: Matrix3<f64>, translation: Vector3<f64>) -> Vec<(Vector6<f64>, bool)> {
        let (solutions, lest_square_ness) =
            (self.sub_problem_solver)(&rot, &translation, &self.kinematics);
        solutions
            .into_iter()
            .zip(lest_square_ness.into_iter())
            .collect()
    }
}

#[no_mangle]
pub fn irb6640() -> Robot {
    Robot::spherical_two_parallel(Irb6640::get_kin())
}

#[no_mangle]
pub fn ur5() -> Robot {
    Robot::three_parallel_two_intersecting(Ur5::get_kin())
}

#[no_mangle]
pub fn three_parallel_bot() -> Robot {
    Robot::three_parallel(ThreeParallelBot::get_kin())
}

#[no_mangle]
pub fn two_parallel_bot() -> Robot {
    Robot::two_parallel(TwoParallelBot::get_kin())
}

#[no_mangle]
pub fn spherical_bot() -> Robot {
    Robot::spherical(SphericalBot::get_kin())
}

pub struct KukaR800FixedQ3 {
    robot: Robot,
    r_6t: Matrix3<f64>,
}

impl KukaR800FixedQ3 {
    pub fn new() -> Self {
        let (kinematics, r_6t) = setups::KukaR800FixedQ3::get_kin_partial();
        KukaR800FixedQ3 {
            robot: Robot::spherical_two_intersecting(kinematics),
            r_6t: r_6t,
        }
    }
}

impl IKSolver for KukaR800FixedQ3 {
    fn ik(&self, rot: Matrix3<f64>, translation: Vector3<f64>) -> Vec<(Vector6<f64>, bool)> {
        self.robot.ik(rot * self.r_6t.transpose(), translation)
    }
}

pub struct RrcFixedQ6 {
    robot: Robot,
    r_6t: Matrix3<f64>,
}

impl RrcFixedQ6 {
    pub fn new() -> Self {
        let (kinematics, r_6t) = setups::RrcFixedQ6::get_kin_partial();
        RrcFixedQ6 {
            robot: Robot::spherical_two_intersecting(kinematics),
            r_6t: r_6t,
        }
    }
}

impl IKSolver for RrcFixedQ6 {
    fn ik(&self, rot: Matrix3<f64>, translation: Vector3<f64>) -> Vec<(Vector6<f64>, bool)> {
        self.robot.ik(rot * self.r_6t.transpose(), translation)
    }
}

pub struct YumiFixedQ3 {
    robot: Robot,
    r_6t: Matrix3<f64>,
}

impl YumiFixedQ3 {
    pub fn new() -> Self {
        let (kinematics, r_6t) = setups::YumiFixedQ3::get_kin_partial();
        YumiFixedQ3 {
            robot: Robot::spherical_two_intersecting(kinematics),
            r_6t: r_6t,
        }
    }
}

impl IKSolver for YumiFixedQ3 {
    fn ik(&self, rot: Matrix3<f64>, translation: Vector3<f64>) -> Vec<(Vector6<f64>, bool)> {
        self.robot.ik(rot * self.r_6t.transpose(), translation)
    }
}
