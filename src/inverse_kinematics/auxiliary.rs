use {
    crate::subproblems::auxiliary::rot, 
    nalgebra::{ArrayStorage, Const, Matrix, Matrix3, Vector3, U1, U3, U7, U8}, nlopt::{Algorithm, Nlopt}, 
    std::f64::consts::{PI, TAU},
    
};

pub type Matrix3x7<T> = Matrix<T, U3, U7, ArrayStorage<T, 3, 7>>;
pub type Matrix3x8<T> = Matrix<T, U3, U8, ArrayStorage<T, 3, 8>>;

pub type Vector<T, const N: usize> = Matrix<T, Const<N>, U1, ArrayStorage<f64, N, 1>>;

#[derive(Debug, Clone)]
pub struct Kinematics<const C1: usize, const C2: usize> {
    // TODO: somehow statically ensure that C2 - C1 = 1
    pub h: Matrix<f64, U3, Const<C1>, ArrayStorage<f64, 3, C1>>,
    pub p: Matrix<f64, U3, Const<C2>, ArrayStorage<f64, 3, C2>>,
}

impl<const C1: usize, const C2: usize> Kinematics<C1, C2> {
    pub fn new() -> Self {
        Self {
            h: Matrix::<f64, U3, Const<C1>, ArrayStorage<f64, 3, C1>>::zeros(),
            p: Matrix::<f64, U3, Const<C2>, ArrayStorage<f64, 3, C2>>::zeros(),
        }
    }

    pub fn forward_kinematics(
        &self,
        theta: &Matrix<f64, Const<C1>, U1, ArrayStorage<f64, C1, 1>>,
    ) -> (Matrix3<f64>, Vector3<f64>) {
        let mut p: Vector3<f64> = self.p.column(0).into();
        let mut r = Matrix3::identity();

        for (i, &t) in theta.iter().enumerate() {
            r = r * rot(&self.h.column(i).into(), t);
            p = p + r * self.p.column(i + 1);
        }

        (r, p)
    }
}

impl Kinematics<7, 8> {
    pub fn forward_kinematics_partial(
        &self,
        q_n: f64,
        n: usize,
        r_6t: &Matrix3<f64>,
    ) -> (Kinematics<6, 7>, Matrix3<f64>) {
        let mut kin_new: Kinematics<6, 7> = Kinematics::new();
        let r_n = rot(&self.h.column(n).into(), q_n);

        for i in 0..self.h.ncols() {
            if i > n {
                kin_new.h.set_column(i - 1, &(r_n * self.h.column(i)));
            } else {
                kin_new.h.set_column(i, &self.h.column(i));
            }
        }

        for i in 0..self.p.ncols() {
            if i == n {
                kin_new
                    .p
                    .set_column(i, &(self.p.column(i) + r_n * self.p.column(i + 1)));
            } else if i > n + 1 {
                kin_new.p.set_column(i - 1, &(r_n * self.p.column(i)));
            } else {
                kin_new.p.set_column(i, &self.p.column(i));
            }
        }

        (kin_new, r_n * r_6t)
    }
}

pub fn wrap_to_pi(theta: f64) -> f64 {
    (theta + PI).rem_euclid(TAU) - PI
}

pub fn search_1d<const N: usize, F: Fn(f64) -> Vector<f64, N> > (
    f: F,
    left: f64,
    right: f64
) -> Vec<(f64, usize)> {

    let mut results : Vec<(f64,usize)> = Vec::new();
    let max_evals = [50, 200, 1000, 2000];
    let populations = [10, 30, 200, 400];
    let epsilon = 1e-8;

    // Do some iterative deepening
    for (max_eval, population) in core::iter::zip(max_evals, populations) { 
        for i in 0..N {
            // TODO: parallelize this, there are 4 branches that don't depend on each other
            // Problems occur trying to multithread this due to the closure not being thread-safe.

            let cost_function = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| {
                let err = f(x[0])[i];
                err * err
            };

            
            let mut optimizer = Nlopt::new(
                Algorithm::GnMlslLds,
                1,
                cost_function,
                nlopt::Target::Minimize, ());


            let mut x = [(left + right) / 2.0];
            optimizer.set_upper_bounds(&[right]).unwrap();
            optimizer.set_lower_bounds(&[left]).unwrap();
            optimizer.set_xtol_abs(&[epsilon]).unwrap();
            optimizer.set_stopval(epsilon).unwrap();
            optimizer.set_maxeval(max_eval).unwrap();
            optimizer.set_population(population).unwrap();

            let mut local_optimizer = Nlopt::new(
                Algorithm::Neldermead,
                1,
                cost_function,
                nlopt::Target::Minimize, ());
            local_optimizer.set_maxeval(max_eval / 2).unwrap();
            optimizer.set_local_optimizer(local_optimizer).unwrap();
            
            let result = optimizer.optimize(&mut x);
            // Check for error
            if let Err(e) = result {
                // Didn't optimize, no zero on this branch
                println!("Error: {:?}", e);
                continue;
            }
            // Get the error
            let error = result.unwrap().1;

            if error < epsilon {
                results.push((x[0], i));
            }
            
        }
        if results.len() > 0 {
            break;
        }
    }
    results

}

pub fn search_2d<const N: usize, F: Fn(f64, f64) -> Vector<f64, N>>(
    f: F,
    min: (f64, f64),
    max: (f64, f64)
) -> Vec<(f64, f64, usize)> {
    let mut results : Vec<(f64, f64,usize)> = Vec::new();
    let max_evals = [50, 200, 1000, 3000, 6000];

    let populations = [10, 30, 200, 400, 900];
    let epsilon = 1e-6;
    let stopval = 1e-8;

    // Do some iterative deepening
    for (max_eval, population) in core::iter::zip(max_evals, populations) { 
        for i in 0..N {
            // TODO: parallelize this, there are 4 branches that don't depend on each other
            // Problems occur trying to multithread this due to the closure not being thread-safe.

            let cost_function = |x: &[f64], _gradient: Option<&mut [f64]>, _params: &mut ()| {
                let err = f(x[0], x[1])[i];
                err * err
            };

            
            let mut optimizer = Nlopt::new(
                Algorithm::GnMlsl,
                2,
                cost_function,
                nlopt::Target::Minimize, ());


            let mut x = [(min.0 + max.0) / 2.0, (min.1 + max.1) / 2.0];
            optimizer.set_upper_bounds(&[max.0, max.1]).unwrap();
            optimizer.set_lower_bounds(&[min.0, min.1]).unwrap();
            optimizer.set_xtol_abs(&[stopval, stopval]).unwrap();
            optimizer.set_stopval(stopval).unwrap();
            optimizer.set_maxeval(max_eval).unwrap();
            optimizer.set_population(population).unwrap();

            let mut local_optimizer = Nlopt::new(
                Algorithm::Neldermead,
                2,
                cost_function,
                nlopt::Target::Minimize, ());
            // local_optimizer.set_maxeval(local_max_eval).unwrap();
            // local_optimizer.set_xtol_abs(&[stopval, stopval]).unwrap();
            local_optimizer.set_maxeval(max_eval / 2).unwrap();
            optimizer.set_local_optimizer(local_optimizer).unwrap();
            
            let result = optimizer.optimize(&mut x);
            // Check for error
            if let Err(e) = result {
                println!("Error: {:?}", e);
                continue;
            }
            // Get the error
            let error = result.unwrap().1;

            if error < epsilon {
                results.push((x[0], x[1], i));
            }
        }
        if results.len() > 0 {
            break;
        }
    }

    results
}
