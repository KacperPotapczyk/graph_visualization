mod exact_model;
mod grid_model;

use std::time::Instant;

use nalgebra::DVector;
use super::{Node, Connection, Coordinates};


pub fn solve(nodes: &Vec<Node>, connections: &Vec<Connection>, max_iterations: u32) -> Vec<Coordinates> {

    let size = nodes.len();
    let electroctatic_constant = 1.0;
    let mut step = 0.005;
    let eps: f64 = 1e-6;
    let line_search_iter = (max_iterations as f64 / 2.0).floor() as u32;

    let now = Instant::now();
    
    let mut position = DVector::<f64>::zeros(2*size);
    let mut total_force = 0.0;

    let mut index=0;
    for node in nodes {
        let pos = &node.coordinates;
        position[index] = pos.x;
        index += 1;
        position[index] = pos.y;
        index += 1;
    }

    let mut iteration = 0;
    let mut solution_found = false;
    let mut gradient_prev;

    while solution_found == false {

        if iteration >= max_iterations {
            println!("Solution stopped due to reaching max iteration number");
            break;
        }

        iteration += 1;

        let previous_total_force = total_force;

        let mut gradient = DVector::<f64>::zeros(2*size);
        let previous_position = position.clone();
        evaluate_force_and_gradient(size, electroctatic_constant, nodes, &position, &mut total_force, connections, &mut gradient);    

        let direction = gradient.scale(-1.0);
        gradient_prev = gradient.clone();

        if iteration < line_search_iter {
            line_search(&mut position, &direction, electroctatic_constant, nodes, connections, step, &mut gradient, &mut total_force);
        } else {
            position += step * direction;
            evaluate_force_and_gradient(size, electroctatic_constant, nodes, &position, &mut total_force, connections, &mut gradient); 
        }

        if previous_total_force < total_force {
            step *= 0.5;
        } 

        solution_found = check_stop_conditions(&gradient_prev, &gradient, previous_position, &position, previous_total_force, total_force, eps);

        println!("Iter: {}, total_force: {}, gradient_norm: {}", iteration, total_force, gradient.norm());
    }

    let mut positions = Vec::with_capacity(size);
    for i in 0..size {
        positions.push(Coordinates {
            x: position[2*i],
            y: position[2*i+1]
        });
    }

    let elapsed = now.elapsed();
    println!("Elapsed: {:.2?}", elapsed);
    return positions;
}

fn check_stop_conditions(
        gradient_prev: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        gradient: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>,     
        previous_position: nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        position: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        previous_total_force: f64, 
        total_force: f64,
        eps: f64
    ) -> bool {

    if (gradient_prev.clone() - gradient.clone()).norm() / gradient.norm() < eps {
        println!("Solution found on relative gradient improvement < {}", eps);
        return true;
    }

    if (previous_position - position.clone()).norm() / position.norm() < eps {
        println!("Solution found on relative solution improvement < {}", eps);
        return true;
    }

    if (previous_total_force - total_force).abs() / total_force < eps {
        println!("Solution found on relative total force change < {}", eps);
        return true;
    }

    return false;
}

fn line_search(
        position: &mut nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        direction: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        electroctatic_constant: f64, 
        nodes: &Vec<Node>, 
        connections: &Vec<Connection>,
        initial_step: f64,
        gradient: &mut DVector<f64>,
        total_force: &mut f64
    ) {

    let size = nodes.len();
    let eps = 1e-3;

    let iter_max = 10;
    let mut iter = 0;
    let mut step = initial_step;
    let mut step_prev = 0.0;
    let mut solution_found = false;

    let rho = 0.1;
    let sigma = 0.9;
    let step_multiplier = 2.0;

    let mut a = 0.0;
    let mut b = step;
    let mut f_search_gradient = 0.0;

    let x = position.clone();
    let mut x_n = DVector::<f64>::zeros(2*size);
    // let x_a = position.clone();
    let f_0 = *total_force;
    let mut f_n = 0.0;
    let mut f_a = f_0;
    let mut f_b = 0.0;
    let gradient_0 = direction.dot(&gradient);

    if gradient.norm() < 1e-4 {
        solution_found = true;
    }

    while iter <= iter_max && !solution_found {
        iter += 1;

        let x_b = x.clone() + step * direction;
        evaluate_force_and_gradient(size, electroctatic_constant, nodes, &x_b, &mut f_b, connections, gradient);

        if f_b > f_a {
            a = step_prev;
            b = step;
            break;
        }

        f_search_gradient = direction.dot(&gradient);

        if f_search_gradient.abs() <= eps {
            solution_found = true;
            break;
        }

        if f_search_gradient >= 0.0 {
            a = step_prev;
            b = step;
            break;
        }

        let tmp = step;
        step += step_multiplier*(step - step_prev);
        step_prev = tmp;
        f_a = f_b;
    }

    println!("Bracketing iter: {}", iter);
    if !solution_found {
        iter = 0;
        while iter <= iter_max && !solution_found {
            iter += 1;

            step = (a+b) / 2.0;
            x_n = x.clone() + step * direction;
            f_n = 0.0;
            evaluate_force_and_gradient(size, electroctatic_constant, nodes, &x_n, &mut f_n, connections, gradient);

            if f_n > f_0 + rho*step*gradient_0 || f_n > f_a {
                b = step;
            } else {
                f_search_gradient = direction.dot(&gradient);
                if f_search_gradient.abs() <= -sigma*gradient_0 {
                    break;
                }
                a = step;
                f_a = f_n;
            }

            if f_search_gradient.abs() <= eps || (b-a) <= eps {
                break;
            }
        }
        println!("Bisect iter: {}", iter);
        *position = x_n.clone();
        *total_force = f_n;
    }
}

fn evaluate_force_and_gradient(
        size: usize, 
        electroctatic_constant: f64, 
        nodes: &Vec<Node>, 
        position: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        total_force: &mut f64, 
        connections: &Vec<Connection>, 
        gradient: &mut nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>
    ) {
    if size >= 100  {
        grid_model::evaluate_force_and_gradient(size, electroctatic_constant, nodes, position, total_force, connections, gradient);
    } else {
        exact_model::evaluate_force_and_gradient(size, electroctatic_constant, nodes, position, total_force, connections, gradient);
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn solve_test_1() {
        let eps = 0.5;
        let node1 = Node::new(0.0, 0.0, String::from("node1"), 12.0, 100.0);
        let node2 = Node::new(2.0, 0.0, String::from("node2"), 12.0, 100.0);
        let connection1 = Connection::new(0, 1, 1.0);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(10.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_2() {
        let eps = 0.5;
        let node1 = Node::new(0.0, 0.0, String::from("node1"), 12.0, 100.0);
        let node2 = Node::new(-2.0, 0.0, String::from("node2"), 12.0, 100.0);
        let connection1 = Connection::new(0, 1, 1.0);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(-10.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_3() {
        let eps = 0.5;
        let node1 = Node::new(0.0, 0.0, String::from("node1"), 12.0, 100.0);
        let node2 = Node::new(-2.0, 0.0, String::from("node2"), 12.0, 100.0);
        let connection1 = Connection::new(1, 0, 1.0);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(-10.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_4() {
        let eps = 0.5;

        let nodes = vec![
            Node::new(0.0, 0.0, String::from("node1"), 12.0, 100.0),
            Node::new(2.0, 0.0, String::from("node2"), 12.0, 100.0),
            Node::new(0.0, 2.0, String::from("node3"), 12.0, 100.0)
        ];

        let connections = vec![
            Connection::new(0, 1, 1.0),
            Connection::new(0, 2, 1.0),
            Connection::new(1, 2, 1.0)
        ];

        let new_coordinates = solve(&nodes, &connections, 50);
        assert_eq!(3, new_coordinates.len());
        assert_approx_eq!(10.0, new_coordinates[0].distance(&new_coordinates[1]), eps);
        assert_approx_eq!(10.0, new_coordinates[0].distance(&new_coordinates[2]), eps);
        // assert_approx_eq!(10.0, new_coordinates[1].distance(&new_coordinates[2]), eps);
    }
}
