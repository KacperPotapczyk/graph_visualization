use nalgebra::{DMatrix, DVector};
use super::{Node, Connection, Coordinates};


pub fn solve(nodes: &Vec<Node>, connections: &Vec<Connection>, max_iterations: u32) -> Vec<Coordinates> {

        let size = nodes.len();
        let electroctatic_constant = 1.0;
        let mut step = 0.005;
        let eps: f64 = 1e-6;
        
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

            line_search(&mut position, &direction, electroctatic_constant, nodes, connections, step);
            evaluate_force_and_gradient(size, electroctatic_constant, nodes, &position, &mut total_force, connections, &mut gradient);

            if previous_total_force < total_force {
                step *= 0.5;
            } 

            if (gradient_prev.clone() - gradient.clone()).norm() / gradient.norm() < eps {
                solution_found = true;
                println!("Solution found on relative gradient improvement < {}", eps);
            }

            if (previous_position - position.clone()).norm() / position.norm() < eps {
                solution_found = true;
                println!("Solution found on relative solution improvement < {}", eps);
            }

            if (previous_total_force - total_force).abs() / total_force < eps {
                solution_found = true;
                println!("Solution found on relative total force change < {}", eps);
            }

            println!("Iter: {}, total_force: {}, gradient_norm: {}", iteration, total_force, gradient.norm());
        }

        let mut positions = Vec::with_capacity(size);
        for i in 0..size {
            positions.push(Coordinates {
                x: position[2*i],
                y: position[2*i+1]
            });
        }

        return positions;
}

fn line_search(
        position: &mut nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        direction: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>, 
        electroctatic_constant: f64, 
        nodes: &Vec<Node>, 
        connections: &Vec<Connection>,
        initial_step: f64
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

    let mut gradient = DVector::<f64>::zeros(2*size);

    let x = position.clone();
    let mut x_n = DVector::<f64>::zeros(2*size);
    // let x_a = position.clone();
    let mut f_0 = 0.0;
    evaluate_force_and_gradient(size, electroctatic_constant, nodes, &x, &mut f_0, connections, &mut gradient);
    let mut f_a = f_0;
    let mut f_b = 0.0;
    let gradient_0 = direction.dot(&gradient);

    if gradient.norm() < 1e-4 {
        solution_found = true;
    }

    while iter <= iter_max && !solution_found {
        iter += 1;

        let x_b = x.clone() + step * direction;
        evaluate_force_and_gradient(size, electroctatic_constant, nodes, &x_b, &mut f_b, connections, &mut gradient);

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
            let mut f_n = 0.0;
            evaluate_force_and_gradient(size, electroctatic_constant, nodes, &x_n, &mut f_n, connections, &mut gradient);

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
    let mut partial_x_gradient = DMatrix::<f64>::zeros(size, size);
    let mut partial_y_gradient = DMatrix::<f64>::zeros(size, size);

    *total_force = 0.0;
    for i in 0..size {
        gradient[2*i] = 0.0;
        gradient[2*i+1] = 0.0;
    }


    for i in 0..size {
        for j in i..size {

            if i == j {
                continue;
            }
            let force_multiplier = 2.0 * electroctatic_constant * &nodes[i].get_charge() * &nodes[j].get_charge();
            let dx = position[2*i] - position[2*j];
            let dy = position[2*i+1] - position[2*j+1];
            let distance_double_squared = (dx.powf(2.0) + dy.powf(2.0)).powf(2.0);
            *total_force += 2.0*force_multiplier / (dx.powf(2.0) + dy.powf(2.0));

            let x_gradient = -1.0 * force_multiplier * dx / distance_double_squared;
            let y_gradient = -1.0 * force_multiplier * dy / distance_double_squared;
            if i == 0 {
                partial_x_gradient[(i, j)] = 0.0;
                partial_y_gradient[(i, j)] = 0.0;
            } else {
                partial_x_gradient[(i, j)] = x_gradient;
                partial_y_gradient[(i, j)] = y_gradient;
            }
            partial_x_gradient[(j, i)] = -x_gradient;
            partial_y_gradient[(j, i)] = -y_gradient;
        }
    }

    for connection in connections {
        let node1_index = connection.index1;
        let node2_index = connection.index2;
        let k = connection.get_stifness();

        let dx = position[2*node1_index] - position[2*node2_index];
        let dy = position[2*node1_index+1] - position[2*node2_index+1];

        let x_force_gradient = 2.0*k*dx;
        let y_force_gradient = 2.0*k*dy;

        *total_force += 2.0*k*(dx.powf(2.0) + dy.powf(2.0)).sqrt();

        if node1_index != 0 {
            partial_x_gradient[(node1_index, node2_index)] += x_force_gradient;
            partial_y_gradient[(node1_index, node2_index)] += y_force_gradient;
        }
        if node2_index != 0 {
            partial_x_gradient[(node2_index, node1_index)] -= x_force_gradient;
            partial_y_gradient[(node2_index, node1_index)] -= y_force_gradient;
        }
    }

    for i in 0..size {
        for j in 0..size {

            gradient[2*i] += partial_x_gradient[(i, j)];
            gradient[2*i+1] += partial_y_gradient[(i, j)];
        }
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
