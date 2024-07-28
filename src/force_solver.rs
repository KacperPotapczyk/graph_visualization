use nalgebra::{DMatrix, DVector};
use super::{Node, Connection, Coordinates};


pub fn solve(nodes: &Vec<Node>, connections: &Vec<Connection>, max_iterations: u32) -> Vec<Coordinates> {

    let step_multiplier = 0.1;
        let size = nodes.len();
        let electroctatic_constant = 1.0;
        
        let mut position = DVector::<f64>::zeros(2*size);

        let mut index=0;
        for node in nodes {
            let pos = &node.coordinates;
            position[index] = pos.x;
            index += 1;
            position[index] = pos.y;
            index += 1;
        }

        let root_coordinates = &nodes[0].coordinates;

        let mut iteration = 0;
        let mut solution_found = false;

        while solution_found == false {

            if iteration >= max_iterations {
                println!("Solution stopped due to reaching max iteration number");
                break;
            }

            iteration += 1;
            let mut gradient = DVector::<f64>::zeros(2*size);
            let previous_position = position.clone();
            let mut partial_x_gradient = DMatrix::<f64>::zeros(size, size);
            let mut partial_y_gradient = DMatrix::<f64>::zeros(size, size);

            for i in 0..size {
                for j in 0..size {

                    if i == j {
                        continue;
                    }
                    let force_multiplier = 2.0 * electroctatic_constant * &nodes[i].get_charge() * &nodes[j].get_charge();
                    let dx = position[2*i] - position[2*j];
                    let dy = position[2*i+1] - position[2*j+1];
                    let distance_double_squared = (dx.powf(2.0) + dy.powf(2.0)).powf(2.0);

                    let x_gradient = -1.0 * force_multiplier * dx / distance_double_squared;
                    let y_gradient = -1.0 * force_multiplier * dy / distance_double_squared;
                    partial_x_gradient[(i, j)] = x_gradient;
                    partial_x_gradient[(j, i)] = -x_gradient;
                    partial_y_gradient[(i, j)] = y_gradient;
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

                partial_x_gradient[(node1_index, node2_index)] += x_force_gradient;
                partial_x_gradient[(node2_index, node1_index)] -= x_force_gradient;
                partial_y_gradient[(node1_index, node2_index)] += y_force_gradient;
                partial_y_gradient[(node2_index, node1_index)] -= y_force_gradient;
            }

            for i in 0..size {
                for j in 0..size {

                    gradient[2*i] += partial_x_gradient[(i, j)];
                    gradient[2*i+1] += partial_y_gradient[(i, j)];
                }
            }

            position -= gradient.scale(step_multiplier);
            position[0] = root_coordinates.x;
            position[1] = root_coordinates.y;

            if gradient.norm() < 1e-4 {
                solution_found = true;
                println!("Solution found on gradient.norm() < 1e-4")
            }

            if (previous_position - position.clone()).norm() < 1e-4 {
                solution_found = true;
                println!("Solution found on solution improvement < 1e-4")
            }
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

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn solve_test_1() {
        let eps = 1e-2;
        let node1 = Node::new(0.0, 0.0, String::from("node1"));
        let node2 = Node::new(2.0, 0.0, String::from("node2"));
        let connection1 = Connection::new(0, 1);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(1.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_2() {
        let eps = 1e-2;
        let node1 = Node::new(0.0, 0.0, String::from("node1"));
        let node2 = Node::new(-2.0, 0.0, String::from("node2"));
        let connection1 = Connection::new(0, 1);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(-1.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_3() {
        let eps = 1e-2;
        let node1 = Node::new(0.0, 0.0, String::from("node1"));
        let node2 = Node::new(-2.0, 0.0, String::from("node2"));
        let connection1 = Connection::new(1, 0);

        let mut nodes = Vec::new();
        nodes.push(node1);
        nodes.push(node2);

        let mut connections = Vec::new();
        connections.push(connection1);

        let new_coordinates = solve(&nodes, &connections, 50);

        assert_eq!(2, new_coordinates.len());
        assert_approx_eq!(0.0, new_coordinates[0].x, eps);
        assert_approx_eq!(0.0, new_coordinates[0].y, eps);
        assert_approx_eq!(-1.0, new_coordinates[1].x, eps);
        assert_approx_eq!(0.0, new_coordinates[1].y, eps);
    }

    #[test]
    fn solve_test_4() {
        let eps = 1e-3;

        let nodes = vec![
            Node::new(0.0, 0.0, String::from("node1")),
            Node::new(2.0, 0.0, String::from("node2")),
            Node::new(0.0, 2.0, String::from("node3"))
        ];

        let connections = vec![
            Connection::new(0, 1),
            Connection::new(0, 2),
            Connection::new(1, 2)
        ];

        let new_coordinates = solve(&nodes, &connections, 50);
        assert_eq!(3, new_coordinates.len());
        assert_approx_eq!(1.0, new_coordinates[0].distance(&new_coordinates[1]), eps);
        assert_approx_eq!(1.0, new_coordinates[0].distance(&new_coordinates[2]), eps);
        assert_approx_eq!(1.0, new_coordinates[1].distance(&new_coordinates[2]), eps);
    }
}
