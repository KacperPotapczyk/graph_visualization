use nalgebra::DMatrix;
use super::{Node, Connection};


pub fn evaluate_force_and_gradient(
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

    let (dx, dy) = distance_along_axis(size, &position);
    let distance_square = evaluate_distance_squared(size, &dx, &dy);

    for i in 0..size {
        for j in i+1..size {

            let force_multiplier = 2.0 * electroctatic_constant * &nodes[i].get_charge() * &nodes[j].get_charge();
            let distance_double_squared = distance_square[(i, j)].powf(2.0);
            *total_force += 2.0*force_multiplier / distance_square[(i, j)];

            let x_gradient = -1.0 * force_multiplier * dx[(i, j)] / distance_double_squared;
            let y_gradient = -1.0 * force_multiplier * dy[(i, j)] / distance_double_squared;
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

        let x_force_gradient = 2.0*k*dx[(node1_index, node2_index)];
        let y_force_gradient = 2.0*k*dy[(node1_index, node2_index)];

        *total_force += 2.0*k*distance_square[(node1_index, node2_index)].sqrt();

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


fn evaluate_distance_squared(
    size: usize,
    dx: &DMatrix<f64>,
    dy: &DMatrix<f64>
) -> nalgebra::DMatrix<f64> {

    let mut distance_squared = DMatrix::<f64>::zeros(size, size);
    for i in 0..size {
        for j in i+1..size {
            let dist_square = dx[(i, j)].powf(2.0) + dy[(i, j)].powf(2.0);
            distance_squared[(i, j)] = dist_square;
            distance_squared[(j, i)] = dist_square;
        }
    }

    return distance_squared;
}

fn distance_along_axis(
    size: usize,
    position: &nalgebra::Matrix<f64, nalgebra::Dyn, nalgebra::Const<1>, nalgebra::VecStorage<f64, nalgebra::Dyn, nalgebra::Const<1>>>
) -> (DMatrix<f64>, DMatrix<f64>) {

    let mut dx = DMatrix::<f64>::zeros(size, size);
    let mut dy = DMatrix::<f64>::zeros(size, size);
    for i in 0..size {
        for j in i+1..size {
            dx[(i, j)] = position[2*i] - position[2*j];
            dx[(j, i)] = -dx[(i, j)];
            dy[(i, j)] = position[2*i+1] - position[2*j+1];
            dy[(j, i)] = -dy[(i, j)];
        }
    }
    return (dx, dy);
}