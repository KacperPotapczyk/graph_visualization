use std::vec;

use nalgebra::{DMatrix, DVector};
use super::{Node, Connection, Coordinates};

struct Region {
    ll: Coordinates,
    ur: Coordinates,
    nodes_indexes: Vec<usize>,
    charge: f64,
    center_of_charge: Coordinates
}

impl Region {
    pub fn new(x: f64, y: f64, width: f64, heigth: f64) -> Self {
        Region {
            ll: Coordinates { x, y }, 
            ur: Coordinates { x: x+width, y: y+heigth },
            nodes_indexes: vec![],
            charge: 0.0,
            center_of_charge: Coordinates { x: x+(width / 2.0), y: y+(heigth/2.0) }
        }
    }

    pub fn add(&mut self, node_index: usize, node: &Node) -> bool {

        if self.contains(node) {

            self.nodes_indexes.push(node_index);
            
            if node.charge > 0.0 {
                let new_center_of_charge_x = (self.center_of_charge.x * self.charge + node.coordinates.x * node.charge) / (self.charge + node.charge);
                let new_center_of_charge_y = (self.center_of_charge.y * self.charge + node.coordinates.y * node.charge) / (self.charge + node.charge);
    
                self.center_of_charge = Coordinates {x: new_center_of_charge_x, y: new_center_of_charge_y};
                self.charge += node.charge;
            }
            return true;
        }
        return false;
    }

    pub fn contains(&self, node: &Node) -> bool {
        self.ll.x <= node.coordinates.x && node.coordinates.x < self.ur.x && 
        self.ll.y <= node.coordinates.y && node.coordinates.y < self.ur.y
    }
}

struct Grid {
    x_split: usize,
    y_split: usize,
    regions: Vec<Region>,
    neighbour_indexes: Vec<Vec<usize>>
}

impl Grid {
    pub fn new(ll_x: f64, ll_y: f64, ur_x: f64, ur_y: f64, x_split: usize, y_split: usize) -> Self {

        let region_width = (ur_x - ll_x) / x_split as f64;
        let region_heigth = (ur_y - ll_y) / y_split as f64;
        let mut regions = vec![];

        let mut neighbour_indexes = Vec::with_capacity(x_split*y_split);

        for j in 0..y_split {
            for i in 0..x_split {
                regions.push(Region::new(ll_x + (i as f64)*region_width, ll_y + (j as f64)*region_heigth, region_width, region_heigth));

                let mut current_region_neighbour_indexes = vec![];
                for m in j as i32-1 ..= j as i32+1  {
                    for n in i as i32-1 ..= i as i32+1 {

                        if 0 <= m && m < y_split as i32 && 0 <= n && n < x_split as i32 {
                            current_region_neighbour_indexes.push(m as usize*x_split + n as usize);
                        }

                    }
                }
                neighbour_indexes.push(current_region_neighbour_indexes);
            }
        }

        Grid {
            x_split, y_split, 
            regions,
            neighbour_indexes
        }
    }

    pub fn add(&mut self, node_index: usize, node: &Node) -> usize {
        let number_of_regions = self.number_of_regions();
        for index in 0..number_of_regions {
            if self.regions[index].add(node_index, node) {
                return index;
            }
        }
        return 0;
    }

    pub fn get_region(&self, (i, j): (usize, usize)) -> &Region {
        &self.regions[i + j*self.x_split]
    }

    pub fn number_of_regions(&self) -> usize {
        self.regions.len()
    }

    pub fn get_region_with_neighbours(&self, index: usize) -> &Vec<usize> {
        &self.neighbour_indexes[index]
    }
}

pub fn evaluate_force_and_gradient(
    number_of_nodes: usize, 
    electroctatic_constant: f64, 
    nodes: &Vec<Node>, 
    position: &DVector<f64>, 
    total_force: &mut f64, 
    connections: &Vec<Connection>, 
    gradient: &mut DVector<f64>
) {

    let mut ll_x = position[0];
    let mut ll_y = position[1];
    let mut ur_x = position[0];
    let mut ur_y = position[1];
    let pad = 1.0;

    for i in 1..number_of_nodes {
        if position[2*i] < ll_x {
            ll_x = position[2*i];
        } else if ur_x < position[2*i] {
            ur_x = position[2*i];
        }

        if position[2*i+1] < ll_y {
            ll_y = position[2*i+1];
        } else if ur_y < position[2*i+1] {
            ur_y = position[2*i+1];
        }
    }

    let number_of_regions = number_of_nodes as f64 / 4.0;
    let aspect_ratio = (ur_x - ll_x) / (ur_y - ll_y);
    let x_split = (number_of_regions * aspect_ratio).sqrt();
    let y_split = x_split / aspect_ratio;

    let mut grid = Grid::new(
        ll_x-pad, 
        ll_y-pad, 
        ur_x+pad, 
        ur_y+pad, 
        x_split.floor() as usize, 
        y_split.floor() as usize
    );
    let mut nodes_region = Vec::with_capacity(number_of_nodes);

    for (node_index, node) in nodes.iter().enumerate() {
        nodes_region.push(grid.add(node_index, node));
    }

    let number_of_regions = grid.number_of_regions();
    let nodes_and_regions = number_of_nodes + number_of_regions;
    let mut partial_x_gradient = DMatrix::<f64>::zeros(number_of_nodes, nodes_and_regions);
    let mut partial_y_gradient = DMatrix::<f64>::zeros(number_of_nodes, nodes_and_regions);

    *total_force = 0.0;
    for i in 0..number_of_nodes {
        gradient[2*i] = 0.0;
        gradient[2*i+1] = 0.0;
    }

    let mask = nodes_electrostatic_mask(number_of_nodes, &grid);
    let (dx, dy) = distance_along_axis(number_of_nodes, position, &grid, &nodes_region, connections, &mask);
    let distance_squared = evaluate_distance_squared(number_of_nodes, grid.number_of_regions(), &dx, &dy, &grid, &nodes_region, &mask);


    for i in 0..number_of_nodes {
        let region_and_neighbours = grid.get_region_with_neighbours(nodes_region[i]);
        for j in i+1..number_of_nodes {

            if mask[(i, j)] {
                let force_multiplier = 2.0 * electroctatic_constant * &nodes[i].get_charge() * &nodes[j].get_charge();
                let distance_double_squared = distance_squared[(i, j)].powf(2.0);
                *total_force += 2.0*force_multiplier / distance_squared[(i, j)];

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
        for r in 0..number_of_regions {
            let j = number_of_nodes + r;

            if region_and_neighbours.iter().any(|region| *region == r) {
                continue;
            } else {
                let force_multiplier = 2.0 * electroctatic_constant * &nodes[i].get_charge() * grid.regions[r].charge;
                let distance_double_squared = distance_squared[(i, j)].powf(2.0);
                *total_force += 2.0*force_multiplier / distance_squared[(i, j)];

                let x_gradient = -1.0 * force_multiplier * dx[(i, j)] / distance_double_squared;
                let y_gradient = -1.0 * force_multiplier * dy[(i, j)] / distance_double_squared;

                if i == 0 {
                    partial_x_gradient[(i, j)] = 0.0;
                    partial_y_gradient[(i, j)] = 0.0;
                } else {
                    partial_x_gradient[(i, j)] = x_gradient;
                    partial_y_gradient[(i, j)] = y_gradient;
                }
            }
        }
    }

    for connection in connections {
        let node1_index = connection.index1;
        let node2_index = connection.index2;
        let k = connection.get_stifness();

        let x_force_gradient = 2.0*k*dx[(node1_index, node2_index)];
        let y_force_gradient = 2.0*k*dy[(node1_index, node2_index)];

        *total_force += 2.0*k*distance_squared[(node1_index, node2_index)].sqrt();

        if node1_index != 0 {
            partial_x_gradient[(node1_index, node2_index)] += x_force_gradient;
            partial_y_gradient[(node1_index, node2_index)] += y_force_gradient;
        }
        if node2_index != 0 {
            partial_x_gradient[(node2_index, node1_index)] -= x_force_gradient;
            partial_y_gradient[(node2_index, node1_index)] -= y_force_gradient;
        }
    }

    for i in 0..number_of_nodes {
        for j in 0..nodes_and_regions {
            gradient[2*i] += partial_x_gradient[(i, j)];
            gradient[2*i+1] += partial_y_gradient[(i, j)];
        }
    }
}

fn nodes_electrostatic_mask (
    size: usize,
    grid: &Grid
) -> DMatrix<bool> {

    let mut mask = DMatrix::<bool>::from_element(size, size, false);

    for y in 0..grid.y_split {
        for x in 0..grid.x_split {

            let mut regions = vec![grid.get_region((x, y))];

            // neighbour regions
            if y < grid.y_split-1 {
                regions.push(grid.get_region((x, y+1)));
            } 
            if x < grid.x_split-1 {
                regions.push(grid.get_region((x+1, y)));
            }
            if x < grid.x_split-1 && y < grid.y_split-1 {
                regions.push(grid.get_region((x+1, y+1)));
            }

            let mut nodes_indexes = vec![];
            regions.iter().for_each(
                |n_region| n_region.nodes_indexes.iter().for_each(
                    |index| nodes_indexes.push(index)
                )
            );

            let number_of_region_nodes = nodes_indexes.len();
 
            // nodes in region and neighbours
            for i in 0..number_of_region_nodes {
                let index1 = *nodes_indexes[i];
                for j in i+1..number_of_region_nodes {
                    let index2 = *nodes_indexes[j];
                    mask[(index1, index2)] = true;
                    mask[(index2, index1)] = true;
                }
            }
        }
    }
    return mask;
}

fn distance_along_axis(
    number_of_nodes: usize,
    position: &DVector<f64>,
    grid: &Grid,
    nodes_region: &Vec<usize>,
    connections: &Vec<Connection>,
    nodes_electrostatic_mask: &DMatrix<bool>
) -> (DMatrix<f64>, DMatrix<f64>) {

    let nodes_and_regions = number_of_nodes+grid.number_of_regions();
    let mut dx = DMatrix::<f64>::zeros(number_of_nodes, nodes_and_regions);
    let mut dy = DMatrix::<f64>::zeros(number_of_nodes, nodes_and_regions);
    for i in 0..number_of_nodes {
        for j in i+1..number_of_nodes {
            if nodes_electrostatic_mask[(i, j)] {
                dx[(i, j)] = position[2*i] - position[2*j];
                dx[(j, i)] = -dx[(i, j)];
                dy[(i, j)] = position[2*i+1] - position[2*j+1];
                dy[(j, i)] = -dy[(i, j)];
            }
        }
    }
    for connection in connections {
        let i = connection.index1;
        let j = connection.index2;

        if !nodes_electrostatic_mask[(i, j)] {
            dx[(i, j)] = position[2*i] - position[2*j];
            dx[(j, i)] = -dx[(i, j)];
            dy[(i, j)] = position[2*i+1] - position[2*j+1];
            dy[(j, i)] = -dy[(i, j)];
        }
    }
    for i in 0..number_of_nodes {
        let region_and_neighbours = grid.get_region_with_neighbours(nodes_region[i]);
        for r in 0..grid.number_of_regions() {
            let j = number_of_nodes + r;

            if region_and_neighbours.iter().any(|region| *region == r) {
                continue;
            } else {              
                dx[(i, j)] = position[2*i] - grid.regions[r].center_of_charge.x;
                dy[(i, j)] = position[2*i+1] - grid.regions[r].center_of_charge.y;
            }
        }
    }
    return (dx, dy);
}

fn evaluate_distance_squared(
    number_of_nodes: usize,
    number_of_regions: usize,
    dx: &DMatrix<f64>,
    dy: &DMatrix<f64>,
    grid: &Grid,
    nodes_region: &Vec<usize>,
    nodes_electrostatic_mask: &DMatrix<bool>
) -> nalgebra::DMatrix<f64> {


    let nodes_and_regions = number_of_nodes + number_of_regions;
    let mut distance_squared = DMatrix::<f64>::zeros(number_of_nodes, nodes_and_regions);
    for i in 0..number_of_nodes {
        for j in i+1..number_of_nodes {
            if nodes_electrostatic_mask[(i, j)] {
                let dist_square = dx[(i, j)].powf(2.0) + dy[(i, j)].powf(2.0);
                distance_squared[(i, j)] = dist_square;
                distance_squared[(j, i)] = dist_square;
            }
        }

        let region_and_neighbours = grid.get_region_with_neighbours(nodes_region[i]);
        for r in 0..number_of_regions {
            let j = number_of_nodes + r;
            if region_and_neighbours.iter().any(|region| *region == r) {
                continue;
            } else {   
                distance_squared[(i, j)] = dx[(i, j)].powf(2.0) + dy[(i, j)].powf(2.0);
            }
        }
    }

    return distance_squared;
}
