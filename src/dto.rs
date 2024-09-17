use super::{Graph, Node, Connection, Coordinates};
use serde::{Deserialize, Serialize};

const DEFAULT_FONT_SIZE: f32 = 12.0;
const DEFAULT_CHARGE: f64 = 1.0;
const DEFAULT_STIFFNESS: f64 = 1.0;

pub trait Dto<Source=Self> {
    type Target;

    fn to_model(&self) -> Self::Target;
    fn from_model(target: &Self::Target) -> Self;
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CoordinatesDto {
    x: f64,
    y: f64,
}

impl Dto for CoordinatesDto {
    type Target = Coordinates;

    fn to_model(&self) -> Self::Target {
        Coordinates {x: self.x, y: self.y}
    }

    fn from_model(target: &Self::Target) -> Self {
        CoordinatesDto {x: target.x, y: target.y}
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct NodeDto {
    coordinates: CoordinatesDto,
    label: String,
    description: Option<String>,
    font_size: Option<f32>,
    charge: Option<f64>,
}

impl Dto for NodeDto {
    type Target = Node;

    fn to_model(&self) -> Self::Target {
        Node {
            coordinates: self.coordinates.to_model(),
            label: self.description.clone().unwrap_or(self.label.clone()),
            font_size: self.font_size.unwrap_or(DEFAULT_FONT_SIZE),
            charge: self.charge.unwrap_or(DEFAULT_CHARGE)
        }
    }
    
    fn from_model(target: &Self::Target) -> Self {
        NodeDto {
            coordinates: CoordinatesDto::from_model(&target.coordinates),
            label: target.label.clone(),
            description: None,
            font_size: Some(target.font_size),
            charge: Some(target.charge)
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConnectionDto {
    node1: String,
    node2: String,
    stiffness: Option<f64>,
}

impl ConnectionDto {

    fn to_model(&self, node_dtos: &Vec<NodeDto>) -> Connection {

        let mut index1 = 0;
        let mut index2 = 0;
        let mut index1_found = false;
        let mut index2_found = false;

        for (index, node_dto) in node_dtos.iter().enumerate() {
            if !index1_found && node_dto.label == self.node1 {
                index1 = index;
                index1_found = true;
                if index2_found {
                    break;
                }
            } else if !index2_found && node_dto.label == self.node2 {
                index2 = index;
                index2_found = true;
                if index1_found {
                    break;
                }
            }
        }

        if !index1_found {
            panic!("Connection is invalid! Could not find node with label: {}", self.node1);
        }
        if !index2_found {
            panic!("Connection is invalid! Could not find node with label: {}", self.node2);
        }

        Connection {
            index1,
            index2,
            stiffness: self.stiffness.unwrap_or(DEFAULT_STIFFNESS)
        }
    }

    fn from_model(connection: &Connection, nodes: &Vec<Node>) -> Self {
        ConnectionDto {
            node1: nodes[connection.index1].label.clone(),
            node2: nodes[connection.index2].label.clone(),
            stiffness: Some(connection.stiffness)
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GraphDto {
    nodes: Vec<NodeDto>,
    connections: Vec<ConnectionDto>,
}

impl Dto for GraphDto {
    type Target = Graph;

    fn to_model(&self) -> Self::Target {
        Graph {
            nodes: self.nodes.iter()
                    .map(|node_dto| node_dto.to_model())
                    .collect(),
            connections: self.connections.iter()
                    .map(|connection_dto| connection_dto.to_model(&self.nodes))
                    .collect()
        }
    }

    fn from_model(target: &Self::Target) -> Self {
        GraphDto {
            nodes: target.nodes.iter()
                    .map(|target_node| NodeDto::from_model(target_node))
                    .collect(),
            connections: target.connections.iter()
                    .map(|target_connection| ConnectionDto::from_model(target_connection, &target.nodes))
                    .collect()
        }
    }
}