use std::{collections::HashSet, error::Error, fmt::Display, vec};

use super::{Graph, Node, Connection, Coordinates};
use serde::{Deserialize, Serialize};

const DEFAULT_FONT_SIZE: f32 = 12.0;
const DEFAULT_CHARGE: f64 = 1.0;
const DEFAULT_STIFFNESS: f64 = 1.0;

pub trait Dto {
    type DtoType;
    type Model;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>>;
    fn from_model(model: &Self::Model) -> Result<Self::DtoType, Box<dyn Error>>;
}

pub trait RelationDto {
    type DtoType;
    type Model;
    type RelatedDto;
    type RelatedModel;

    fn to_model(&self, related_dto: &Self::RelatedDto) -> Result<Self::Model, Box<dyn Error>>;
    fn from_model(model: &Self::Model, related_model: &Self::RelatedModel) -> Result<Self::DtoType, Box<dyn Error>>;
}

#[derive(Debug)]
struct DtoError(String);

impl Display for DtoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Error during dto mapping: {}", self.0)
    }
}

impl Error for DtoError {}

#[derive(Serialize, Deserialize, Debug)]
pub struct CoordinatesDto {
    x: f64,
    y: f64,
}

impl Dto for CoordinatesDto {
    type DtoType = Self;
    type Model = Coordinates;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>> {
        Ok(Self::Model {x: self.x, y: self.y})
    }

    fn from_model(model: &Self::Model) -> Result<Self::DtoType, Box<dyn Error>> {
        Ok(Self {x: model.x, y: model.y})
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
    type DtoType = Self;
    type Model = Node;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>> {
        Ok(Self::Model {
            coordinates: self.coordinates.to_model()?,
            label: self.description.clone().unwrap_or(self.label.clone()),
            font_size: self.font_size.unwrap_or(DEFAULT_FONT_SIZE),
            charge: self.charge.unwrap_or(DEFAULT_CHARGE)
        })
    }
    
    fn from_model(model: &Self::Model) -> Result<Self::DtoType, Box<dyn Error>> {
        Ok(Self {
            coordinates: CoordinatesDto::from_model(&model.coordinates)?,
            label: model.label.clone(),
            description: None,
            font_size: Some(model.font_size),
            charge: Some(model.charge)
        })
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConnectionDto {
    node1: String,
    node2: String,
    stiffness: Option<f64>,
}

impl RelationDto for ConnectionDto {
    type DtoType = Self;
    type RelatedModel = Vec<Node>;
    type RelatedDto = Vec<NodeDto>;
    type Model = Connection;

    fn to_model(&self, node_dtos: &Self::RelatedDto) -> Result<Self::Model, Box<dyn Error>> {

        let index1 = node_dtos.iter().position(|node| node.label == self.node1);
        let index2 = node_dtos.iter().position(|node| node.label == self.node2);

        Ok(Self::Model {
            index1: match index1 {
                Some(index) => index,
                None => return Err(Box::new(
                    DtoError("Connection is invalid! Could not find node with label: ".to_string() + &self.node1)
                ))
            },
            index2: match index2 {
                Some(index) => index,
                None => return Err(Box::new(
                    DtoError("Connection is invalid! Could not find node with label: ".to_string() + &self.node2)
                ))
            },
            stiffness: self.stiffness.unwrap_or(DEFAULT_STIFFNESS)
        })
    }

    fn from_model(model: &Self::Model, nodes: &Self::RelatedModel) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            node1: nodes[model.index1].label.clone(),
            node2: nodes[model.index2].label.clone(),
            stiffness: Some(model.stiffness)
        })
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct GraphDto {
    nodes: Vec<NodeDto>,
    connections: Vec<ConnectionDto>,
}

impl Dto for GraphDto {
    type DtoType = Self;
    type Model = Graph;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>> {

        let mut node_labels = HashSet::new();
        let mut nodes = vec![];
        for node_dto in &self.nodes {
            if node_labels.contains(&node_dto.label) {
                return Err(Box::new(
                    DtoError("Node is invalid! Duplicate label: ".to_string() + &node_dto.label)
                ))
            }
            node_labels.insert(node_dto.label.clone());
            nodes.push(node_dto.to_model()?);
        }

        let mut connections = vec![];
        for connection_dto in &self.connections {
            connections.push(connection_dto.to_model(&self.nodes)?);
        }

        Ok(Self::Model {
            nodes,
            connections
        })
    }

    fn from_model(model: &Self::Model) -> Result<Self::DtoType, Box<dyn Error>> {

        let mut nodes = vec![];
        for node in &model.nodes {
            nodes.push(NodeDto::from_model(node)?);
        }

        let mut connections = vec![];
        for connection in &model.connections {
            connections.push(ConnectionDto::from_model(connection, &model.nodes)?);
        }

        Ok(Self {
            nodes,
            connections
        })
    }
}

#[cfg(test)]
mod tests {
    use assert_approx_eq::assert_approx_eq;
    use super::*;
    const EPS: f64 = 1e-6;

    #[test]
    fn coordinates_dto_to_coordinates() {
        let x = 1.324;
        let y = -6.32;
        
        let coordinates_dto = CoordinatesDto {x, y};
        let coordinates = coordinates_dto.to_model().unwrap();

        assert_approx_eq!(x, coordinates.x, EPS);
        assert_approx_eq!(y, coordinates.y, EPS);
    }

    #[test]
    fn coordinates_to_coordinates_dto() {
        let x = -234.13;
        let y = 894.113;

        let coordinates = Coordinates {x, y};
        let coordinates_dto = CoordinatesDto::from_model(&coordinates).unwrap();

        assert_approx_eq!(x, coordinates_dto.x, EPS);
        assert_approx_eq!(y, coordinates_dto.y, EPS);
    }

    #[test]
    fn node_dto_to_node() {
        let x = 3.14;
        let y = -1.0;
        let label = "node_label".to_string();
        let description = "node_description".to_string();
        let font_size = 9.0;
        let charge = 1e2;

        let node_dto = NodeDto {
            coordinates: CoordinatesDto {x, y},
            label,
            description: Some(description.clone()),
            font_size: Some(font_size),
            charge: Some(charge)
        };
        let node = node_dto.to_model().unwrap();

        assert_approx_eq!(x, node.coordinates.x, EPS);
        assert_approx_eq!(y, node.coordinates.y, EPS);
        assert_eq!(description, node.label);
        assert_approx_eq!(font_size, node.font_size, 1e-6);
        assert_approx_eq!(charge, node.charge, EPS);
    }

    #[test]
    fn node_dto_with_defaults_to_node() {
        let x = 3.14;
        let y = -1.0;
        let label = "node_label".to_string();

        let node_dto = NodeDto {
            coordinates: CoordinatesDto {x, y},
            label: label.clone(),
            description: None,
            font_size: None,
            charge: None
        };
        let node = node_dto.to_model().unwrap();

        assert_approx_eq!(x, node.coordinates.x, EPS);
        assert_approx_eq!(y, node.coordinates.y, EPS);
        assert_eq!(label, node.label);
        assert_approx_eq!(DEFAULT_FONT_SIZE, node.font_size, 1e-6);
        assert_approx_eq!(DEFAULT_CHARGE, node.charge, EPS);
    }
    
    #[test]
    fn node_to_node_dto() {
        let x = 3.14;
        let y = -1.0;
        let label = "node_label".to_string();
        let font_size = 9.0;
        let charge = 1e2;

        let node = Node {
            coordinates: Coordinates {x, y},
            label: label.clone(),
            font_size,
            charge
        };

        let node_dto = NodeDto::from_model(&node).unwrap();
        
        assert_approx_eq!(x, node_dto.coordinates.x, EPS);
        assert_approx_eq!(y, node_dto.coordinates.y, EPS);
        assert_eq!(label, node_dto.label);
        assert_approx_eq!(font_size, node_dto.font_size.unwrap(), 1e-6);
        assert_approx_eq!(charge, node_dto.charge.unwrap(), EPS);
    }

    #[test]
    fn connection_dto_to_connection() {
        let node_1_label = "A".to_string();
        let node_2_label = "B".to_string();
        let node_3_label = "C".to_string();
        let node_1_coordinates = CoordinatesDto {x: 1.0, y: 2.0};
        let node_2_coordinates = CoordinatesDto {x: 1.0, y: 2.0};
        let node_3_coordinates = CoordinatesDto {x: 1.0, y: 2.0};
        let stiffness = 10.0;

        let node_dtos = vec![
            NodeDto {coordinates: node_1_coordinates, label: node_1_label.clone(), description: None, font_size: None, charge: None},
            NodeDto {coordinates: node_2_coordinates, label: node_2_label.clone(), description: None, font_size: None, charge: None},
            NodeDto {coordinates: node_3_coordinates, label: node_3_label.clone(), description: None, font_size: None, charge: None}
        ];

        let connection_dto = ConnectionDto {
            node1: node_1_label.clone(),
            node2: node_3_label.clone(),
            stiffness: Some(stiffness)
        };

        let connection = connection_dto.to_model(&node_dtos).unwrap();

        assert_eq!(0, connection.index1);
        assert_eq!(2, connection.index2);
        assert_approx_eq!(stiffness, connection.get_stifness(), EPS);
    }

    #[test]
    fn connection_to_connection_dto() {
        let node_1_label = "A".to_string();
        let node_2_label = "B".to_string();
        let node_3_label = "C".to_string();
        let node_1_coordinates = Coordinates {x: 1.0, y: 2.0};
        let node_2_coordinates = Coordinates {x: 1.0, y: 2.0};
        let node_3_coordinates = Coordinates {x: 1.0, y: 2.0};
        let stiffness = 10.0;

        let nodes = vec![
            Node {coordinates: node_1_coordinates, label: node_1_label.clone(), font_size: 12.0, charge: 1e2},
            Node {coordinates: node_2_coordinates, label: node_2_label.clone(), font_size: 12.0, charge: 1e2},
            Node {coordinates: node_3_coordinates, label: node_3_label.clone(), font_size: 12.0, charge: 1e2}
        ];

        let connection = Connection {
            index1: 2,
            index2: 0,
            stiffness
        };

        let connection_dto = ConnectionDto::from_model(&connection, &nodes).unwrap();

        assert_eq!(node_3_label, connection_dto.node1);
        assert_eq!(node_1_label, connection_dto.node2);
        assert_approx_eq!(stiffness, connection_dto.stiffness.unwrap(), EPS);
    }

    #[test]
    fn graph_dto_to_graph() {
        let node_1_label = "A".to_string();
        let node_2_label = "B".to_string();
        let node_1_coordinates = CoordinatesDto {x: 1.0, y: 2.0};
        let node_2_coordinates = CoordinatesDto {x: 10.0, y: 20.0};
        let stiffness = 10.0;

        let node_dtos = vec![
            NodeDto {coordinates: node_1_coordinates, label: node_1_label.clone(), description: None, font_size: None, charge: None},
            NodeDto {coordinates: node_2_coordinates, label: node_2_label.clone(), description: None, font_size: None, charge: None},
        ];

        let connection_dtos = vec![
            ConnectionDto {node1: node_2_label.clone(), node2: node_1_label.clone(), stiffness: Some(stiffness)}
        ];

        let graph_dto = GraphDto {nodes: node_dtos, connections: connection_dtos};
        let graph = graph_dto.to_model().unwrap();

        assert_eq!(2, graph.nodes.len());
        assert_eq!(1, graph.connections.len());

        assert_eq!(node_1_label, graph.nodes[0].label);
        assert_eq!(node_2_label, graph.nodes[1].label);
        assert_eq!(1, graph.connections[0].index1);
        assert_eq!(0, graph.connections[0].index2);
    }

    #[test]
    fn graph_to_graph_dto() {
        let node_1_label = "A".to_string();
        let node_2_label = "B".to_string();
        let node_1_coordinates = Coordinates {x: 1.0, y: 2.0};
        let node_2_coordinates = Coordinates {x: 10.0, y: 20.0};
        let stiffness = 10.0;

        let nodes = vec![
            Node {coordinates: node_1_coordinates, label: node_1_label.clone(), font_size: 12.0, charge: 1e3},
            Node {coordinates: node_2_coordinates, label: node_2_label.clone(), font_size: 12.0, charge: 1e3},
        ];

        let connections = vec![
            Connection {index1: 1, index2: 0, stiffness}
        ];

        let graph = Graph {nodes, connections};
        let graph_dto = GraphDto::from_model(&graph).unwrap();

        assert_eq!(2, graph_dto.nodes.len());
        assert_eq!(1, graph_dto.connections.len());

        assert_eq!(node_1_label, graph_dto.nodes[0].label);
        assert_eq!(node_2_label, graph_dto.nodes[1].label);
        assert_eq!(node_2_label, graph_dto.connections[0].node1);
        assert_eq!(node_1_label, graph_dto.connections[0].node2);
    }
}