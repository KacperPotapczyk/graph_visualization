use std::{collections::HashSet, error::Error, fmt::Display, vec};

use super::{Graph, Node, Connection, Coordinates};
use serde::{Deserialize, Serialize};

const DEFAULT_FONT_SIZE: f32 = 12.0;
const DEFAULT_CHARGE: f64 = 1.0;
const DEFAULT_STIFFNESS: f64 = 1.0;

pub trait Dto {
    type Source;
    type Model;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>>;
    fn from_model(model: &Self::Model) -> Result<Self::Source, Box<dyn Error>>;
}

pub trait RelationDto {
    type Source;
    type Model;
    type RelatedDto;
    type RelatedModel;

    fn to_model(&self, related_dto: &Self::RelatedDto) -> Result<Self::Model, Box<dyn Error>>;
    fn from_model(model: &Self::Model, related_model: &Self::RelatedModel) -> Result<Self::Source, Box<dyn Error>>;
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
    type Source = Self;
    type Model = Coordinates;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>> {
        Ok(Self::Model {x: self.x, y: self.y})
    }

    fn from_model(model: &Self::Model) -> Result<Self::Source, Box<dyn Error>> {
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
    type Source = Self;
    type Model = Node;

    fn to_model(&self) -> Result<Self::Model, Box<dyn Error>> {
        Ok(Self::Model {
            coordinates: self.coordinates.to_model()?,
            label: self.description.clone().unwrap_or(self.label.clone()),
            font_size: self.font_size.unwrap_or(DEFAULT_FONT_SIZE),
            charge: self.charge.unwrap_or(DEFAULT_CHARGE)
        })
    }
    
    fn from_model(model: &Self::Model) -> Result<Self::Source, Box<dyn Error>> {
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
    type Source = Self;
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
    type Source = Self;
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

    fn from_model(model: &Self::Model) -> Result<Self::Source, Box<dyn Error>> {

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