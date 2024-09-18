mod force_solver;
pub mod dto;

use std::{error::Error, fs};
use std::fs::File;
use std::io::BufWriter;

use dto::{Dto, GraphDto};

use printpdf::*;

const DEFAULT_PAGE_PADDING: Mm = Mm(10.0);

#[derive(Debug)]
pub struct Coordinates {
    x: f64,
    y: f64,
}

impl Coordinates {
    pub fn distance(&self, coordinate: &Coordinates) -> f64 {
        ((self.x - coordinate.x).powf(2.0) + (self.y - coordinate.y).powf(2.0)).sqrt()
    }
}

impl Clone for Coordinates {
    fn clone(&self) -> Self {
        Coordinates { x: self.x, y: self.y }
    }
}

#[derive(Debug)]
pub struct Node {
    coordinates: Coordinates,
    label: String,
    font_size: f32,
    charge: f64,
}

impl Node {
    pub fn new(x: f64, y: f64, label: String, font_size: f32, charge: f64) -> Self {
        Node {
            coordinates: Coordinates {x, y},
            label,
            font_size,
            charge
        }
    }

    pub fn get_charge(&self) -> f64 {
        self.charge
    }
}

#[derive(Debug)]
pub struct Connection {
    index1: usize,
    index2: usize,
    stiffness: f64,
}

impl Connection {
    pub fn new(index1: usize, index2: usize, stiffness: f64) -> Self {
        Connection {
            index1,
            index2,
            stiffness
        }
    }
}

impl Connection {
    pub fn get_stifness(&self) -> f64 {
        self.stiffness
    }
}

#[derive(Debug)]
pub struct Graph {
    nodes: Vec<Node>,
    connections: Vec<Connection>,
}

impl Graph {
    fn move_nodes(&mut self, new_coordinates: &Vec<Coordinates>) {
        for i in 0..new_coordinates.len() {
            self.nodes[i].coordinates = new_coordinates[i].clone();
        }
    }
}

struct PdfNode {
    left: Mm,
    bottom: Mm,
    text: String,
    text_left: Mm,
    text_bottom: Mm,
    font_size: f32,
    width: Pt,
    heigth: Pt
}

impl PdfNode {
    fn from_node(node: &Node) -> Self {
        let font_size = node.font_size;
        let font_width = 0.6 * font_size;
        let label_width = Pt(node.label.len() as f32 * font_width);

        let left_padding = Pt(font_width);
        let right_padding = Pt(font_width);
        let bottom_padding = Pt(font_size / 2.0);
        let top_padding = Pt(font_size / 2.0);

        PdfNode {
            left: Mm(node.coordinates.x as f32) - Mm::from(left_padding) - Mm::from(label_width / 2.0),
            bottom: Mm(node.coordinates.y as f32) - Mm::from(right_padding) - Mm::from(Pt(font_size) / 2.0),
            text: node.label.clone(),
            text_left: Mm(node.coordinates.x as f32) - Mm::from(label_width / 2.0),
            text_bottom: Mm(node.coordinates.y as f32) - Mm::from(Pt(font_size) / 2.0),
            font_size,
            width: left_padding + label_width + right_padding,
            heigth: bottom_padding + Pt(font_size) + top_padding
        }
    }

    fn get_rectangle(&self) -> Rect {
        Rect {
            ll: Point { x: self.left.into_pt(), y: self.bottom.into_pt() },
            ur: Point { x: self.left.into_pt() + self.width, y: self.bottom.into_pt() + self.heigth },
            mode: path::PaintMode::FillStroke,
            winding: path::WindingOrder::NonZero
        }
    }

    fn get_center_point(&self) -> Point {
        Point {
            x: self.left.into_pt() + self.width / 2.0,
            y: self.bottom.into_pt() + self.heigth / 2.0
        }
    }

    fn get_top(&self) -> Mm {
        self.bottom + Mm::from(self.heigth)
    }

    fn get_rigth(&self) -> Mm {
        self.left + Mm::from(self.width)
    }

    fn move_by(&mut self, x: Mm, y: Mm) {
        self.left += x;
        self.bottom += y;
        self.text_left += x;
        self.text_bottom += y;
    }
}

pub fn read_graph_from_file(file_path: String) -> Result<Graph, Box<dyn Error>> {

    let content = fs::read_to_string(file_path)?;
    let graph_dto: GraphDto = serde_json::from_str(&content)?;
    let graph: Graph = graph_dto.to_model()?;
    Ok(graph)
}

pub fn organize_graph(graph: &mut Graph, max_iterations: u32) {

    let new_coordinates = force_solver::solve(&graph.nodes, &graph.connections, max_iterations);
    graph.move_nodes(&new_coordinates);
}

pub fn print_graph_to_pdf(graph: &Graph, output_file_name: String) -> Result<(), Box<dyn Error>> {

    // println!("Graph: {:?}", graph);

    let mut pdf_nodes: Vec<PdfNode> = graph.nodes.iter()
            .map(|node| PdfNode::from_node(node))
            .collect();

    let (document, page, graph_layer) = create_page(&mut pdf_nodes);
    let current_layer = document.get_page(page).get_layer(graph_layer);
    let font = document.add_builtin_font(BuiltinFont::Courier)?;

    print_graph(current_layer, graph, &pdf_nodes, font);

    document.save(&mut BufWriter::new(File::create(output_file_name)?))?;
    Ok(())
}

fn create_page(pdf_nodes: &mut Vec<PdfNode>) -> (PdfDocumentReference, PdfPageIndex, PdfLayerIndex) {

    let min_bottom = pdf_nodes.iter()
            .map(|pdf_node| pdf_node.bottom)
            .min()
            .unwrap();
    
    let min_left = pdf_nodes.iter()
            .map(|pdf_node| pdf_node.left)
            .min()
            .unwrap();
    
    let mut dx = Mm(0.0);
    let mut dy = Mm(0.0);
    
    if min_left < DEFAULT_PAGE_PADDING {
        dx = DEFAULT_PAGE_PADDING - min_left;
    }
    
    if min_bottom < DEFAULT_PAGE_PADDING {
        dy = DEFAULT_PAGE_PADDING - min_bottom;
    }

    if dx > Mm(0.0) || dy > Mm(0.0) {
        for pdf_node in &mut *pdf_nodes {
            pdf_node.move_by(dx, dy);
        }
    }
    
    let max_top = pdf_nodes.iter()
            .map(|pdf_node| pdf_node.get_top())
            .max()
            .unwrap()
            + DEFAULT_PAGE_PADDING;
    
    let max_rigth = pdf_nodes.iter()
            .map(|pdf_node| pdf_node.get_rigth())
            .max()
            .unwrap()
            + DEFAULT_PAGE_PADDING;
    
    let (document, page, graph_layer) = PdfDocument::new("Graph", max_rigth, max_top, "Graph");
    (document, page, graph_layer)
}

fn print_graph(current_layer: PdfLayerReference, graph: &Graph, pdf_nodes: &Vec<PdfNode>, font: IndirectFontRef) {

    let fill_color = Color::Rgb(Rgb::new(1.0, 1.0, 1.0, None));
    let font_color = Color::Rgb(Rgb::new(0.0, 0.0, 0.0, None));
    let outline_color = Color::Rgb(Rgb::new(0.0, 0.0, 0.0, None));

    current_layer.set_fill_color(fill_color);
    current_layer.set_outline_color(outline_color);
    
    for connection in &graph.connections {
        let line_points = vec![
            (pdf_nodes[connection.index1].get_center_point(), false),
            (pdf_nodes[connection.index2].get_center_point(), false)
        ];

        let line = Line {
            points: line_points,
            is_closed: false,
        };

        current_layer.add_line(line);
    }

    for pdf_node in pdf_nodes {
        current_layer.add_rect(pdf_node.get_rectangle());
    }

    current_layer.set_fill_color(font_color);
    for pdf_node in pdf_nodes {
        current_layer.use_text(
            &pdf_node.text, 
            pdf_node.font_size, 
            pdf_node.text_left, 
            pdf_node.text_bottom, 
            &font
        );
    }
}
