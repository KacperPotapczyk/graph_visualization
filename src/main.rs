use graph_visualization::{organize_graph, print_graph_to_pdf, read_graph_from_file, save_graph_to_file};
use clap::Parser;

/// Graph visualization tool
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// File name of input graph data
    #[arg(short, long)]
    graph: String,

    /// File name of output pdf
    #[arg(short, long)]
    pdf: String,

    /// File name of output graph
    #[arg(short, long)]
    out_graph: Option<String>,

    /// Organize graph using solver
    #[arg(short, long, default_value_t = false)]
    solve: bool,

    /// Number of iterations
    #[arg(short = 'i', long, default_value_t = 50)]
    max_iterations: u32,
}

fn main() {
    let args = Args::parse();

    let mut graph = read_graph_from_file(args.graph).unwrap();
    
    if args.solve {
        organize_graph(&mut graph, args.max_iterations);
    }
    print_graph_to_pdf(&graph, args.pdf).unwrap();

    if args.out_graph.is_some() {
        save_graph_to_file(args.out_graph.unwrap(), &graph).unwrap();
    }
}