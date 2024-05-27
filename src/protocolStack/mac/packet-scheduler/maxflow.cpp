#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>
#include <iostream>
#include <vector>
#include <boost/variant.hpp>

struct NodeTypeGeneral {
    std::string name = "General";
};

struct NodeTypeUE {
    std::string name = "UE";
};

struct NodeTypeRBGset {
    std::string name = "RBGset";
};

struct VertexProperties {
    boost::variant<NodeTypeGeneral, NodeTypeUE, NodeTypeRBGset> node;
};

struct EdgeProperties {
    int capacity;
    int residual_capacity;
};

typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::directedS,
    VertexProperties, EdgeProperties> Graph;

typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

class MaxFlowGraph {
    public:
        MaxFlowGraph() = default;

    Vertex addVertex(const boost::variant<NodeTypeGeneral, NodeTypeUE, NodeTypeRBGset>& node) {
        Vertex v = boost::add_vertex(graph_);
        graph_[v].node = node;
        return v;
    }

    void addEdge(Vertex u, Vertex v, int capacity) {
        Edge e;
        bool added;
        boost::tie(e, added) = boost::add_edge(u, v, graph_);
        if (added) {
            graph_[e].capacity = capacity;
            graph_[e].residual_capacity = capacity;
        }
    }

    int computeMaxFlow(Vertex source, Vertex sink) {
        auto capacity_map = boost::get(&EdgeProperties::capacity, graph_);
        auto residual_capacity_map = boost::get(&EdgeProperties::residual_capacity, graph_);
        auto reverse_edge_map = boost::make_assoc_property_map(reverse_edges_);

        std::vector<boost::default_color_type> color(boost::num_vertices(graph_)); // corrected the typo
        std::vector<Edge> pred(boost::num_vertices(graph_));

        int max_flow = boost::edmonds_karp_max_flow(
            graph_, 
            source, 
            sink,
            capacity_map, 
            residual_capacity_map,
            reverse_edge_map, 
            boost::make_iterator_property_map(color.begin(), boost::get(boost::vertex_index, graph_)),
            boost::make_iterator_property_map(pred.begin(), boost::get(boost::vertex_index, graph_))
        );

        computeEdgeFlows();

        return max_flow;
    }

    void printEdgeFlows() const {
        auto edges = boost::edges(graph_);
        for (auto it = edges.first; it != edges.second; ++it) {
            Edge e = *it;
            std::cout << "Edge (" << boost::source(e, graph_) << " -> " 
                      << boost::target(e, graph_) << ") "
                      << "Flow: " << graph_[e].capacity - graph_[e].residual_capacity 
                      << " / " << graph_[e].capacity << std::endl;
        }
    }

    private:
        Graph graph_;
        std::map<Edge, Edge> reverse_edges_;

        void computeEdgeFlows() {
            auto edges = boost::edges(graph_);
            for (auto it  = edges.first; it != edges.second; ++it) {
                Edge e = *it;
                graph_[e].capacity -= graph_[e].residual_capacity;
            }
        }
};


