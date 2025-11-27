#include "graph_node_msgs/graph_node.hpp"

GraphNode::GraphNode()
{
    graphNode.id = 0;
    graphNode.position.x = 0.0;
    graphNode.position.y = 0.0;
    graphNode.position.z = 0.0;
    graphNode.score = 0.0;
}

GraphNode::~GraphNode() = default;

void GraphNode::setId(int id)
{
    graphNode.id = id;
}

void GraphNode::setPosition(const geometry_msgs::msg::Point& position)
{
    graphNode.position = position;
}

void GraphNode::setScore(double score)
{
    graphNode.score = score;
}

int GraphNode::getId() const
{
    return graphNode.id;
}

geometry_msgs::msg::Point GraphNode::getPosition() const
{
    return graphNode.position;
}

double GraphNode::getScore() const
{
    return graphNode.score;
}

graph_node_msgs::msg::GraphNode GraphNode::getGraphNode() const
{
    return graphNode;
}

bool GraphNode::getIsObserved() const
{
    return graphNode.is_observed;
}

void GraphNode::setIsObserved(bool is_observed)
{
    graphNode.is_observed = is_observed;
}