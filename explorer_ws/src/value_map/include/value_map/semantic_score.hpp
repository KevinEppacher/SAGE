#ifndef SEMANTIC_SCORE_HPP
#define SEMANTIC_SCORE_HPP

#include "std_msgs/msg/header.hpp"

class SemanticScore {
public:
    SemanticScore() : score(0.0) {}
    explicit SemanticScore(double initial_score) : score(initial_score) {}

    double getScore() const { return score; }
    void setScore(double new_score) { score = new_score; }

    const std_msgs::msg::Header & getHeader() const { return header; }
    void setHeader(const std_msgs::msg::Header & new_header) { header = new_header; }

private:
    double score;
    std_msgs::msg::Header header;
};

#endif // SEMANTIC_SCORE_HPP
