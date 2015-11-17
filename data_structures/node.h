#ifndef UTIL_NODE_H_
#define UTIL_NODE_H_

#include <vector>
#include <memory>	// shared_ptr


namespace pathfinding
{
namespace sssp	// single source shortest path
{


template <class T>
struct Node
{
	Node(const T& node_id, const double& edge_weight, const double& vertex_value)
	{
		u = node_id;
		v = vertex_value;
		w = edge_weight;
		parent = nullptr;
	}

	T u;
	double v;
	double w;
	std::shared_ptr<Node> parent;
	std::vector<std::pair<double, std::shared_ptr<Node>>> adjacent;
};


}  // namespace sssp
} // namespace pathfinding


#endif /* UTIL_NODE_H_ */
