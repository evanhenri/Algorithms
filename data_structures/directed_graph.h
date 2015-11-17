#ifndef DATA_STRUCTURES_DIRECTED_GRAPH_H_
#define DATA_STRUCTURES_DIRECTED_GRAPH_H_

#include <vector>
#include <limits>			// numeric_limits
#include <algorithm>	// find_if
#include <memory>			// shared_ptr

#include "node.h"


namespace data_structures
{


template <template<class> class NodeT, class T>
class DirectedGraph
{
public:
	DirectedGraph();
	virtual ~DirectedGraph();

	void insert(const T& node_id, const T& adj_id, const double& edge_weight);
	std::vector<std::shared_ptr<NodeT<T>>> get();

protected:
	virtual void make_adjacent(std::shared_ptr<NodeT<T>>& outgoing_node, std::shared_ptr<NodeT<T>>& incoming_node, const double& edge_weight);
	double infinity;
	std::vector<std::shared_ptr<NodeT<T>>> domain;
};

// define inifinity value to initialize graph vertex values
template <template<class> class NodeT, class T>
DirectedGraph<NodeT, T>::DirectedGraph()
{
	infinity = std::numeric_limits<double>::max();
}

template <template<class> class NodeT, class T>
DirectedGraph<NodeT, T>::~DirectedGraph()
{

}

// creates new node(s) for outgoing_id and incoming_id if not found in existing graph domain
template <template<class> class NodeT, class T>
void DirectedGraph<NodeT, T>::insert(const T& outgoing_id, const T& incoming_id, const double& edge_weight)
{
	auto outgoing_it = std::find_if(domain.begin(), domain.end(),[&outgoing_id](const std::shared_ptr<NodeT<T>>& existing_n)
	{
		return existing_n->u == outgoing_id;
	});

	auto incoming_it = std::find_if(domain.begin(), domain.end(),[&incoming_id](const std::shared_ptr<NodeT<T>>& existing_n)
	{
		return existing_n->u == incoming_id;
	});

	if (outgoing_it == domain.end())
	{
		auto outgoing_n = std::make_shared<NodeT<T>>(outgoing_id, edge_weight, infinity);

		if (incoming_it == domain.end())
		{
			auto incoming_n = std::make_shared<NodeT<T>>(incoming_id, edge_weight, infinity);
			make_adjacent(outgoing_n, incoming_n, edge_weight);

			domain.emplace_back(outgoing_n);
			domain.emplace_back(incoming_n);
		}
		else
		{
			make_adjacent(outgoing_n, *incoming_it, edge_weight);
			domain.emplace_back(outgoing_n);
		}
	}
	else if (incoming_it == domain.end())
	{
		auto incoming_n = std::make_shared<NodeT<T>>(incoming_id, edge_weight, infinity);
		make_adjacent(*outgoing_it, incoming_n, edge_weight);
		domain.emplace_back(incoming_n);
	}
	else
	{
		make_adjacent(*outgoing_it, *incoming_it, edge_weight);
	}
}

template <template<class> class NodeT, class T>
std::vector<std::shared_ptr<NodeT<T>>> DirectedGraph<NodeT,T>::get()
{
	return domain;
}

// If the vertex value of incoming_node is not found in the adjacency list of outgoing_node,
// the edge weight and a pointer to the incoming_node at the other end of that edge is inserted
// into the adjacency list of the outgoing node
template <template<class> class NodeT, class T>
void DirectedGraph<NodeT, T>::make_adjacent(std::shared_ptr<NodeT<T>>& outgoing_node, std::shared_ptr<NodeT<T>>& incoming_node, const double& edge_weight)
{
	auto adj_incoming_it = std::find_if(outgoing_node->adjacent.begin(), outgoing_node->adjacent.end(),[&incoming_node](std::pair<double, std::shared_ptr<NodeT<T>>>& outgoing_adj)
	{
		return outgoing_adj.second->u == incoming_node->u;
	});

	if (adj_incoming_it == outgoing_node->adjacent.end())
	{
		outgoing_node->adjacent.emplace_back(edge_weight, incoming_node);
	}
}


} // namespace data_structures


#endif /* UTIL_DIRECTED_GRAPH_H_ */
