#ifndef DATA_STRUCTURES_UNDIRECTED_GRAPH_H_
#define DATA_STRUCTURES_UNDIRECTED_GRAPH_H_

#include <vector>
#include <algorithm>	// find_if
#include <memory>			// shared_ptr

#include "directed_graph.h"


namespace data_structures
{


template <template<class> class NodeT, class T>
class UndirectedGraph : public DirectedGraph<NodeT, T>
{
public:
	UndirectedGraph();

private:
	void make_adjacent(std::shared_ptr<NodeT<T>>& outgoing_node, std::shared_ptr<NodeT<T>>& incoming_node, const double& edge_weight);
};

template <template<class> class NodeT, class T>
UndirectedGraph<NodeT, T>::UndirectedGraph()
: DirectedGraph<NodeT, T>()
{

}

template <template<class> class NodeT, class T>
void UndirectedGraph<NodeT, T>::make_adjacent(std::shared_ptr<NodeT<T>>& outgoing_node, std::shared_ptr<NodeT<T>>& incoming_node, const double& edge_weight)
{
	auto adj_incoming_it = std::find_if(outgoing_node->adjacent.begin(), outgoing_node->adjacent.end(),[&incoming_node](std::pair<double, std::shared_ptr<NodeT<T>>>& outgoing_adj)
	{
		return outgoing_adj.second->u == incoming_node->u;
	});

	auto adj_outgoing_it = std::find_if(incoming_node->adjacent.begin(), incoming_node->adjacent.end(),[&outgoing_node]( std::pair<double, std::shared_ptr<NodeT<T>>>& incoming_adj)
	{
		return incoming_adj.second->u == outgoing_node->u;
	});

	if (adj_incoming_it == outgoing_node->adjacent.end())
	{
		outgoing_node->adjacent.emplace_back(edge_weight, incoming_node);
	}
	if (adj_outgoing_it == incoming_node->adjacent.end())
	{
		incoming_node->adjacent.emplace_back(edge_weight, outgoing_node);
	}
}


} // namespace data_structures


#endif /* UTIL_UNDIRECTED_GRAPH_H_ */
