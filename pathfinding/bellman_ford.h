#ifndef PATHFINDING_BELLMAN_FORD_H_
#define PATHFINDING_BELLMAN_FORD_H_

#include <vector>
#include <algorithm>	// find_if, make_heap, pop_heap, sort
#include <memory>			// shared_ptr
#include <iostream>

#include "../data_structures/node.h"

namespace pathfinding
{

using sssp::Node;

template <class T>
class BellmanFord
{
public:
	BellmanFord(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier);
	std::vector<Node<T>> path();
private:
	void find_path();
	void retrieve_path(const std::shared_ptr<Node<T>>& node);

	T start;
	T goal;
	std::vector<std::shared_ptr<Node<T>>> frontier;
	std::vector<Node<T>> shortest_path;
};

template <class T>
BellmanFord<T>::BellmanFord(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier_)
{
	frontier = frontier_;

	auto start_it = std::find_if(frontier.begin(), frontier.end(),[&start_val](std::shared_ptr<Node<T>>& n)
	{
		return n->u == start_val;
	});

	auto goal_it = std::find_if(frontier.begin(), frontier.end(),[&goal_val](std::shared_ptr<Node<T>>& n)
	{
		return n->u == goal_val;
	});

	// check if start value is detected in domain
	if (start_it != frontier.end())
	{
		// set vertex value of starting node to 0
		(*start_it)->v = 0;
		start = (*start_it)->u;
	}

	// check if goal value is detected in domain
	if (goal_it != frontier.end())
	{
		goal = (*goal_it)->u;
	}
	find_path();
}

template <class T>
std::vector<Node<T>> BellmanFord<T>::path()
{
	return shortest_path;
}

template <class T>
void BellmanFord<T>::find_path()
{
	for (auto& node : frontier)
	{
		for (auto& neighbor : node->adjacent)
		{
			// relax all neighboring nodes reachable from the current node
			if (node->v + neighbor.first < neighbor.second->v)
			{
				neighbor.second->v = node->v + neighbor.first;
				neighbor.second->parent = node;
			}
		}
	}

	bool cycle_detected = false;

	for (auto& node : frontier)
	{
		for (auto& neighbor : node->adjacent)
		{
			// attempt to relax all neighboring nodes from current node again
			// will only trigger condition if a negative weight cycle is present
			if (node->v + neighbor.first < neighbor.second->v)
			{
				//std::cout << "Cycle detected" << std::endl;
				cycle_detected = true;

				// early exit out of nested for loop
				goto loop_early_exit;
			}
		}
	}

	loop_early_exit:

	if (!cycle_detected)
	{
		auto goal_node_it = std::find_if(frontier.begin(), frontier.end(),[this](std::shared_ptr<Node<T>>& n)
		{
			return n->u == goal;
		});

		retrieve_path(*goal_node_it);
	}
}

// recursively follow parent pointers from goal node to source node and insert them into shortest_path vector
template <class T>
void BellmanFord<T>::retrieve_path(const std::shared_ptr<Node<T>>& node)
{
	if (node->parent != nullptr)
	{
		retrieve_path(node->parent);
	}
	shortest_path.emplace_back(*node);
}

// if no cycle detected, returns a vector with graph nodes in order of shortest path from start to goal
// returns an empty vector otherwise
template <typename T>
std::vector<Node<T>> bellman_ford(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier)
{
	BellmanFord<T> bf(start_val, goal_val, frontier);
	return bf.path();
}


}


#endif /* PATHFINDING_BELLMAN_FORD_H_ */


// bellman_ford demo
//#include "data_structures/directed_graph.h"
//#include "pathfinding/bellman_ford.h"
//
//int main()
//{
//	using pathfinding::sssp::Node;
//
//	data_structures::DirectedGraph<Node, char> frontier;
//
//	frontier.insert('A', 'B', 2);
//	frontier.insert('B', 'C', 1);
//	frontier.insert('C', 'D', -4); // causes a negative cycle
//	frontier.insert('D', 'B', 2);
//	frontier.insert('C', 'E', 3);
//
//	auto bf = pathfinding::bellman_ford('A', 'E', frontier.get());
//
//	for (auto i : bf)
//	{
//		std::cout << i.u << " ";
//	}
//	std::cout << std::endl;
//
//	return 0;
//}
