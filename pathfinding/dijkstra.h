#ifndef PATHFINDING_DIJKSTRA_H_
#define PATHFINDING_DIJKSTRA_H_

#include <vector>
#include <algorithm>	// find_if, make_heap, pop_heap, sort
#include <memory>			// shared_ptr
#include <iostream>


#include "../data_structures/node.h"


namespace pathfinding
{

using sssp::Node;

template <class T>
class Dijkstra
{
public:
	Dijkstra(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier);
	std::vector<Node<T>> path();
private:
	void find_path();
	void retrieve_path(const std::shared_ptr<Node<T>>& node);

	T start;
	T goal;
	std::vector<std::shared_ptr<Node<T>>> unexplored;
	std::vector<std::shared_ptr<Node<T>>> explored;
	std::vector<Node<T>> shortest_path;
};

template <class T>
Dijkstra<T>::Dijkstra(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier)
{
	unexplored = frontier;

	auto start_it = std::find_if(unexplored.begin(), unexplored.end(),[&start_val](std::shared_ptr<Node<T>>& n)
	{
		return n->u == start_val;
	});

	auto goal_it = std::find_if(unexplored.begin(), unexplored.end(),[&goal_val](std::shared_ptr<Node<T>>& n)
	{
		return n->u == goal_val;
	});

	// check if start value is detected in domain
	if (start_it != unexplored.end())
	{
		// set vertex value of starting node to 0
		(*start_it)->v = 0;
		start = (*start_it)->u;
	}

	// check if goal value is detected in domain
	if (goal_it != unexplored.end())
	{
		goal = (*goal_it)->u;
	}
	find_path();
}

template <class T>
std::vector<Node<T>> Dijkstra<T>::path()
{
	return shortest_path;
}

template <class T>
void Dijkstra<T>::find_path()
{
	while (!unexplored.empty())
	{
		std::make_heap(unexplored.begin(), unexplored.end(), [](std::shared_ptr<Node<T>>& lhs, std::shared_ptr<Node<T>>& rhs)
		{
			return lhs->v + lhs->w > rhs->v + rhs->w;
		});

		// relax all nodes reachable from outgoing edges of current node
		for (auto& neighbor : unexplored.front()->adjacent)
		{
			if (unexplored.front()->v + neighbor.first < neighbor.second->v)
			{
				neighbor.second->v = unexplored.front()->v + neighbor.first;
				neighbor.second->parent = unexplored.front(); // set parent of relaxed node to current node
			}
		}
		explored.emplace_back(std::move(unexplored.front()));
		std::pop_heap(unexplored.begin(), unexplored.end());
		unexplored.pop_back();
	}

	auto goal_node_it = std::find_if(explored.begin(), explored.end(),[this](std::shared_ptr<Node<T>>& n)
	{
		return n->u == goal;
	});

	retrieve_path(*goal_node_it);
}

template <class T>
void Dijkstra<T>::retrieve_path(const std::shared_ptr<Node<T>>& node)
{
	if (node->parent != nullptr)
	{
		retrieve_path(node->parent);
	}
	shortest_path.emplace_back(*node);
}

// returns a vector with graph nodes in order of shortest path from start to goal
template <typename T>
std::vector<Node<T>> dijkstra(const T& start_val, const T& goal_val, const std::vector<std::shared_ptr<Node<T>>>& frontier)
{
	Dijkstra<T> d(start_val, goal_val, frontier);
	return d.path();
}


} // namespace pathfinding


#endif /* DIJKSTRA_H_ */


// dijkstra demo
//#include "data_structures/directed_graph.h"
//#include "pathfinding/dijkstra.h"
//
//int main()
//{
//	using pathfinding::sssp::Node;
//
//	data_structures::DirectedGraph<Node, char> frontier;
//
//	frontier.insert('A', 'B', 4);
//	frontier.insert('A', 'C', 2);
//	frontier.insert('B', 'C', 1);
//	frontier.insert('C', 'B', 1);
//	frontier.insert('C', 'D', 8);
//	frontier.insert('D', 'C', 8);
//	frontier.insert('B', 'D', 5);
//	frontier.insert('C', 'E', 10);
//	frontier.insert('E', 'C', 10);
//	frontier.insert('D', 'E', 2);
//	frontier.insert('E', 'D', 2);
//	frontier.insert('D', 'Z', 6);
//	frontier.insert('Z', 'D', 6);
//	frontier.insert('Z', 'E', 3);
//	frontier.insert('E', 'Z', 3);
//
//	auto d = pathfinding::dijkstra('A', 'Z', frontier.get());
//
//	for (auto& i : d)
//	{
//		std::cout << i.u << " ";
//	}
//	std::cout << std::endl;
//
//	return 0;
//}
