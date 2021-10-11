//! A-Star pathfinding algorithm for an Axial grid alignment.
//!
//! Axial coordinates use the convention of `q` for column and `r` for row. In the example below the `r` is a diagonal row.
//! For hexagon layouts where the pointy tops are facing up the calculations remain exactly the same as you're effectively
//! just rotating the grid by 30 degrees making `r` horizontal and `q` diagonal.
//!
//!```txt
//!             _______
//!            /   0   \
//!    _______/         \_______
//!   /  -1   \      -1 /   1   \
//!  /         \_______/         \
//!  \       0 /   q   \      -1 /
//!   \_______/         \_______/
//!   /  -1   \       r /   1   \
//!  /         \_______/         \
//!  \       1 /   0   \       0 /
//!   \_______/         \_______/
//!           \       1 /
//!            \_______/
//!```
//!

use ::std::collections::HashMap;
use core::panic;
use crate::helpers::node_distance;
use crate::helpers::axial_to_cubic;
use crate::helpers::node_neighbours_axial;

/// From a starting node calculate the most efficient path to the end node
///
/// The `nodes` input is structured such:
///
/// * The keys are tuples of the nodes positions in the form (q,r)
/// * The layout builds a circular-like grid
/// * The values are the complexity of traversing a particular node
///
/// `count_rings` is the number of rings around the origin `(0, 0)` of the circular hexagonal grid. It
/// is an inclusive value. NB: `count_rings` is NOT the nubmer of rings around the start or end node, it
/// is explicitly the number of rings around the origin of our hexagon grid
/// 
/// For instance from our origin of `(0, 0)`:
/// 
///```txt
///                              _________
///                             /    0    \
///                            /           \
///                  _________/    Ring2    \_________
///                 /   -1    \          -2 /    1    \
///                /           \           /           \
///      _________/    Ring2    \_________/    Ring2    \_________
///     /   -2    \          -1 /    0    \          -2 /    2    \
///    /           \           /           \           /           \
///   /    Ring2    \_________/    Ring1    \_________/    Ring2    \
///   \           0 /   -1    \          -1 /    1    \          -2 /
///    \           /           \           /           \           /
///     \_________/    Ring1    \_________/    Ring1    \_________/
///     /   -2    \           0 /    0    \          -1 /    2    \
///    /           \           /           \           /           \
///   /    Ring2    \_________/             \_________/    Ring2    \
///   \           1 /   -1    \           0 /    1    \          -1 /
///    \           /           \           /           \           /
///     \_________/    Ring1    \_________/    Ring1    \_________/
///     /   -2    \           1 /    0    \           0 /    2    \
///    /           \           /           \           /           \
///   /    Ring2    \_________/    Ring1    \_________/    Ring2    \
///   \           2 /   -1    \           1 /    1    \           0 /
///    \           /           \           /           \           /
///     \_________/    Ring2    \_________/    Ring2    \_________/
///               \           2 /    0    \           1 /
///                \           /           \           /
///                 \_________/    Ring2    \_________/
///                           \           2 /
///                            \           /
///                             \_________/
///  ```
/// 
/// We have 2 rings of hexagons surrounding it.
///
/// The return Vec contains a number of tuples which for `0..n` show the best path to take
pub fn astar_path(
	start_node: (i32, i32),
	nodes: HashMap<(i32, i32), f32>,
	end_node: (i32, i32),
	count_rings: i32,
) -> Vec<(i32, i32)> {
	// ensure nodes data contains start and end points
	if !nodes.contains_key(&start_node) {
		panic!(
			"Node data does not contain start node ({},{})",
			start_node.0, start_node.1
		);
	}
	if !nodes.contains_key(&end_node) {
		panic!(
			"Node data does not contain end node ({},{})",
			end_node.0, end_node.1
		);
	}
	// ensure start and end nodes are within the max bounds of the grid
	// we use the ring boundary hence it's easier to check this in cubic coords
	let cubic_start = axial_to_cubic(start_node);
	let cubic_end = axial_to_cubic(end_node);
	if cubic_start.0.abs() > count_rings || cubic_start.1.abs() > count_rings || cubic_start.2.abs() > count_rings {
		panic!("Start node is outside of searchable grid")
	}
	if cubic_end.0.abs() > count_rings || cubic_end.1.abs() > count_rings || cubic_end.2.abs() > count_rings {
		panic!("End node is outside of searchable grid")
	}
	// calculate the weight of each node and produce a new combined data set of everthing we need
	// keys are nodes and values are a tuple of (complexity, weight)
	let mut nodes_weighted: HashMap<(i32, i32), (f32, f32)> = HashMap::new();
	// calculate a weighting for each node based on its distance from the end node
	for (k, v) in nodes.iter() {
		nodes_weighted.insert(
			k.to_owned(),
			(
				v.to_owned(),
				calculate_node_weight(k, &end_node),
			),
		);
	}

	let start_weight: f32 = match nodes_weighted.get(&start_node) {
		Some(x) => x.1,
		None => panic!("Unable to find node weight"),
	};

	// every time we process a new node we add it to a map
	// if a node has already been recorded then we replace it if it has a better a-star score (smaller number)
	// otherwise we discard it.
	// this is used to optimise the searching whereby if we find a new path to a previously
	// discovered node we can quickly decide to discard or explore the new route
	let mut node_astar_scores: HashMap<(i32, i32), f32> = HashMap::new();
	// add starting node a-star score to data set (starting node score is just its weight)
	node_astar_scores.insert(start_node.clone(), start_weight.clone());

	// create a queue of nodes to be processed based on discovery
	// of form (current_node, a_star_score, vec_previous_nodes_traversed, total_complexity)
	let mut queue = Vec::new();
	// add starting node to queue
	queue.push((
		start_node.clone(),
		start_weight, // we haven't moved so starting node score is just its weight
		Vec::<(i32, i32)>::new(),
		0.0,
	));

	// target node will eventually be shifted to first of queue so finish processing once it arrives, meaning that we know the best path
	while queue[0].0 != end_node {
		// remove the first element ready for processing
		let current_path = queue.swap_remove(0);
		// expand the node in the current path
		let available_nodes =
			node_neighbours_axial(current_path.0, count_rings);
		// process each new path
		for n in available_nodes.iter() {
			let previous_complexities: f32 = current_path.3.clone();
			let current_node_complexity: f32 = match nodes_weighted.get(&current_path.0) {
				Some(x) => x.0 * 0.5,
				None => panic!("Unable to find current node complexity for {:?}", &n),
			};
			let target_node_complexity: f32 = match nodes_weighted.get(&n) {
				Some(x) => x.0 * 0.5,
				None => panic!("Unable to find target node complexity for {:?}", &n),
			};
			// calculate its fields
			let complexity =
				previous_complexities + target_node_complexity + current_node_complexity;
			let target_weight: f32 = match nodes_weighted.get(&n) {
				Some(x) => x.1,
				None => panic!("Unable to find node weight for {:?}", &n),
			};
			let astar = a_star_score(complexity, target_weight);
			let mut previous_nodes_traversed = current_path.2.clone();
			previous_nodes_traversed.push(current_path.0);
			// update the a-star data set
			if node_astar_scores.contains_key(&n) {
				if node_astar_scores.get(&n) >= Some(&astar) {
					// data set contains a worse score so update the set with the better score
					node_astar_scores.insert(n.clone(), astar);
					// search the queue to see if we already have a route to this node.
					// If we do but this new path is better then replace it, otherwise discard
					let mut new_queue_item_required_for_node = true;
					for mut q in queue.iter_mut() {
						new_queue_item_required_for_node = false;
						if &q.0 == n {
							// if existing score is worse then replace the queue item
							if &q.1 >= &astar {
								q.1 = astar;
								q.2 = previous_nodes_traversed.clone();
								q.3 = complexity;
							}
						}
					}
					// queue doesn't contain a route to this node, as we have now found a better route
					// update the queue with it so it can be explored
					if new_queue_item_required_for_node {
						queue.push((n.clone(), astar, previous_nodes_traversed, complexity));
					}
				}
			} else {
				// no record of node and new path required in queue
				node_astar_scores.insert(n.clone(), astar);
				queue.push((n.clone(), astar, previous_nodes_traversed, complexity));
			}
		}

		// sort the queue by a-star sores so each loop processes the best
		queue.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
	}
	let mut best_path = queue[0].2.clone();
	// add end node to data
	best_path.push(end_node);
	return best_path;
}

/// Determines a score to rank a chosen path, lower scores are better
fn a_star_score(complexity: f32, weighting: f32) -> f32 {
	complexity + weighting
}

/// Finds a nodes weight based on the number of 'jumps' you'd have to make from
/// your current node to the end node. For the Axial grid we cannot compute the
/// number of jumps directly, instead we have to convert the Axial coordinates
/// of our nodes to the Cubic based coordinate system.
fn calculate_node_weight(
	current_node: &(i32, i32),
	end_node: &(i32, i32),
) -> f32 {
	let cubic_start = axial_to_cubic((current_node.0, current_node.1));
	let cubic_end = axial_to_cubic((end_node.0, end_node.1));
	// by finding the distance between nodes we're effectively finding the 'ring' it sits on which is the number of jumps to it
	node_distance(cubic_start, cubic_end) as f32
}

#[cfg(test)]
mod tests {
	use crate::astar_axial::astar_path;
	use crate::astar_axial::calculate_node_weight;
	use std::collections::HashMap;

	#[test]
	/// Calcualtes a nodes weight
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (2,2)  \ ----> /  (4,4)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_down() {
		let source: (i32, i32) = (1, -1);
		let end_node: (i32, i32) = (1, 2);
		let weight = calculate_node_weight(&source, &end_node);
		let actual_weight = 3.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calculates a nodes weight where the end node is located in the -ve x-y direction
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (4,4)  \ ----> /  (2,2)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_towards_origin() {
		let source: (i32, i32) = (-2, 3);
		let end_node: (i32, i32) = (0, 0);
		let weight = calculate_node_weight(&source, &end_node);
		let actual_weight = 3.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///                              _________
	///                             /    0    \
	///                            /           \
	///                  _________/     C:1     \_________
	///                 /   -1    \          -2 /    1    \
	///                /           \           /           \
	///      _________/     C:2     \_________/     C:14    \_________
	///     /   -2    \          -1 /    0    \          -2 /    2    \
	///    /           \           /           \           /     E     \
	///   /     C:1     \_________/     C:1     \_________/     C:1     \
	///   \           0 /   -1    \          -1 /    1    \          -2 /
	///    \           /           \           /           \           /
	///     \_________/     C:7     \_________/     C:15    \_________/
	///     /   -2    \           0 /    0    \          -1 /    2    \
	///    /           \           /     S     \           /           \
	///   /     C:8     \_________/     C:1     \_________/     C:1     \
	///   \           1 /   -1    \           0 /    1    \          -1 /
	///    \           /           \           /           \           /
	///     \_________/     C:6     \_________/     C:14    \_________/
	///     /   -2    \           1 /    0    \           0 /    2    \
	///    /           \           /           \           /           \
	///   /     C:1     \_________/     C:2     \_________/     C:1     \
	///   \           2 /   -1    \           1 /    1    \           0 /
	///    \           /           \           /           \           /
	///     \_________/     C:3     \_________/     C:1     \_________/
	///               \           2 /    0    \           1 /
	///                \           /           \           /
	///                 \_________/     C:1     \_________/
	///                           \           2 /
	///                            \           /
	///                             \_________/
	///  ```
	fn astar_tick() {
		let start_node: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, -1), 1.0);
		nodes.insert((1, -1), 15.0);
		nodes.insert((1, 0), 14.0);
		nodes.insert((0, 1), 2.0);
		nodes.insert((-1, 1), 6.0);
		nodes.insert((-1, 0), 7.0);
		nodes.insert((0, -2), 1.0);
		nodes.insert((1, -2), 14.0);
		nodes.insert((2, -2), 1.0);
		nodes.insert((2, -1), 1.0);
		nodes.insert((2, 0), 1.0);
		nodes.insert((1, 1), 1.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((-1, 2), 3.0);
		nodes.insert((-2, 2), 1.0);
		nodes.insert((-2, 1), 8.0);
		nodes.insert((-2, 0), 1.0);
		nodes.insert((-1, -1), 2.0);
		let end_node: (i32, i32) = (2, -2);
		let rings = 2;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			rings,
		);
		let actual = vec![(0, 0), (0, 1), (1, 1), (2, 0), (2, -1), (2, -2)];
		assert_eq!(actual, best);
	}
}
