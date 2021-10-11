//! This module is an implementation of the A-Star pathfinding algorithm tailored for traversing a bespoke
//! collection of weighted hexagons in an Offset grid alignment. It's intended to calculate the most optimal path to a target
//! hexagon where you are traversing from the centre of one hexagon to the next along a line orthogonal to a hexagon edge.
//!
//! The calculations are dpendent on the layout of your hexagon grid.
//!
//! ## Hexagon Layout/Orientation
//!
//! There are different ways in which a hexagon grid can be portrayed which in turn affects the
//! discoverable neighbouring hexagons for path traversal. This library assumes that all hexagons have
//! been plotted across a plane where the origin points sits at the bottom left - a deviation from this
//! and the calcualtion simply won't work. Additionally a hexagon is herbey referred to as a 'node'.
//!
//! Each node has a label defining its position, known as `(column, row)`.
//!
//! ### Flat Topped - odd columns shifted up
//!
//! ```txt
//!              _______
//!             /       \
//!     _______/  (1,1)  \_______
//!    /       \         /       \
//!   /  (0,1)  \_______/  (2,1)  \
//!   \         /       \         /
//!    \_______/  (1,0)  \_______/
//!    /       \         /       \
//!   /  (0,0)  \_______/  (2,0)  \
//!   \         /       \         /
//!    \_______/         \_______/
//! ```
//!
//! The column shift changes how we discover nearby nodes. For instance if we take the node at
//! (0,0) and wish to discover the node to its North-East, (1,0), we can simply increment the `column` value by one.
//!
//! However if we take the node (1,0) and wish to discover its North-East node at (2,1) we have
//! to increment both the `column` value and the `row` value. I.e the calculation changes depending
//!  on whether the odd column has been shifted up or down.
//!
//! In full for a node in an even column we can calculate a nodes neighbours thus:
//!
//! ```txt
//! north      = (column, row + 1)
//! north-east = (column + 1, row)
//! south-east = (column + 1, row - 1)
//! south      = (column, row -1)
//! south-west = (column - 1, row - 1)
//! north-west = (column - 1, row)
//! ```
//!
//! And for a node in an odd column the node neighbours can be found:
//!
//! ```txt
//! north      = (column, row + 1)
//! north-east = (column + 1, row + 1)
//! south-east = (column + 1, row)
//! south      = (column, row -1)
//! south-west = (column - 1, row)
//! north-west = (column - 1, row + 1)
//! ```
//!
//! ### Flat Topped - odd columns shifted down
//!
//! ```txt
//!     _______           _______
//!    /       \         /       \
//!   /  (0,1)  \_______/  (2,1)  \
//!   \         /       \         /
//!    \_______/  (1,1)  \_______/
//!    /       \         /       \
//!   /  (0,0)  \_______/  (2,0)  \
//!   \         /       \         /
//!    \_______/  (1,0)  \_______/
//!            \         /
//!             \_______/
//! ```
//!
//! The column shift changes how we discover nearby nodes. For instance if we take the node at
//! (0,0) and wish to discover the node to its North-East, (1,1), we increment the `column` and
//! `row` values by one.
//!
//! However if we take the node (1,1) and wish to discover its North-East node at (2,1) we have to
//! only increment the `column` value by one.
//!
//! In full for a node in an even column we can calculate a nodes neighbours thus:
//!
//! ```txt
//! north      = (column, row + 1)
//! north-east = (column + 1, row + 1)
//! south-east = (column + 1, row)
//! south      = (column, row -1)
//! south-west = (column - 1, row)
//! north-west = (column - 1, row + 1)
//! ```
//!
//! And for a node in an odd column the node neighbours can be found:
//!
//! ```txt
//! north      = (column, row + 1)
//! north-east = (column + 1, row)
//! south-east = (column + 1, row - 1)
//! south      = (column, row -1)
//! south-west = (column - 1, row - 1)
//! north-west = (column - 1, row)
//! ```

use ::std::collections::HashMap;
use core::panic;
use crate::helpers::node_distance;
use crate::helpers::offset_to_cubic;
use crate::helpers::node_neighbours_offset;
use crate::HexOrientation;

/// From a starting node calculate the most efficient path to the end node
///
/// The `nodes` input is structured such:
///
/// * The keys are tuples of the nodes position in a grid with the origin being based on the bottom left, (x,y)
/// * The layout builds a square/rectangular like grid space
/// * The values are the complexity of traversing a particular node which is from the centre point of a side to its direct opposite
///
/// E.g
/// ```txt
///    ___________
///   /     ^     \
///  /      |      \
/// /  C    |       \
/// \       |       /
///  \      ▼      /
///   \___________/
/// ```
///
/// For a grid of perfectly flush hexagons the distance from the centre to the midpoint of an edge is the same in
/// all directions. This module is akin to idea that you wake up in a 'hexagon world' and you can only move from
/// the centre of one hexagon to another in a straight line, but while distance is static you'll find that as you
/// cross the boundary of one hexagon into another you'll suddenly be sprinting instead of slow-motion walking.
///
/// `min_column`, `max_column`, `min_row` and `max_row` indicate the boundary of the hexagon space and are exclusive.
/// For instance with a square grid space where the origin (bottom left) is `(0, 0)` and the top most right node is positioned at
/// `(3, 3) our `min_column` and `min_row` will be equal to `-1` and our `max_column` and `max_row` will both equal `4`.
///
/// `orientation` refers to your hexagonal grid layout.
///
/// The return Vec contains a number of tuples which for `0..n` show the best path to take
pub fn astar_path(
	start_node: (i32, i32),
	nodes: HashMap<(i32, i32), f32>,
	end_node: (i32, i32),
	min_column: i32,
	max_column: i32,
	min_row: i32,
	max_row: i32,
	orientation: HexOrientation,
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
	// max bounds are exclusive hence equal to or greater than
	if start_node.0 >= max_column || start_node.0 <= min_column || start_node.1 >= max_row || start_node.1 <= min_row {
		panic!("Start node is outside of searchable grid")
	}
	if end_node.0 >= max_column || end_node.0 <= min_column || end_node.1 >= max_row || end_node.1 <= min_row {
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
				calculate_node_weight(k, &end_node, &orientation),
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
		// println!("QUEUE");
		// println!("{:?}", queue);
		// remove the first element ready for processing
		let current_path = queue.swap_remove(0);
		// expand the node in the current path
		let available_nodes =
			node_neighbours_offset(current_path.0, &orientation, min_column, max_column, min_row, max_row);
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
					// search the queue to see if we already has a route to this node.
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
/// your current node to the end node. For the Offset grid we cannot compute the
/// number of jumps directly, instead we have to convert the Offset coordinates
/// of our nodes to the Cubic based coordinate system.
fn calculate_node_weight(
	current_node: &(i32, i32),
	end_node: &(i32, i32),
	orientation: &HexOrientation,
) -> f32 {
	let cubic_start = offset_to_cubic((current_node.0 as i32, current_node.1 as i32), orientation);
	let cubic_end = offset_to_cubic((end_node.0 as i32, end_node.1 as i32), orientation);
	// by finding the distance between nodes we're effectively finding the 'ring' it sits on which is the number of jumps to it
	node_distance(cubic_start, cubic_end) as f32
}

#[cfg(test)]
mod tests {
	use crate::astar_offset::astar_path;
	use crate::astar_offset::calculate_node_weight;
	use crate::HexOrientation;
	use std::collections::HashMap;

	#[test]
	/// Calcualtes a nodes weight where the end node is located in the +ve x-y direction
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (2,2)  \ ----> /  (4,4)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_positive() {
		let source: (i32, i32) = (2, 2);
		let end_node: (i32, i32) = (4, 4);
		let orientation = HexOrientation::FlatTopOddUp;
		let weight = calculate_node_weight(&source, &end_node, &orientation);
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
	fn node_weight_negative() {
		let source: (i32, i32) = (4, 4);
		let end_node: (i32, i32) = (2, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let weight = calculate_node_weight(&source, &end_node, &orientation);
		let actual_weight = 3.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calcualtes a node weight where the end node is located in the +ve x direction and -ve y direction
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (2,4)  \ ----> /  (4,2)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_positive_and_negative() {
		let source: (i32, i32) = (2, 4);
		let end_node: (i32, i32) = (4, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let weight = calculate_node_weight(&source, &end_node, &orientation);
		let actual_weight = 3.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     E     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    C:3    /           \    C:9    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    C:4    /           \    C:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    C:9    /           \    C:4    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     S     \    C:2    /           \    C:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \            /
	///   \    C:1    /           \    C:2    /
	///    \_________/             \_________/
	///  ```
	fn astar_up_right() {
		let start_node: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, 1), 1.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((0, 3), 3.0);
		nodes.insert((1, 0), 2.0);
		nodes.insert((1, 1), 9.0);
		nodes.insert((1, 2), 4.0);
		nodes.insert((1, 3), 2.0);
		nodes.insert((2, 0), 2.0);
		nodes.insert((2, 1), 6.0);
		nodes.insert((2, 2), 8.0);
		nodes.insert((2, 3), 9.0);
		nodes.insert((3, 0), 3.0);
		nodes.insert((3, 1), 4.0);
		nodes.insert((3, 2), 5.0);
		nodes.insert((3, 3), 2.0);
		let end_node: (i32, i32) = (3, 3);
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			min_column,
			max_column,
			min_row,
			max_row,
			orientation,
		);
		let actual = vec![(0, 0), (0, 1), (0, 2), (1, 2), (2, 3), (3, 3)];
		assert_eq!(actual, best);
	}
	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     E     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    C:3    /           \    C:9    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    C:4    /           \    C:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    C:9    /           \    C:4    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    C:6    /           \    C:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     S     \    C:2    /           \    C:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \            /
	///   \    C:1    /           \    C:2    /
	///    \_________/             \_________/
	///  ```
	fn astar_right_up() {
		let start_node: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, 1), 6.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((0, 3), 3.0);
		nodes.insert((1, 0), 2.0);
		nodes.insert((1, 1), 9.0);
		nodes.insert((1, 2), 4.0);
		nodes.insert((1, 3), 2.0);
		nodes.insert((2, 0), 2.0);
		nodes.insert((2, 1), 6.0);
		nodes.insert((2, 2), 8.0);
		nodes.insert((2, 3), 9.0);
		nodes.insert((3, 0), 3.0);
		nodes.insert((3, 1), 4.0);
		nodes.insert((3, 2), 5.0);
		nodes.insert((3, 3), 2.0);
		let end_node: (i32, i32) = (3, 3);
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			min_column,
			max_column,
			min_row,
			max_row,
			orientation,
		);
		let actual = vec![(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2), (3, 3)];
		assert_eq!(actual, best);
	}
	#[test]
	/// Calcualtes the best path from S (3, 3) to E (0, 0)
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     S     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    C:3    /           \    C:9    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    C:4    /           \    C:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    C:9    /           \    C:4    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     E     \    C:2    /           \    C:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \            /
	///   \    C:1    /           \    C:2    /
	///    \_________/             \_________/
	///  ```
	fn astar_down_left() {
		let start_node: (i32, i32) = (3, 3);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, 1), 1.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((0, 3), 3.0);
		nodes.insert((1, 0), 2.0);
		nodes.insert((1, 1), 9.0);
		nodes.insert((1, 2), 4.0);
		nodes.insert((1, 3), 2.0);
		nodes.insert((2, 0), 2.0);
		nodes.insert((2, 1), 6.0);
		nodes.insert((2, 2), 8.0);
		nodes.insert((2, 3), 9.0);
		nodes.insert((3, 0), 3.0);
		nodes.insert((3, 1), 4.0);
		nodes.insert((3, 2), 5.0);
		nodes.insert((3, 3), 2.0);
		let end_node: (i32, i32) = (0, 0);
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			min_column,
			max_column,
			min_row,
			max_row,
			orientation,
		);
		let actual = vec![(3, 3), (2, 3), (1, 2), (0, 2), (0, 1), (0, 0)];
		assert_eq!(actual, best);
	}
	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///                 _________               _________
	///                /         \             /         \
	///               /           \           /     E     \
	///     _________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:2    /
	///  /    (0,3)    \_________/    (2,3)    \_________/
	///  \             /         \             /         \
	///   \    C:3    /           \    C:4    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:8    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /           \    C:9    /           \    C:9    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///    /         \             /         \             /
	///   /     S     \    C:2    /           \    C:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \            /
	///   \    C:1    /           \    C:6    /
	///    \_________/             \_________/
	///  ```
	fn astar_left_down() {
		let start_node: (i32, i32) = (3, 3);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, 1), 1.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((0, 3), 3.0);
		nodes.insert((1, 0), 2.0);
		nodes.insert((1, 1), 9.0);
		nodes.insert((1, 2), 2.0);
		nodes.insert((1, 3), 2.0);
		nodes.insert((2, 0), 6.0);
		nodes.insert((2, 1), 6.0);
		nodes.insert((2, 2), 8.0);
		nodes.insert((2, 3), 4.0);
		nodes.insert((3, 0), 3.0);
		nodes.insert((3, 1), 9.0);
		nodes.insert((3, 2), 5.0);
		nodes.insert((3, 3), 2.0);
		let end_node: (i32, i32) = (0, 0);
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			min_column,
			max_column,
			min_row,
			max_row,
			orientation,
		);
		let actual = vec![(3, 3), (2, 3), (1, 2), (0, 2), (0, 1), (0, 0)];
		assert_eq!(actual, best);
	}
	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///     _________               _________
	///    /         \             /         \
	///   /           \           /     E     \
	///  /    (0,3)    \_________/    (2,3)    \_________
	///  \             /         \             /         \
	///   \    C:3    /           \    C:4    /           \
	///    \_________/    (1,3)    \_________/    (3,3)    \
	///    /         \             /         \             /
	///   /           \    C:2    /           \    C:5    /
	///  /    (0,2)    \_________/    (2,2)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:8    /           \
	///    \_________/    (1,2)    \_________/    (3,2)    \
	///    /         \             /         \             /
	///   /           \    C:9    /           \    C:9    /
	///  /    (0,1)    \_________/    (2,1)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:6    /           \
	///    \_________/    (1,1)    \_________/    (3,1)    \
	///    /         \             /         \             /
	///   /     S     \    C:2    /           \    C:3    /
	///  /    (0,0)    \_________/    (2,0)    \_________/
	///  \             /         \             /         \
	///   \    C:1    /           \    C:6    /           \
	///    \_________/    (1,0)    \_________/    (3,0)    \
	///              \             /         \             /
	///               \    C:4    /           \    C:2    /
	///                \_________/             \_________/
	///  ```
	fn astar_odd_column_down() {
		let start_node: (i32, i32) = (0, 0);
		let mut nodes: HashMap<(i32, i32), f32> = HashMap::new();
		nodes.insert((0, 0), 1.0);
		nodes.insert((0, 1), 1.0);
		nodes.insert((0, 2), 1.0);
		nodes.insert((0, 3), 3.0);
		nodes.insert((1, 0), 4.0);
		nodes.insert((1, 1), 2.0);
		nodes.insert((1, 2), 9.0);
		nodes.insert((1, 3), 2.0);
		nodes.insert((2, 0), 6.0);
		nodes.insert((2, 1), 6.0);
		nodes.insert((2, 2), 8.0);
		nodes.insert((2, 3), 4.0);
		nodes.insert((3, 0), 2.0);
		nodes.insert((3, 1), 3.0);
		nodes.insert((3, 2), 9.0);
		nodes.insert((3, 3), 5.0);
		let end_node: (i32, i32) = (2, 3);
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddDown;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			min_column,
			max_column,
			min_row,
			max_row,
			orientation,
		);
		let actual = vec![(0, 0), (0, 1), (0, 2), (1, 3), (2, 3)];
		assert_eq!(actual, best);
	}
}
