//! This library is an implementation of the A-Star pathfinding algorithm tailored for traversing a bespoke
//! collection of weighted hexagons. It's intended to calculate the most optimal path to a target
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
//! NB: the `column` and `row` values can never be negative.
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

/// Specifies the orientation of the hexagon space. This is important for determining the available neighbouring nodes during expansion.
/// 
/// Flat-top odd columns moved up
///```txt
///       ___
///   ___/ O \
///  / E \___/
///  \___/
///```
/// Flat-top odd columns moved down
/// ```txt
///   ___
///  / E \___
///  \___/ O \
///      \___/
/// ```
pub enum HexOrientation {
	FlatTopOddUp,
	FlatTopOddDown,
}

/// From a starting node calculate the most efficient path to the end node
///
/// The `nodes` input is structured such:
///
/// * The keys are tuples of the nodes position in a grid with the (0,0) origin being based on the bottom left, (x,y) heading along right and up
/// * The values are the complexity of traversing a particular node
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
/// all directions. This library is akin to idea that you wake up in a 'hexagon world' and you can only move from
/// the centre of one hexagon to another in a straight line, but while distance is static you'll find that as you
/// cross the boundary of one hexagon into another you'll suddenly be sprinting instead of slow-motion walking.
/// 
/// `max_column` and `max_row` indicate the boundary of the hexagon space and are exclusive. For instance with a
/// square grid space where the top most right node is positioned at (3, 3) our `max_column` and `max_row`
/// will both equal `4`.
/// 
/// `orientation` refers to your hexagonal grid layout.
///
/// The return Vec contains a number of tuples which for `0..n` show the best path to take
pub fn astar_path(
	start_node: (usize, usize),
	nodes: HashMap<(usize, usize), f32>,
	end_node: (usize, usize),
	max_column: usize,
	max_row: usize,
	orientation: HexOrientation,
) -> Vec<(usize, usize)> {
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
	if start_node.0 >= max_column || start_node.1 >= max_row {
		panic!("Start node is outside of searchable grid")
	}
	if end_node.0 >= max_column || end_node.1 >= max_row {
		panic!("End node is outside of searchable grid")
	}
	// calculate the weight of each node and produce a new combined data set of everthing we need
	// keys are nodes and values are a tuple of (complexity, weight)
	let mut nodes_weighted: HashMap<(usize, usize), (f32, f32)> = HashMap::new();
	// calculate a weighting for each node based on its distance from the end node
	for (k, v) in nodes.iter() {
		nodes_weighted.insert(
			k.to_owned(),
			(v.to_owned(), calculate_node_weight(k, &end_node)),
		);
	}

	let start_astar = a_star_score(nodes_weighted[&start_node].0, nodes_weighted[&start_node].1);
	// create a queue of nodes to be processed based on discovery
	// of form (current_node, a_star_score, vec_previous_nodes_traversed, total_complexity)
	let mut queue = Vec::new();
	// add starting node to queue
	queue.push((
		start_node.clone(),
		start_astar,
		Vec::<(usize, usize)>::new(),
		nodes_weighted[&start_node].0,
	));

	// target node will eventually be shifted to first of queue so finish processing once it arrives, meaning that we know the best path
	while queue[0].0 != end_node {
		// remove the first element ready for processing
		let current_path = queue.swap_remove(0);
		// expand the node in the current path
		let available_nodes =
			expand_neighrbor_nodes(current_path.0, &orientation, max_column, max_row);
		// process each new path
		for n in available_nodes.iter() {
			let previous_complexities: f32 = current_path.3.clone();
			let target_node_complexity: f32 = nodes_weighted[&n].0.clone();
			// calculate its fields
			let complexity = previous_complexities + target_node_complexity;
			let astar = a_star_score(complexity, nodes_weighted[&n].1);
			let mut previous_nodes_traversed = current_path.2.clone();
			previous_nodes_traversed.push(current_path.0);
			// search the queue to see if we already has a route to this node.
			// If we do but this new path is better then replace it, otherwise discard
			let mut no_record_of_node = true;
			for mut q in queue.iter_mut() {
				// target node alread has a path but it is less efficient, so replace it
				if &q.0 == n && &q.1 > &astar {
					q.1 = astar;
					q.2 = previous_nodes_traversed.clone();
					q.3 = complexity;
					no_record_of_node = false;
				} else if &q.0 == n && &q.1 < &astar {
					// we discard the new path as the queue already contains a better one
				};
			}
			if no_record_of_node {
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

/// Finds the neighbouring nodes based on the orientation of the hexagon grid.
/// During expansion it will check for overflow i.e expanding to a node at a negative position - this is deemed invalid
/// and considered a boundary of the hexagon grid space.
/// Likewise the `max_column` and `max_row` inputs define the outer boundary of the grid space, note they are exclusive values.
/// This means that for most source hexagons 6 neighbours will be expanded but for those lining the boundaries fewer neighrbors will be discovered
fn expand_neighrbor_nodes(
	source: (usize, usize),
	orientation: &HexOrientation,
	max_column: usize,
	max_row: usize,
) -> Vec<(usize, usize)> {
	let mut neighbours = Vec::new();
	// starting from north round a tile clockwise
	// https://www.redblobgames.com/grids/hexagons/
	match orientation {
		//       ___
		//   ___/   \
		//  /   \___/
		//  \___/
		// flat topped arrangemnt of hexagns, odd columns shifted up
		HexOrientation::FlatTopOddUp => {
			// even column
			if source.0 % 2 == 0 {
				// north
				if source.1 + 1 < max_row {
					neighbours.push((source.0, source.1 + 1));
				};
				// north-east
				if source.0 + 1 < max_column {
					neighbours.push((source.0 + 1, source.1));
				};
				// south-east
				if source.0 + 1 < max_column && source.1.checked_sub(1) != None {
					neighbours.push((source.0 + 1, source.1 - 1));
				};
				// south
				if source.1.checked_sub(1) != None {
					neighbours.push((source.0, source.1 - 1));
				};
				// south-west
				if source.0.checked_sub(1) != None && source.1.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1 - 1));
				}
				// north-west
				if source.0.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1));
				}
				return neighbours;
			} else {
				// odd column
				// north
				if source.1 + 1 < max_row {
					neighbours.push((source.0, source.1 + 1));
				}
				// north-east
				if source.0 + 1 < max_column && source.1 + 1 < max_row {
					neighbours.push((source.0 + 1, source.1 + 1))
				}
				// south-east
				if source.0 + 1 < max_column {
					neighbours.push((source.0 + 1, source.1));
				}
				// south
				if source.1.checked_sub(1) != None {
					neighbours.push((source.0, source.1 - 1));
				}
				// south-west
				if source.0.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1));
				}
				// north-east
				if source.0.checked_sub(1) != None && source.1 + 1 < max_row {
					neighbours.push((source.0 - 1, source.1 + 1))
				}
				return neighbours;
			}
		}
		//   ___
		//  /   \___
		//  \___/   \
		//      \___/
		// flat topped arrangemnt of hexagns, odd columns shifted down
		HexOrientation::FlatTopOddDown => {
			// even column
			if source.0 % 2 == 0 {
				// north
				if source.1 + 1 < max_row {
					neighbours.push((source.0, source.1 + 1));
				}
				// north-east
				if source.0 + 1 < max_column && source.1 + 1 < max_row {
					neighbours.push((source.0 + 1, source.1 + 1));
				}
				// south-east
				if source.0 + 1 < max_column {
					neighbours.push((source.0 + 1, source.1));
				}
				// south
				if source.1.checked_sub(1) != None {
					neighbours.push((source.0, source.1 - 1));
				}
				// south-west
				if source.0.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1));
				}
				// north-west
				if source.0.checked_sub(1) != None && source.1 + 1 < max_row {
					neighbours.push((source.0 - 1, source.1 + 1));
				}
				return neighbours;
			} else {
				// odd column
				// north
				if source.1 + 1 < max_row {
					neighbours.push((source.0, source.1 + 1))
				}
				// north-east
				if source.0 + 1 < max_column {
					neighbours.push((source.0 + 1, source.1))
				}
				// south-east
				if source.0 + 1 < max_column && source.1.checked_sub(1) != None {
					neighbours.push((source.0 + 1, source.1 - 1))
				}
				// south
				if source.1.checked_sub(1) != None {
					neighbours.push((source.0, source.1 - 1))
				}
				// south-west
				if source.0.checked_sub(1) != None && source.1.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1 - 1))
				}
				// north-west
				if source.0.checked_sub(1) != None {
					neighbours.push((source.0 - 1, source.1))
				}
				return neighbours;
			}
		}
	}
}
/// Determines a score to rank a chosen path, lower scores are better
fn a_star_score(complexity: f32, weighting: f32) -> f32 {
	complexity + weighting
}

/// Finds a nodes weight based on its distance from the end_node. As we are
/// dealing with a usize based grid we avoid overflow by verifying the 'direction' we
/// traverse. E.g the difference between current node '(0,0)' and end node '(2,2)' means
/// we have to subtract the current node from the end node. Conversely if the current node
/// is '(2,2)' and end node '(1,1)' we have to subtract the end node from the current node to avoid overflow
fn calculate_node_weight(current_node: &(usize, usize), end_node: &(usize, usize)) -> f32 {
	let delta_x_squared: f32 = if current_node.0 <= end_node.0 {
		(end_node.0 - current_node.0).pow(2) as f32
	} else {
		(current_node.0 - end_node.0).pow(2) as f32
	};
	let delta_y_squared: f32 = if current_node.1 <= end_node.1 {
		(end_node.1 - current_node.1).pow(2) as f32
	} else {
		(current_node.1 - end_node.1).pow(2) as f32
	};
	delta_x_squared + delta_y_squared
}

#[cfg(test)]
mod tests {
	use crate::astar_path;
	use crate::calculate_node_weight;
	use crate::expand_neighrbor_nodes;
	use crate::HexOrientation;
	use std::collections::HashMap;

	#[test]
	/// Expands an even columned node in a flat topped odd column shifted up alignment and tests that the correct neighbours are returned
	/// ```txt
	///             _______
	///            /       \
	///    _______/  (2,3)  \_______
	///   /       \         /       \
	///  /  (1,2)  \_______/  (3,2)  \
	///  \         /       \         /
	///   \_______/  (2,2)  \_______/
	///   /       \    S    /       \
	///  /  (1,1)  \_______/  (3,1)  \
	///  \         /       \         /
	///   \_______/  (2,1)  \_______/
	///           \         /
	///            \_______/
	///  ```
	fn flat_top_odd_up_even_node_neighbours() {
		let source: (usize, usize) = (2, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let max_column = 4;
		let max_row = 4;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let actual = vec![(2, 3), (3, 2), (3, 1), (2, 1), (1, 1), (1, 2)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// Expands an odd columned node in a flat topped odd column shifted down alignment and tests that the correct neighbours are returned
	/// ```txt
	///             _______
	///            /       \
	///    _______/  (3,3)  \_______
	///   /       \         /       \
	///  /  (2,3)  \_______/  (4,3)  \
	///  \         /       \         /
	///   \_______/  (3,2)  \_______/
	///   /       \    S    /       \
	///  /  (2,2)  \_______/  (4,2)  \
	///  \         /       \         /
	///   \_______/  (3,1)  \_______/
	///           \         /
	///            \_______/
	///  ```
	fn flat_top_odd_up_odd_node_neighbours() {
		let source: (usize, usize) = (3, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let max_column = 5;
		let max_row = 5;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let actual = vec![(3, 3), (4, 3), (4, 2), (3, 1), (2, 2), (2, 3)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// Expands an even columned node in a flat topped odd column shifted down alignment and tests that the correct neighbours are returned
	/// ```txt
	///             _______
	///            /       \
	///    _______/  (2,3)  \_______
	///   /       \         /       \
	///  /  (1,3)  \_______/  (3,3)  \
	///  \         /       \         /
	///   \_______/  (2,2)  \_______/
	///   /       \    S    /       \
	///  /  (1,2)  \_______/  (3,2)  \
	///  \         /       \         /
	///   \_______/  (2,1)  \_______/
	///           \         /
	///            \_______/
	///  ```
	fn flat_top_odd_down_even_node_neighbours() {
		let source: (usize, usize) = (2, 2);
		let orientation = HexOrientation::FlatTopOddDown;
		let max_column = 4;
		let max_row = 4;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let actual = vec![(2, 3), (3, 3), (3, 2), (2, 1), (1, 2), (1, 3)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// Expands an odd columned node in a flat topped odd column shifted down alignment and tests that the correct neighbours are returned
	/// ```txt
	///             _______
	///            /       \
	///    _______/  (3,3)  \_______
	///   /       \         /       \
	///  /  (2,2)  \_______/  (4,2)  \
	///  \         /       \         /
	///   \_______/  (3,2)  \_______/
	///   /       \    S    /       \
	///  /  (2,1)  \_______/  (4,1)  \
	///  \         /       \         /
	///   \_______/  (3,1)  \_______/
	///           \         /
	///            \_______/
	///  ```
	fn flat_top_odd_down_odd_node_neighbours() {
		let source: (usize, usize) = (3, 2);
		let orientation = HexOrientation::FlatTopOddDown;
		let max_column = 5;
		let max_row = 5;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let actual = vec![(3, 3), (4, 2), (4, 1), (3, 1), (2, 1), (2, 2)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// Expands a node next to a negative boundary to ensure only valid discovered nodes are returned
	/// ```txt
	///    _______
	///   /       \
	///  /  (0,1)  \_______
	///  \         /       \
	///   \_______/  (1,0)  \
	///   /       \         /
	///  /  (0,0)  \_______/
	///  \    S    /
	///   \_______/
	///  ```
	fn neighbors_along_negative_boundary() {
		let source: (usize, usize) = (0, 0);
		let orientation = HexOrientation::FlatTopOddUp;
		let max_column = 5;
		let max_row = 5;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let expected_neighbour_count = 2;
		assert_eq!(expected_neighbour_count, neighbours.len());
	}
	#[test]
	/// Expands a node next to a positive boundary to ensure only valid discovered nodes are returned
	/// ```txt
	///    _______
	///   /       \
	///  /  (1,1)  \_______
	///  \         /       \
	///   \_______/  (2,1)  \
	///   /       \    S    /
	///  /  (1,0)  \_______/
	///  \         /       \
	///   \_______/  (2,0)  \
	///           \         /
	///            \_______/
	///  ```
	fn neighbors_along_positive_boundary() {
		let source: (usize, usize) = (2, 1);
		let orientation = HexOrientation::FlatTopOddUp;
		let max_column = 3;
		let max_row = 2;
		let neighbours = expand_neighrbor_nodes(source, &orientation, max_column, max_row);
		let expected_neighbour_count = 3;
		assert_eq!(expected_neighbour_count, neighbours.len());
	}
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
		let source: (usize, usize) = (2, 2);
		let end_node: (usize, usize) = (4, 4);
		let weight = calculate_node_weight(&source, &end_node);
		let actual_weight = 8.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calcualtes a nodes weight where the end node is located in the -ve x-y direction
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (4,4)  \ ----> /  (2,2)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_negative() {
		let source: (usize, usize) = (4, 4);
		let end_node: (usize, usize) = (2, 2);
		let weight = calculate_node_weight(&source, &end_node);
		let actual_weight = 8.0;
		assert_eq!(actual_weight, weight);
	}
	#[test]
	/// Calcualtes a node weight where the end node is located in the -ve x direction and +ve y direction
	/// ```txt
	///    _______           _______
	///   /       \         /       \
	///  /  (2,4)  \ ----> /  (4,2)  \
	///  \         /       \         /
	///   \_______/         \_______/
	///  ```
	fn node_weight_positive_and_negative() {
		let source: (usize, usize) = (2, 4);
		let end_node: (usize, usize) = (4, 2);
		let weight = calculate_node_weight(&source, &end_node);
		let actual_weight = 8.0;
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
	fn astar_north() {
		let start_node: (usize, usize) = (0, 0);
		let mut nodes: HashMap<(usize, usize), f32> = HashMap::new();
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
		let end_node: (usize, usize) = (3, 3);
		let max_column = 4;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			max_column,
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
	fn astar_south() {
		let start_node: (usize, usize) = (0, 0);
		let mut nodes: HashMap<(usize, usize), f32> = HashMap::new();
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
		let end_node: (usize, usize) = (3, 3);
		let max_column = 4;
		let max_row = 4;
		let orientation = HexOrientation::FlatTopOddUp;
		let best = astar_path(
			start_node,
			nodes,
			end_node,
			max_column,
			max_row,
			orientation,
		);
		let actual = vec![(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2), (3, 3)];
		assert_eq!(actual, best);
	}
}
