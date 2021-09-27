//! blah
//! 
//! 
//! 
//! 
//! 
//! 
//! 
//!    _______
//!   /       \
//!  /         \
//!  \         /
//!   \_______/

use ::std::collections::HashMap;

/// Specifies the orientation of the hexagon space. This is important for determining the available nodes during expansion
pub enum HexOrientation {
	FlatTopOddUp,
	FlatTopOddDown,
}

/// From a starting node calculate the most efficient path to the end node
/// 
/// The nodes input is structured such:
/// 
/// * The keys are tuples of the nodes position in a grid with the (0,0) origin being based on the bottom left
/// * The values are tuples of the form (complexity, weighting)
/// 
/// Complexity is a measure of how difficult it is to traverse a hexagon from one side to the other.
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
/// For a grid of perfectly flush hexagons the distance from the center to the midpoint of an edge is the same in all directions. This library is akin to idea that you wake up in a 'hexagon world' and you can only move from the center of one hexagon to another in a straight line, but while distance is static you'll find that as you cross the boundary of one hexagon into another you'll suddenly be sprinting instead of slow-motion walking.
/// 
/// The return Vec contains a number of tuples where the first element is the coordinates of a node and
/// the second element is the weighting of that particular node
pub fn astar_path(
	start_node: (usize, usize),
	nodes: HashMap<(usize, usize), (f32, f32)>,
	end_node: (usize, usize),
	max_column: usize,
	max_row: usize,
	orientation: HexOrientation,
) -> Vec<(usize, usize)> {
	let mut best_path = Vec::new();
	// A-star algorithm (based on Dijkstra's algorithim)
	// https://www.youtube.com/watch?v=6TsL96NAZCo
	// we use the tile element tuples to create a weighted measure of the distance
	// which we can use to optimise the calculation (that makes it A-star, normal Dujkstra is more brute force-ish)
	////////
	// combine input maps to calculate absolute speed modifiers for tiles
	// we also add a weighting to each value which is the distance of a tile from the quest tile
	// hash format
	// key = tile elements tuple
	// value = (combined speed modifier, coords of tile in 2d space, element delta from tile to quest)

	// every time we process a new node we add it to a map
	// if a node already has already been recorded then we replace it if it has a better a-star score (smaller number) otherwise we discard it
	let mut node_astar_scores: HashMap<(usize, usize), f32> = HashMap::new();

	// create a queue of nodes to be processed based on discovery
	let mut queue = Vec::new();
	// add starting node to queue
	queue.push((
		start_node.clone(),
		// lookup the distance and weighting of the starting_node based on the full nodes data set
		a_star_score(nodes[&start_node].1, nodes[&start_node].0),
		// Vec::new(),
	));
	// target node will eventually be shifted to first of queue so finish processing once it arrives, meaning that we know the best path
	while queue[0].0 != end_node {
		// // sort the queue by a-star sores
		// queue.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
		// // data about the previous tiles processed to get to the current tile being processed
		// let came_from = queue[0].0.clone();
		// let came_from_history = queue[0].2.clone();
		// for n in expand_neighrbor_nodes(queue[0].0, &orientation, max_column, max_row).iter() {
		// 	// go through previous paths getting weight
		// 	let mut combined_weight = 0.0;
		// 	for s in queue[0].2.iter() {
		// 		combined_weight += nodes[s].0;
		// 	}
		// 	// add current weighting
		// 	combined_weight += nodes[n].0;
		// 	let score = a_star_score(nodes[n].2, combined_weight);
		// 	// check to see if a weighting to a tile already exists
		// 	// if it does and its score is greater replace it
		// 	let mut updated_queue = false;
		// 	for mut q in queue.iter_mut() {
		// 		if &q.0 == n {
		// 			// queue contains path to node already
		// 			// normally only change it greater, however this leads to loads of duplicate values in the vec
		// 			// by using >= we may discard a path that would later become optimal but it prevents locking
		// 			if q.1 >= score {
		// 				// existing path has higher weighting so replace it
		// 				q.1 = score;
		// 				q.2 = came_from_history.clone();
		// 				q.2.push(came_from.clone());
		// 				updated_queue = true;
		// 			}
		// 		}
		// 	}
		// 	// queue doesn't contain the tile so add it into the vec
		// 	if !updated_queue {
		// 		let mut path = came_from_history.clone();
		// 		path.push(came_from.clone());
		// 		//path.push(n.clone());
		// 		queue.push((n.clone(), score, path.to_vec()));
		// 	}
		// }
		// // handle extreme case where all paths are bad such that the tile previous the quest has a batter a-star score than the quest itself, the previous score would be ordered as best causing an infinite loop as the quest tile score cannot be better than the score of a tile before it - case happens when quest is on mountains/marsh/over a mountain range
		// for i in 0..queue.len() {
		// 	if queue[i].0 == end_node {
		// 		queue.swap(0, i);
		// 	}
		// }
		// //info!("{:?}", queue[0]);
		// //info!("{:?}", queue);
	}
	// //info!("{:?}", queue[0]);
	
	// // take the best path from first element of the queue
	// // vec is tuples of (tile_translation, speed)
	// for key in queue[0].2.iter() {
	// 	best_path.push((nodes[key].1.truncate(), nodes[key].0));
	// }
	// // end point on way to quest
	// best_path.push((
	// 	nodes[&end_node].1.truncate(),
	// 	nodes[&end_node].0,
	// ));

	// info!("{:?}", path_to_quest);
	return best_path;
}

/// Finds the neighbouring nodes based on the orientation of the hexagon grid.
/// During expansion it will check for overflow i.e expanding to a node at a negative position - this is deemed invalid
/// and considered a boundary of the hexagon grid space.
/// Likewise the `max_column` and `max_row` inputs define the outer boundary of the grid space, note they are exclusive values.
/// This means that for most source hexagons 6 neighbours will be expanded but for those lining the boundaries fewer neighrbors will be discovered
fn expand_neighrbor_nodes(source: (usize, usize), orientation: &HexOrientation, max_column: usize, max_row: usize) -> Vec<(usize, usize)> {
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
				if  source.1.checked_sub(1) != None {
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
				return neighbours
			} else { // odd column
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
				return neighbours
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
				return neighbours
			} else { // odd column
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
				return neighbours
			}
		}
	}
}
/// Determines a weighting used to guide the algorithm, low weightings are better
fn a_star_score(complexity: f32, weighting: f32) -> f32 {
	complexity + weighting
}


#[cfg(test)]
mod tests {
	use crate::HexOrientation;
	use crate::expand_neighrbor_nodes;

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
}
