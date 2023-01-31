//! A-Star pathfinding algorithm for a Spiral Hex grid alignment.
//!
//! Spiral Hex coordinates use the convention of a single digit `x` to label a node.
//! For hexagon layouts where the pointy tops are facing up the calculations remain exactly the same as you're effectively
//! just rotating the grid by 30 degrees.
//!
//! ```txt
//!              _______
//!             /       \
//!     _______/    1    \_______
//!    /       \         /       \
//!   /    6    \_______/    2    \
//!   \         /       \         /
//!    \_______/    x    \_______/
//!    /       \         /       \
//!   /    5    \_______/    3    \
//!   \         /       \         /
//!    \_______/    4    \_______/
//!            \         /
//!             \_______/
//! ```
//!

use crate::astar_cubic;
use crate::helpers::cubic_to_spiral_hex;
use crate::helpers::spiral_hex_to_cubic;
use ::std::collections::HashMap;
use core::panic;

/// From a starting node calculate the most efficient path to the end node
///
/// The `nodes` input is structured such:
///
/// * The keys are an `i32` of the node postion
/// * The layout builds a circular-like grid
/// * The values are the complexity of traversing a particular node
///
/// `count_rings` is the number of rings around the origin `0` of the circular hexagonal grid. It
/// is an inclusive value. NB: `count_rings` is NOT the nubmer of rings around the start or end node, it
/// is explicitly the number of rings around the origin of our hexagon grid
///
/// For instance from our origin of `0`:
///
///```txt
///                              _________
///                             /         \
///                            /           \
///                  _________/    Ring2    \_________
///                 /         \      7      /         \
///                /           \           /           \
///      _________/    Ring2    \_________/    Ring2    \_________
///     /         \      18     /         \      8      /         \
///    /           \           /           \           /           \
///   /    Ring2    \_________/    Ring1    \_________/    Ring2    \
///   \      17     /         \      1      /         \      9      /
///    \           /           \           /           \           /
///     \_________/    Ring1    \_________/    Ring1    \_________/
///     /         \      6      /         \      2      /         \
///    /           \           /           \           /           \
///   /    Ring2    \_________/             \_________/    Ring2    \
///   \     16      /         \      0      /         \      10     /
///    \           /           \           /           \           /
///     \_________/    Ring1    \_________/    Ring1    \_________/
///     /         \      5      /         \      3      /         \
///    /           \           /           \           /           \
///   /    Ring2    \_________/    Ring1    \_________/    Ring2    \
///   \      15     /         \      4      /         \      11     /
///    \           /           \           /           \           /
///     \_________/    Ring2    \_________/    Ring2    \_________/
///               \      14     /         \      12     /
///                \           /           \           /
///                 \_________/    Ring2    \_________/
///                           \      13     /
///                            \           /
///                             \_________/
///  ```
///
/// We have 2 rings of hexagons surrounding it.
///
/// The return Vec contains a number of tuples which for `0..n` show the best path to take
pub fn astar_path(
	start_node: i32,
	nodes: HashMap<i32, f32>,
	end_node: i32,
	count_rings: i32,
) -> Vec<i32> {
	// ensure nodes data contains start and end points
	if !nodes.contains_key(&start_node) {
		panic!("Node data does not contain start node {}", start_node);
	}
	if !nodes.contains_key(&end_node) {
		panic!("Node data does not contain end node {}", end_node);
	}
	// ensure start and end nodes are within the max bounds of the grid
	// we use the ring boundary hence it's easier to check this in cubic coords
	let cubic_start = spiral_hex_to_cubic(start_node);
	let cubic_end = spiral_hex_to_cubic(end_node);
	if cubic_start.0.abs() > count_rings
		|| cubic_start.1.abs() > count_rings
		|| cubic_start.2.abs() > count_rings
	{
		panic!("Start node is outside of searchable grid")
	}
	if cubic_end.0.abs() > count_rings
		|| cubic_end.1.abs() > count_rings
		|| cubic_end.2.abs() > count_rings
	{
		panic!("End node is outside of searchable grid")
	}
	let mut cubic_nodes: HashMap<(i32, i32, i32), f32> = HashMap::new();
	for (coord, complexity) in nodes {
		cubic_nodes.insert(spiral_hex_to_cubic(coord), complexity);
	}
	let best_path_cubic = astar_cubic::astar_path(cubic_start, cubic_nodes, cubic_end, count_rings);
	// convert back to spiral hex
	let mut best_path_spiral_hex = Vec::new();
	for i in best_path_cubic {
		best_path_spiral_hex.push(cubic_to_spiral_hex(i));
	}
	best_path_spiral_hex
}

#[cfg(test)]
mod tests {
	use crate::astar_spiral_hex::astar_path;
	use std::collections::HashMap;

	#[test]
	/// Calcualtes the best path from S to E
	///```txt
	///                              _________
	///                             /         \
	///                            /           \
	///                  _________/     C:1     \_________
	///                 /         \      7      /         \
	///                /           \           /           \
	///      _________/     C:2     \_________/     C:14    \_________
	///     /         \     18      /         \      8      /         \
	///    /           \           /           \           /     E     \
	///   /     C:1     \_________/     C:1     \_________/     C:1     \
	///   \     17      /         \      1      /         \      9      /
	///    \           /           \           /           \           /
	///     \_________/     C:7     \_________/     C:15    \_________/
	///     /         \      6      /         \      2      /         \
	///    /           \           /     S     \           /           \
	///   /     C:8     \_________/     C:1     \_________/     C:1     \
	///   \     16      /         \      0      /         \     10      /
	///    \           /           \           /           \           /
	///     \_________/     C:6     \_________/     C:14    \_________/
	///     /         \      5      /         \      3      /         \
	///    /           \           /           \           /           \
	///   /     C:1     \_________/     C:2     \_________/     C:1     \
	///   \     15      /         \      4      /         \     11      /
	///    \           /           \           /           \           /
	///     \_________/     C:3     \_________/     C:1     \_________/
	///               \     14      /         \     12      /
	///                \           /           \           /
	///                 \_________/     C:1     \_________/
	///                           \     13      /
	///                            \           /
	///                             \_________/
	///  ```
	fn astar_tick() {
		let start_node: i32 = 0;
		let mut nodes: HashMap<i32, f32> = HashMap::new();
		nodes.insert(0, 1.0);
		nodes.insert(1, 1.0);
		nodes.insert(2, 15.0);
		nodes.insert(3, 14.0);
		nodes.insert(4, 2.0);
		nodes.insert(5, 6.0);
		nodes.insert(6, 7.0);
		nodes.insert(7, 1.0);
		nodes.insert(8, 14.0);
		nodes.insert(9, 1.0);
		nodes.insert(10, 1.0);
		nodes.insert(11, 1.0);
		nodes.insert(12, 1.0);
		nodes.insert(13, 1.0);
		nodes.insert(14, 3.0);
		nodes.insert(15, 1.0);
		nodes.insert(16, 8.0);
		nodes.insert(17, 1.0);
		nodes.insert(18, 2.0);
		let end_node: i32 = 9;
		let rings = 2;
		let best = astar_path(start_node, nodes, end_node, rings);
		let actual = vec![0, 4, 12, 11, 10, 9];
		assert_eq!(actual, best);
	}
}
