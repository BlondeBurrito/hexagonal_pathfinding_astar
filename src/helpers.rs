//! A collection of functions to convert between coordinates systems, finding neighboring
//! hexagons, finding distance between hexagons and others
//!
//! ## Axial Coordinates <a name="axial"></a>
//!
//! For the given Axial grid:
//!
//! ```txt
//!              _______
//!             /   0   \
//!     _______/         \_______
//!    /  -1   \      -1 /   1   \
//!   /         \_______/         \
//!   \       0 /   q   \      -1 /
//!    \_______/         \_______/
//!    /  -1   \       r /   1   \
//!   /         \_______/         \
//!   \       1 /   0   \       0 /
//!    \_______/         \_______/
//!            \       1 /
//!             \_______/
//! ```
//!
//! Finding a nodes neighbours in this alignment is rather simple, for a given node at `(q, r)` beginnning `north` and moving clockwise:
//!
//! ```txt
//! north      = (q, r - 1)
//! north-east = (q + 1, r - 1)
//! south-east = (q + 1, r)
//! south      = (q, r + 1)
//! south-west = (q - 1, r + 1)
//! north-west = (q - 1, r)
//! ```
//!
//! Programmatically these can be found with the helper function `node_neighbours_axial()`.
//!
//! ## Cubic Coordinates <a name="cubic"></a>
//!
//! A Cubic grid is structured such:
//!
//! ```txt
//!              _______
//!             /   0   \
//!     _______/         \_______
//!    /  -1   \ 1    -1 /   1   \
//!   /         \_______/         \
//!   \ 1     0 /   x   \ 0    -1 /
//!    \_______/         \_______/
//!    /  -1   \ y     z /   1   \
//!   /         \_______/         \
//!   \ 0     1 /   0   \ -1    0 /
//!    \_______/         \_______/
//!            \ -1    1 /
//!             \_______/
//! ```
//!
//!To find a nodes neighbours from `(x, y, z)` starting `north` and moving clockwise:
//!
//! ```txt
//! north      = (x, y + 1, z - 1)
//! north-east = (x + 1, y, z - 1)
//! south-east = (x + 1, y - 1, z)
//! south      = (x, y - 1, z + 1)
//! south-west = (x - 1, y, z + 1)
//! north-west = (x - 1, y + 1, z)
//! ```
//!
//! Programmatically these can be found with the public helper function `node_neighbours_cubic()`.
//!
//! ## Offset Coordinates
//!
//! Offset assumes that all hexagons have been plotted across a plane where the origin points sits
//! at the bottom left (in theory you can have negative coordinates expanding into the other 3
//! quadrants but I haven't tested these here).
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
//! (0,0) and wish to discover the node to its North-East, (1,0), we can simply increment the
//! `column` value by one.
//!
//! However if we take the node (1,0) and wish to discover its North-East node at (2,1) we have
//! to increment both the `column` value and the `row` value. I.e the calculation changes depending
//! on whether the odd column has been shifted up or down.
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
//! Programmatically these can be found with the public helper function `node_neighbours_offset()`
//! where the grid has boundaries in space denoted by the min and max values and`orientation` must
//! be `HexOrientation::FlatTopOddUp`
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
//! The column shift changes how we discover nearby nodes. For instance if we take the node at (0,0)
//! and wish to discover the node to its North-East, (1,1), we increment the `column` and `row`
//! values by one.
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
//!And for a node in an odd column the node neighbours can be found:
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
//! Programmatically these can be found with the public helper function `node_neighbours_offset()`
//! where the grid has boundaries in space denoted by the min and max values and`orientation` must
//! be `HexOrientation::FlatTopOddDown`

use crate::HexOrientation;

/// Converts Offset coordinates (based on an orientation) to Cubic coordinates
pub fn offset_to_cubic(node_coords: (i32, i32), orientation: &HexOrientation) -> (i32, i32, i32) {
	match orientation {
		HexOrientation::FlatTopOddUp => {
			let x: i32 = node_coords.0;
			let z: i32 = node_coords.1 - (node_coords.0 + (node_coords.0 & 1)) / 2;
			let y: i32 = -x - z;
			(x, y, z)
		}
		HexOrientation::FlatTopOddDown => {
			let x: i32 = node_coords.0;
			let z: i32 = node_coords.1 - (node_coords.0 - (node_coords.0 & 1)) / 2;
			let y: i32 = -x - z;
			(x, y, z)
		}
	}
}
/// Convert a node with Axial coordinates to Cubic coordinates. `node_coords` is of the form
/// `(q, r)` where `q` is the column and `r` the row
pub fn axial_to_cubic(node_coords: (i32, i32)) -> (i32, i32, i32) {
	let x = node_coords.0;
	let z = node_coords.1;
	let y = -z - x;
	(x, y, z)
}
/// Convert a node with Axial coordinates to Offset coordinates based on an orientation. `node_coords` is of the form
/// `(q, r)` where `q` is the column and `r` the row
pub fn axial_to_offset(node_coords: (i32, i32), orientation: &HexOrientation) -> (i32, i32) {
	match orientation {
		HexOrientation::FlatTopOddUp => {
			let x: i32 = node_coords.0;
			let y: i32 = node_coords.1 + (node_coords.0 + (node_coords.0 & 1)) / 2;
			(x, y)
		}
		HexOrientation::FlatTopOddDown => {
			let x: i32 = node_coords.0;
			let y: i32 = node_coords.1 + (node_coords.0 - (node_coords.0 & 1)) / 2;
			(x, y)
		}
	}
}
/// Convert a node with Cubic coordinates to Axial coordinates. `node_coords` is of the form
/// `(x, y, z)`.
pub fn cubic_to_axial(node_coords: (i32, i32, i32)) -> (i32, i32) {
	let q = node_coords.0;
	let r = node_coords.2;
	(q, r)
}
/// Convert a node with Cubic coordinates to Offset coordinates based on an orientation. `node_coords` is of the form
/// `(x, y, z)`.
pub fn cubic_to_offset(node_coords: (i32, i32, i32), orientation: &HexOrientation) -> (i32, i32) {
	match orientation {
		HexOrientation::FlatTopOddUp => {
			let x: i32 = node_coords.0;
			let y: i32 = node_coords.2 + (node_coords.0 + (node_coords.0 & 1)) / 2;
			(x, y)
		}
		HexOrientation::FlatTopOddDown => {
			let x: i32 = node_coords.0;
			let y: i32 = node_coords.2 + (node_coords.0 - (node_coords.0 & 1)) / 2;
			(x, y)
		}
	}
}
/// Finds the neighboring nodes in an Offset coordinate system. It must be in a grid-like formatiom
///  where 'min_column`,`max_column` `min_row` and `max_row` inputs define the outer boundary of the grid space, note they
/// are exclusive values. This means that for most source hexagons 6 neighbours will be expanded but
/// for those lining the boundaries fewer neighrbors will be discovered.
///
/// Consider:
/// ```txt
///       ___
///   ___/   \___
///  /   \___/   \
///  \___/   \___/
///  /   \___/   \
///  \___/   \___/
/// ```
/// Expanding the bottom left node will only discover two neighbours
pub fn node_neighbours_offset(
	source: (i32, i32),
	orientation: &HexOrientation,
	min_column: i32,
	max_column: i32,
	min_row: i32,
	max_row: i32,
) -> Vec<(i32, i32)> {
	let mut neighbours = Vec::new();
	// starting from north round a tile clockwise
	match orientation {
		//       ___
		//   ___/   \
		//  /   \___/
		//  \___/
		// flat topped arrangemnt of hexagns, odd columns shifted up
		HexOrientation::FlatTopOddUp => {
			// even column
			if source.0 & 1 == 0 {
				// north
				if source.1 + 1 < max_row {
					neighbours.push((source.0, source.1 + 1));
				};
				// north-east
				if source.0 + 1 < max_column {
					neighbours.push((source.0 + 1, source.1));
				};
				// south-east
				if source.0 + 1 < max_column && source.1 - 1 > min_row {
					neighbours.push((source.0 + 1, source.1 - 1));
				};
				// south
				if source.1 - 1 > min_row {
					neighbours.push((source.0, source.1 - 1));
				};
				// south-west
				if source.0 - 1 > min_column && source.1 - 1 > min_row {
					neighbours.push((source.0 - 1, source.1 - 1));
				}
				// north-west
				if source.0 - 1 > min_column {
					neighbours.push((source.0 - 1, source.1));
				}
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
				if source.1 - 1 > min_row {
					neighbours.push((source.0, source.1 - 1));
				}
				// south-west
				if source.0 - 1 < max_column {
					neighbours.push((source.0 - 1, source.1));
				}
				// north-east
				if source.0 - 1 < max_column && source.1 + 1 < max_row {
					neighbours.push((source.0 - 1, source.1 + 1))
				}
			}
		}
		//   ___
		//  /   \___
		//  \___/   \
		//      \___/
		// flat topped arrangemnt of hexagns, odd columns shifted down
		HexOrientation::FlatTopOddDown => {
			// even column with BitwiseAND
			if source.0 & 1 == 0 {
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
				if source.1 - 1 > min_row {
					neighbours.push((source.0, source.1 - 1));
				}
				// south-west
				if source.0 - 1 > min_column {
					neighbours.push((source.0 - 1, source.1));
				}
				// north-west
				if source.0 - 1 > min_column && source.1 + 1 < max_row {
					neighbours.push((source.0 - 1, source.1 + 1));
				}
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
				if source.0 + 1 < max_column && source.1 - 1 > min_row {
					neighbours.push((source.0 + 1, source.1 - 1))
				}
				// south
				if source.1 - 1 > min_row {
					neighbours.push((source.0, source.1 - 1))
				}
				// south-west
				if source.0 - 1 > min_column && source.1 - 1 > min_row {
					neighbours.push((source.0 - 1, source.1 - 1))
				}
				// north-west
				if source.0 - 1 > min_column {
					neighbours.push((source.0 - 1, source.1))
				}
			}
		}
	}
	neighbours
}
/// Finds the neighboring nodes in a Cubic coordinate system. `source` is of the form
/// `(x, y, z)` and denotes the node from which neighbours are discovered. The node grid is in a
/// circular arrangement with `count_rings_from_origin` being the number of rings around the origin
/// of the entire grid, this prevents returning theoretical nodes which exist outside of the grid - the value is inclusive.
///
/// For instance if the entire grid is:
/// ```txt
///              _______
///             /   0   \
///     _______/         \_______
///    /  -1   \ 1    -1 /   1   \
///   /         \_______/         \
///   \ 1     0 /   x   \ 0    -1 /
///    \_______/         \_______/
///    /  -1   \ y     z /   1   \
///   /         \_______/         \
///   \ 0     1 /   0   \ -1    0 /
///    \_______/         \_______/
///            \ -1    1 /
///             \_______/
/// ```
/// Will have a `count_rings_from_origin` of 1 preventing returning non existent nodes.
#[allow(clippy::int_plus_one)]
pub fn node_neighbours_cubic(
	source: (i32, i32, i32),
	count_rings_from_origin: i32,
) -> Vec<(i32, i32, i32)> {
	let mut neighbours = Vec::new();
	// north (x, y + 1, z - 1)
	if (source.1 + 1).abs() <= count_rings_from_origin
		&& (source.2 - 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0, source.1 + 1, source.2 - 1))
	}
	// north-east (x + 1, y, z - 1)
	if (source.0 + 1).abs() <= count_rings_from_origin
		&& (source.2 - 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 + 1, source.1, source.2 - 1))
	}
	// south-east (x + 1, y - 1, z)
	if (source.0 + 1).abs() <= count_rings_from_origin
		&& (source.1 - 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 + 1, source.1 - 1, source.2))
	}
	// south (x, y - 1, z + 1)
	if (source.1 - 1).abs() <= count_rings_from_origin
		&& (source.2 + 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0, source.1 - 1, source.2 + 1))
	}
	// south-west (x - 1, y, z + 1)
	if (source.0 - 1).abs() <= count_rings_from_origin
		&& (source.2 + 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 - 1, source.1, source.2 + 1))
	}
	// north-west (x - 1, y + 1, z)
	if (source.0 - 1).abs() <= count_rings_from_origin
		&& (source.1 + 1).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 - 1, source.1 + 1, source.2))
	}
	neighbours
}
/// Finds the neighboring nodes in an Axial coordinate system. `source` is of the form
/// `(q, r)` where `q` is the column and `r` the row. The node grid is in a circular arrangment
/// around some origin, the `count_rings_from_origin` is inclusive and is used to determine if a neighbour
/// lives outside the searchable grid and is thus ignored.
///
/// For instance:
/// ```txt
///              _______
///             /   0   \
///     _______/         \_______
///    /  -1   \      -1 /   1   \
///   /         \_______/         \
///   \       0 /   q   \      -1 /
///    \_______/         \_______/
///    /  -1   \       r /   1   \
///   /         \_______/         \
///   \       1 /   0   \       0 /
///    \_______/         \_______/
///            \       1 /
///             \_______/
/// ```
/// Only has 1 ring of nodes around the origin so the count is 1.
pub fn node_neighbours_axial(source: (i32, i32), count_rings_from_origin: i32) -> Vec<(i32, i32)> {
	let mut neighbours = Vec::new();
	// finding neighbours is a billion times easier in cubic coords
	let cubic = axial_to_cubic(source);
	let n = node_neighbours_cubic(cubic, count_rings_from_origin);
	for i in n.iter() {
		neighbours.push(cubic_to_axial(*i))
	}
	neighbours
}
/// Finds the nodes on a ring around a given source point in a Cubic coordinate system. `source` is of the form
/// `(x, y, z)`. `radius` is the particular ring you want to know the nodes of.
///
/// For instance `radius = 2` for the second ring around the origin:
/// ```txt
///                     _______
///                    /       \
///            _______/  RING2  \_______
///           /       \         /       \
///   _______/  RING2  \_______/  RING2  \_______
///  /       \         /       \         /       \
/// /  RING2  \_______/         \_______/  RING2  \
/// \         /       \         /       \         /
///  \_______/         \_______/         \_______/
///  /       \         /   x   \         /       \
/// /  RING2  \_______/         \_______/  RING2  \
/// \         /       \ y     z /       \         /
///  \_______/         \_______/         \_______/
///  /       \         /       \         /       \
/// /  RING2  \_______/         \_______/  RING2  \
/// \         /       \         /       \         /
///  \_______/  RING2  \_______/  RING2  \_______/
///          \         /       \         /
///           \_______/  RING2  \_______/
///                   \         /
///                    \_______/
/// ```
pub fn node_ring_cubic(source: (i32, i32, i32), radius: i32) -> Vec<(i32, i32, i32)> {
	let mut ring_nodes = Vec::new();
	// unit lengths to move in a direction of a face, the array starts with the North direction
	// moving clockwise for each edge
	//         N
	//      _______
	//     /       \
	// NW /         \ NE
	// SW \         / SE
	//     \_______/
	//         S
	let cube_directions = [
		(0, 1, -1),
		(1, 0, -1),
		(1, -1, 0),
		(0, -1, 1),
		(-1, 0, 1),
		(-1, 1, 0),
	];
	// from the starting node move to the node joining the south-west and west faces, e.g for radius =2:
	//                            _________
	//                           /         \
	//                          /           \
	//                _________/             \_________
	//               /         \             /         \
	//              /           \           /           \
	//    _________/             \_________/             \_________
	//   /         \             /         \             /         \
	//  /           \           /           \           /           \
	// /             \_________/             \_________/             \
	// \             /         \             /         \             /
	//  \           /           \           /           \           /
	//   \_________/             \_________/             \_________/
	//   /         \             /    x    \             /         \
	//  /           \           /           \           /           \
	// /             \_________/   SOURCE    \_________/             \
	// \             /         \ y         z /         \             /
	//  \           /           \           /           \           /
	//   \_________/             \_________/             \_________/
	//   /         \             /         \             /         \
	//  /           \           /           \           /           \
	// /    START    \_________/             \_________/             \
	// \             /         \             /         \             /
	//  \           /           \           /           \           /
	//   \_________/             \_________/             \_________/
	//             \             /         \             /
	//              \           /           \           /
	//               \_________/             \_________/
	//                         \             /
	//                          \           /
	//                           \_________/
	let scaled_x = cube_directions[4].0 * radius;
	let scaled_y = cube_directions[4].1 * radius;
	let scaled_z = cube_directions[4].2 * radius;
	let mut ring_node_current = (
		source.0 + scaled_x,
		source.1 + scaled_y,
		source.2 + scaled_z,
	);
	// from the node starting on the ring we can walk around the ring discovering all the nodes on it
	// iterate to 6 as a hexagon has 6 faces, we walk along each side of the hex ring
	for i in 0..6 {
		// the length of each face is denoted by the radius
		// e.g radius + 1, so for radius = 2 the sides have length 3 but we only take two steps at a time as to not overlap:
		//                            _________
		//                           /         \
		//                          /           \
		//                _________/             \_________
		//               /         \             /         \
		//              /           \     i=1   /           \
		//    _________/             \_________/             \_________
		//   /         \             /         \             /         \
		//  /           \     i=1   /           \     i=2   /           \
		// /             \_________/             \_________/             \
		// \             /         \             /         \             /
		//  \    i=0    /           \           /           \     i=2   /
		//   \_________/             \_________/             \_________/
		//   /         \             /    x    \             /         \
		//  /           \           /           \           /           \
		// /             \_________/             \_________/             \
		// \             /         \ y         z /         \             /
		//  \    i=0    /           \           /           \     i=3   /
		//   \_________/             \_________/             \_________/
		//   /         \             /         \             /         \
		//  /           \           /           \           /           \
		// /    START    \_________/             \_________/             \
		// \             /         \             /         \             /
		//  \    i=5    /           \           /           \     i=3   /
		//   \_________/             \_________/             \_________/
		//             \             /         \             /
		//              \     i=5   /           \     i=4   /
		//               \_________/             \_________/
		//                         \             /
		//                          \     i=4   /
		//                           \_________/
		for _j in 0..radius {
			// move to next node
			ring_node_current.0 += cube_directions[i].0;
			ring_node_current.1 += cube_directions[i].1;
			ring_node_current.2 += cube_directions[i].2;
			// store node
			ring_nodes.push(ring_node_current);
		}
	}
	ring_nodes
}

/// The distance between two nodes by using cubic coordinates
pub fn node_distance(start: (i32, i32, i32), end: (i32, i32, i32)) -> i32 {
	((start.0 - end.0).abs() + (start.1 - end.1).abs() + (start.2 - end.2).abs()) / 2
}

mod tests {
	#[cfg(test)]
	use super::*;

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
		let source: (i32, i32) = (2, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
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
		let source: (i32, i32) = (3, 2);
		let orientation = HexOrientation::FlatTopOddUp;
		let min_column = -1;
		let max_column = 5;
		let min_row = -1;
		let max_row = 5;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
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
		let source: (i32, i32) = (2, 2);
		let orientation = HexOrientation::FlatTopOddDown;
		let min_column = -1;
		let max_column = 4;
		let min_row = -1;
		let max_row = 4;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
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
		let source: (i32, i32) = (3, 2);
		let orientation = HexOrientation::FlatTopOddDown;
		let min_column = -1;
		let max_column = 5;
		let min_row = -1;
		let max_row = 5;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
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
	fn offset_neighbors_along_negative_boundary() {
		let source: (i32, i32) = (0, 0);
		let orientation = HexOrientation::FlatTopOddUp;
		let min_column = -1;
		let max_column = 5;
		let min_row = -1;
		let max_row = 5;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
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
	fn offset_neighbors_along_positive_boundary() {
		let source: (i32, i32) = (2, 1);
		let orientation = HexOrientation::FlatTopOddUp;
		let min_column = -1;
		let max_column = 3;
		let min_row = -1;
		let max_row = 2;
		let neighbours = node_neighbours_offset(
			source,
			&orientation,
			min_column,
			max_column,
			min_row,
			max_row,
		);
		let expected_neighbour_count = 3;
		assert_eq!(expected_neighbour_count, neighbours.len());
	}
	#[test]
	/// convert axial coordinates to cubic
	fn axial_to_cubic_cords() {
		let axial: (i32, i32) = (2, 1);
		let cubic = axial_to_cubic(axial);
		assert_eq!((2, -3, 1), cubic);
	}
	#[test]
	/// convert cubic coordinates to axial
	fn cubic_to_axial_cords() {
		let cubic: (i32, i32, i32) = (1, -2, 1);
		let axial = cubic_to_axial(cubic);
		assert_eq!((1, 1), axial);
	}
	#[test]
	/// finds a nodes neighbours in axial space
	fn axial_neighbours() {
		let source: (i32, i32) = (2, -1);
		let neighbours = node_neighbours_axial(source, 3);
		let actual = vec![(2, -2), (3, -2), (3, -1), (2, 0), (1, 0), (1, -1)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// finds a nodes neighbours in axial space against a boundary
	fn axial_neighbours_with_boundary() {
		let source: (i32, i32) = (-1, -1);
		let neighbours = node_neighbours_axial(source, 2);
		let actual = vec![(0, -2), (0, -1), (-1, 0), (-2, 0)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// finds a nodes neighbours in cubic space
	fn cubic_neighbours() {
		let source: (i32, i32, i32) = (2, -1, -1);
		let neighbours = node_neighbours_cubic(source, 3);
		let actual = vec![
			(2, 0, -2),
			(3, -1, -2),
			(3, -2, -1),
			(2, -2, 0),
			(1, -1, 0),
			(1, 0, -1),
		];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// finds a nodes neighbours in cubic space where it is up against a boundary
	fn cubic_neighbours_with_boundary() {
		let source: (i32, i32, i32) = (2, -1, -1);
		let neighbours = node_neighbours_cubic(source, 2);
		let actual = vec![(2, 0, -2), (2, -2, 0), (1, -1, 0), (1, 0, -1)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// convert axial coords to offset in a FlatTopOddUp grid orienation
	fn convert_axial_to_offset_odd_up() {
		let source: (i32, i32) = (-1, -1);
		let result = axial_to_offset(source, &HexOrientation::FlatTopOddUp);
		let actual: (i32, i32) = (-1, -1);
		assert_eq!(actual, result);
	}
	#[test]
	/// convert axial coords to offset in a FlatTopOddDown grid orienation
	fn convert_axial_to_offset_odd_down() {
		let source: (i32, i32) = (-1, -1);
		let result = axial_to_offset(source, &HexOrientation::FlatTopOddDown);
		let actual: (i32, i32) = (-1, -2);
		assert_eq!(actual, result);
	}
	#[test]
	/// convert cubic coords to offset in a FlatTopOddUp grid orientation
	fn convert_cubic_to_offset_odd_up() {
		let source: (i32, i32, i32) = (-2, 3, -1);
		let result = cubic_to_offset(source, &HexOrientation::FlatTopOddUp);
		let actual: (i32, i32) = (-2, -2);
		assert_eq!(actual, result);
	}
	#[test]
	/// convert cubic coords to offset in a FlatTopOddDown grid orientation
	fn convert_cubic_to_offset_odd_downp() {
		let source: (i32, i32, i32) = (-1, 1, 0);
		let result = cubic_to_offset(source, &HexOrientation::FlatTopOddDown);
		let actual: (i32, i32) = (-1, -1);
		assert_eq!(actual, result);
	}
	#[test]
	/// test for nodes on ring 1
	fn ring_1() {
		let source = (0, 0, 0);
		let radius = 1;
		let result = node_ring_cubic(source, radius);
		let actual = vec![
			(-1, 1, 0),
			(0, 1, -1),
			(1, 0, -1),
			(1, -1, 0),
			(0, -1, 1),
			(-1, 0, 1),
		];
		assert_eq!(actual, result);
	}
	#[test]
	/// test for nodes on ring 2
	fn ring_2() {
		let source = (0, 0, 0);
		let radius = 2;
		let result = node_ring_cubic(source, radius);
		let actual = vec![
			(-2, 1, 1),
			(-2, 2, 0),
			(-1, 2, -1),
			(0, 2, -2),
			(1, 1, -2),
			(2, 0, -2),
			(2, -1, -1),
			(2, -2, 0),
			(1, -2, 1),
			(0, -2, 2),
			(-1, -1, 2),
			(-2, 0, 2),
		];
		assert_eq!(actual, result);
	}
	#[test]
	/// test for nodes on ring 3
	fn ring_3() {
		let source = (0, 0, 0);
		let radius = 3;
		let result = node_ring_cubic(source, radius);
		let actual = vec![
			(-3, 1, 2),
			(-3, 2, 1),
			(-3, 3, 0),
			(-2, 3, -1),
			(-1, 3, -2),
			(0, 3, -3),
			(1, 2, -3),
			(2, 1, -3),
			(3, 0, -3),
			(3, -1, -2),
			(3, -2, -1),
			(3, -3, 0),
			(2, -3, 1),
			(1, -3, 2),
			(0, -3, 3),
			(-1, -2, 3),
			(-2, -1, 3),
			(-3, 0, 3),
		];
		assert_eq!(actual, result);
	}
	#[test]
	/// Validate offset to cubic conversion in flat topped odd up orientation
	fn convert_offset_to_cubic_flat_top_odd_up() {
		let source: (i32, i32) = (9, 9);
		let result = offset_to_cubic(source, &HexOrientation::FlatTopOddUp);
		let actual: (i32, i32, i32) = (9, -13, 4);
		assert_eq!(actual, result);
	}
	#[test]
	/// Validate offset to cubic conversion in flat topped odd down orientation
	fn convert_offset_to_cubic_flat_top_odd_down() {
		let source: (i32, i32) = (9, 9);
		let result = offset_to_cubic(source, &HexOrientation::FlatTopOddDown);
		let actual: (i32, i32, i32) = (9, -14, 5);
		assert_eq!(actual, result);
	}
}
