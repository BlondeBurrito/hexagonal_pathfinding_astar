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
			let z: i32 = node_coords.1 - (node_coords.0 - (node_coords.0 & 1)) / 2;
			let y: i32 = -x - z;
			(x, y, z)
		}
		HexOrientation::FlatTopOddDown => {
			let x: i32 = node_coords.0;
			let z: i32 = node_coords.1 - (node_coords.0 + (node_coords.0 & 1)) / 2;
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
/// Convert a node with Cubic coordinates to Axial coordinates. `node_coords` is of the form
/// `(x, y, z)`.
pub fn cubic_to_axial(node_coords: (i32, i32, i32)) -> (i32, i32) {
	let q = node_coords.0;
	let r = node_coords.2;
	(q, r)
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
	// the neighbours are on the first ring hence we hardcode the ring_number
	node_ring_cubic(source, 1, count_rings_from_origin)
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
/// `(x, y, z)`. The node grid is in a circular arrangement with `count_rings_from_origin` being
/// the number of rings around the origin of the entire grid, this prevents returning nodes which exist outside of the
/// grid - the value is inclusive. 'ring_number' is the particular ring you want to know the nodes of
///
/// For instance:
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
/// With a `ring_number = 1` will find the immediate neighbouring nodes around the centre
#[allow(clippy::int_plus_one)]
pub fn node_ring_cubic(
	source: (i32, i32, i32),
	ring_number: i32,
	count_rings_from_origin: i32,
) -> Vec<(i32, i32, i32)> {
	let mut neighbours = Vec::new();
	// north (x, y + ring_number, z - ring_number)
	if (source.1 + ring_number).abs() <= count_rings_from_origin
		&& (source.2 - ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0, source.1 + ring_number, source.2 - ring_number))
	}
	// north-east (x + ring_number, y, z - ring_number)
	if (source.0 + ring_number).abs() <= count_rings_from_origin
		&& (source.2 - ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 + ring_number, source.1, source.2 - ring_number))
	}
	// south-east (x + ring_number, y - ring_number, z)
	if (source.0 + ring_number).abs() <= count_rings_from_origin
		&& (source.1 - ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 + ring_number, source.1 - ring_number, source.2))
	}
	// south (x, y - ring_number, z + ring_number)
	if (source.1 - ring_number).abs() <= count_rings_from_origin
		&& (source.2 + ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0, source.1 - ring_number, source.2 + ring_number))
	}
	// south-west (x - ring_number, y, z + ring_number)
	if (source.0 - ring_number).abs() <= count_rings_from_origin
		&& (source.2 + ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 - ring_number, source.1, source.2 + ring_number))
	}
	// north-west (x - ring_number, y + ring_number, z)
	if (source.0 - ring_number).abs() <= count_rings_from_origin
		&& (source.1 + ring_number).abs() <= count_rings_from_origin
	{
		neighbours.push((source.0 - ring_number, source.1 + ring_number, source.2))
	}
	neighbours
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
}
