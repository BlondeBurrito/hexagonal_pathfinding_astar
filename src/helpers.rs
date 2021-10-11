//! A collection of functions to convert between coordinates systems, finding neighboring
//! hexagons, finding distance between hexagons and others

use crate::HexOrientation;

/// Converts Offset coordinates (based on an orinetation) to Cubic coordinates
pub fn offset_to_cubic(node_coords: (i32, i32), orientation: &HexOrientation) -> (i32, i32, i32) {
	match orientation {
		HexOrientation::FlatTopOddUp => {
			let x: i32 = node_coords.0;
			let z: i32 = node_coords.1 - (node_coords.0 - (node_coords.0 & 1)) / 2;
			let y: i32 = -x - z;
			return (x, y, z);
		}
		HexOrientation::FlatTopOddDown => {
			let x: i32 = node_coords.0;
			let z: i32 = node_coords.1 - (node_coords.0 + (node_coords.0 & 1)) / 2;
			let y: i32 = -x - z;
			return (x, y, z);
		}
	}
}
/// Convert a node with Axial coordinates to Cubic coordinates. `node_coords` is of the form
/// `(q, r)` where `q` is the column and `r` the row
pub fn axial_to_cubic(node_coords: (i32, i32)) -> (i32, i32, i32) {
	let x = node_coords.0;
	let z = node_coords.1;
	let y =  -z-x;
	return (x, y, z)
}
/// Convert a node with Cubic coordinates to Axial coordinates. `node_coords` is of the form
/// (x, y, z).
pub fn cubic_to_axial(node_coords: (i32, i32, i32)) -> (i32, i32) {
	let q = node_coords.0;
	let r = node_coords.2;
	return (q, r)
}
/// Finds the neighboring nodes in an Offset coordinate system. It must be in a grid-like formatiom
///  where `max_column` and `max_row` inputs define the outer boundary of the grid space, note they
/// are exclusive values. This means that for most source hexagons 6 neighbours will be expanded but
/// for those lining the boundaries fewer neighrbors will be discovered. Consider:
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
				return neighbours;
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
				return neighbours;
			}
		}
	}
}
/// Finds the neighboring nodes in an Cubic coordinate system. `source` is of the form
/// `(x, y, z). The nodes grid is bound by `min` and `max` `x`, `y` and `z` values to
/// denote the edges of the node space and they are exclusive
pub fn node_neighbours_cubic(source: (i32, i32, i32),
	min_x: i32,
	max_x: i32,
	min_y: i32,
	max_y: i32,
	min_z: i32,
	max_z: i32,
) -> Vec<(i32, i32, i32)> {
	let mut neighbours = Vec::new();
	// north (x, y + 1, z - 1)
	if source.1 + 1 < max_y && source.2 -1 > min_z {
		neighbours.push((source.0, source.1 + 1, source.2 -1))
	}
	// north-east (x + 1, y, z - 1)
	if source.0 + 1 < max_x && source.2 -1 > min_z {
		neighbours.push((source.0 + 1, source.1, source.2 -1))
	}
	// south-east (x + 1, y - 1, z)
	if source.0 + 1 < max_x && source.1 -1 > min_y{
		neighbours.push((source.0 + 1, source.1 -1, source.2))
	}
	// south (x, y - 1, z + 1)
	if source.1 - 1 > min_y && source.2 + 1 < max_z {
		neighbours.push((source.0, source.1 - 1, source.2 + 1))
	}
	// south-west (x - 1, y, z + 1)
	if source.0 - 1 > min_x && source.2 + 1 < max_z {
		neighbours.push((source.0 -1, source.1, source.2 + 1))
	}
	// north-west (x - 1, y + 1, z)
	if source.0 -1 > min_x && source.1 + 1 < max_y {
		neighbours.push((source.0 -1, source.1 + 1, source.2))
	}
	return neighbours

}
/// Finds the neighboring nodes in an Axial coordinate system. `source` is of the form
/// `(q, r)` where `q` is the column and `r` the row. The nodes grid is in a circular arrangment
/// around some origin, the `count_rings_from_origin` is exclusive and is used to determine if a neighbour
/// lives outside the searchable grid and is thus ignored
pub fn node_neighbours_axial(
	source: (i32, i32),
	count_rings_from_origin: i32,
) -> Vec<(i32, i32)> {
	let mut neighbours = Vec::new();
	// north
	if (source.0 + source.1).abs() < count_rings_from_origin {
		neighbours.push((source.0, source.1 - 1))
	}
	// north-east
	if source.0 + 1 < max_column && source.1 -1 > min_row {
		neighbours.push((source.0 + 1, source.1 - 1))
	}
	// south-east
	if source.0 + 1 < max_column {
		neighbours.push((source.0 + 1, source.1))
	}
	// south
	if source.1 + 1 < max_row {
		neighbours.push((source.0, source.1 + 1))
	}
	// south-west
	if source.0 - 1 > min_column && source.1 + 1 < max_row {
		neighbours.push((source.0 - 1, source.1 + 1))
	}
	// north-west
	if source.0 - 1 > min_column {
		neighbours.push((source.0 -1, source.1))
	}
	return neighbours
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_offset(source, &orientation, min_column, max_column, min_row, max_row);
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
		let neighbours = node_neighbours_axial(source, -3, 5, -3, 5);
		let actual = vec![(2, -2), (3, -2), (3, -1), (2, 0), (1, 0), (1, -1)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// finds a nodes neighbours in axial space
	fn axial_neighbours_with_boundary() {
		let source: (i32, i32) = (-1, -1);
		let neighbours = node_neighbours_axial(source, -3, 5, -3, 5);
		let actual = vec![(0, -2), (0, -1), (-1, 0), (-2, 0)];
		assert_eq!(actual, neighbours);
	}
	#[test]
	/// finds a nodes neighbours in cubic space
	fn cubic_neighbours() {
		let source: (i32, i32, i32) = (2, -1, -1);
		let neighbours = node_neighbours_cubic(source, 0, 4, -3, 1, -3, 1);
		let actual = vec![(2, 0, -2), (3, -1, -2), (3, -2, -1), (2, -2, 0), (1, -1, 0), (1, 0, -1)];
		assert_eq!(actual, neighbours);
	}
}