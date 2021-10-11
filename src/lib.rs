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
//!
pub mod helpers;
pub mod astar_offset;
pub mod astar_cubic;
pub mod astar_axial;

/// Specifies the orientation of the hexagon space in Offset layots. This is
/// important for determining the available neighbouring nodes during expansion.
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
