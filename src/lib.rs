//! This library is an implementation of the A-Star pathfinding algorithm tailored for traversing a bespoke
//! collection of weighted hexagons. It's intended to calculate the most optimal path to a target
//! hexagon where you are traversing from the centre of one hexagon to the next along a line orthogonal to a hexagon edge.
//! The algorithm has been implemented for Offset, Axial and Cubic coordinate systems with a selection of helper functions
//! which can be used to convert between coordinate systems, calculate distances between hexagons and more.
//!
//! The calculations are dpendent on the layout of your hexagon grid (coordinate system) and each hexagon has an associated complexity of traversing a particular node.
//!
//! E.g
//! ```txt
//!    ___________
//!   /     ^     \
//!  /      |      \
//! /  C    |       \
//! \       |       /
//!  \      ▼      /
//!   \___________/
//! ```
//!
//! Which influences the calculation to find the best path.
//!
//! ## Hexagon Grids and  Orientation
//!
//! There are different ways in which a hexagon grid can be portrayed which in turn affects the discoverable neighbouring hexagons for path traversal.
//!
//! ### Axial Coordinates
//!
//! Axial coordinates use the convention of `q` for column and `r` for row. In the example below the `r` is a diagonal row. For hexagon layouts where the pointy tops are facing up the calculations remain exactly the same as you're effectively just rotating the grid by 30 degrees making `r` horizontal and `q` diagonal.
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
//! ### Cubic Coordinates
//!
//! You can represent a hexagon grid through three primary axes. We denote the axes `x`, `y` and `z`. The Cubic coordinate system is very useful as some calculations cannot be performed through other coordinate systems (they don't contain enough data), fortunately there are means of converting other systems to Cubic to make calculations easy/possible.
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
//! ### Offset Coordinates
//!
//! Offset assumes that all hexagons have been plotted across a plane where the origin points sits at the bottom left (in theory you can have negative coordinates expanding into the other 3 quadrants but I haven't tested these here).
//!
//! Each node has a label defining its position, known as `(column, row)`.
//!
//! #### Flat Topped - odd columns shifted up
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
//! #### Flat Topped - odd columns shifted down
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
//! ### Pointy Topped - odd rows shifted right
//!
//! Please refer to the README of the proect for an illustration - ascii hexagons with pointy tops are very hard to draw.
//!
//! ### Pointy Topped - odd rows shifted left
//!
//! Please refer to the README of the proect for an illustration - ascii hexagons with pointy tops are very hard to draw.

pub mod astar_axial;
pub mod astar_cubic;
pub mod astar_offset;
pub mod helpers;

/// Specifies the orientation of the hexagon space in Offset layouts. This is
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
	PointyTopOddRight,
	PointyTopOddLeft,
}
