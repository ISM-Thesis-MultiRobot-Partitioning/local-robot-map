//! Library crate to assist in managing local robot maps in the domain of
//! multi-robot coverage tasks.
//!
//! Robots generally need information about the environment they operate in, and
//! maps are a simple way to store this information. The case being made here is
//! more specifically on multi-robot exploration where one tries to obtain full
//! area coverage. However, care is being put into ensuring that the
//! implemenation can generalize to adjacent disciplines outside of terrain
//! mapping (i.e. perform full map coverage, determine resource concentrations,
//! etc.)
//!
//! The aim is to use this library in a decentralized setting, hence the *local*
//! part in the name. Each robot should hold its own version of the map, upon
//! which it can make decisions. Synchronizing of the maps across robots is
//! outside the scope of this library; this one merely provides a basis on which
//! to get started.

mod cell_map;
mod coords;
mod polygon_map;

pub use cell_map::CellMap;
pub use coords::Coords;
use matrix::{Element, Size};
pub use polygon_map::PolygonMap;

/// Visualize a map.
pub trait Visualize {
    /// This function should visualize the map using a GUI window.
    fn visualize(&self);
}

/// Partitiong the map.
///
/// This trait requires implementing a partitioning algorithm.
/// Upon calling the `partition()` function, the map will be consumed and a new
/// map with updated information will be returned.
pub trait Partition {
    /// Consumes the map to be partitioned, and returns the partitioned version
    /// of the map.
    fn partition(self) -> Self;
}

/// Retrieve a subarea of the map.
///
/// Subareas specified through the [`MapState`] enum are automatically
/// implemented. It is only required to provide the `get_map_region()` function.
pub trait Mask {
    /// Retrieve a subarea of the map by filtering the locations based on a
    /// condition.
    fn get_map_region(&self, f: dyn Fn() -> bool) -> &Self;
    fn get_out_of_map(&self) -> &Self {
        todo!("Implement get_out_of_map");
    }
    fn get_explored(&self) -> &Self {
        todo!("Implement get_explored");
    }
    fn get_unexplored(&self) -> &Self {
        todo!("Implement get_unexplored");
    }
    fn get_frontier(&self) -> &Self {
        todo!("Implement get_frontier");
    }
    fn get_assigned(&self) -> &Self {
        todo!("Implement get_assigned");
    }
}

/// Describe states of locations in the map.
///
/// For example, in the case of a [`CellMap`] it allows indicating what the
/// state of each cell is. The [`Mask`] trait allows filtering of the map
/// according to these states.
#[derive(PartialEq, Copy, Clone)]
pub enum MapState {
    /// Indicates the location is outside the map region (mostly relevant for
    /// non-square maps such as those which can be produced by [`PolygonMap`])
    OutOfMap,
    /// Indicates the location is occupied by another robot
    OtherRobot,
    /// Indicates the location is occupied by the current robot
    MyRobot,
    /// Indicates the location has already been explored by some robot
    Explored,
    /// Indicates the location has not been explored yet
    Unexplored,
    /// Indicates the location is a frontier which marks boundary between
    /// [`MapState::Explored`] and [`MapState::Unexplored`]
    Frontier,
    /// Indicates the location is assigned to the current robot
    Assigned,
}

impl Element for MapState {
    /// [Zero Element](https://en.wikipedia.org/wiki/Zero_element)
    /// should be [`MapState::Unexplored`] as we can assume the area to be
    /// unknown at first.
    fn zero() -> Self {
        MapState::Unexplored
    }
}

/// Struct to holding a pair of numbers implementing the [`Size`] trait.
struct CellMapSize<'a> {
    p1: &'a Coords,
    p2: &'a Coords,
    resolution: f64,
}

impl Size for CellMapSize<'_> {
    fn rows(&self) -> usize {
        let width = (self.p1.x - self.p2.x).abs();
        (width * self.resolution) as usize
    }
    fn columns(&self) -> usize {
        let height = (self.p1.y - self.p2.y).abs();
        (height * self.resolution) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
