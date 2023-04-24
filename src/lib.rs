//! Library crate to assist in managing local robot maps in the domain of multi-robot coverage tasks.
//!
//! Robots generally need information about the environment they operate in, and maps are a simple way to store this information. The case being made here is more specifically on multi-robot exploration where one tries to obtain full area coverage. However, care is being put into ensuring that the implemenation can generalize to adjacent disciplines outside of terrain mapping (i.e. perform full map coverage, determine resource concentrations, etc.)
//!
//! The aim is to use this library in a decentralized setting, hence the *local* part in the name. Each robot should hold its own version of the map, upon which it can make decisions. Synchronizing of the maps across robots is outside the scope of this library; this one merely provides a basis on which to get started.

mod coords;
mod polygon_map;
mod cell_map;

pub use coords::Coords;
pub use polygon_map::PolygonMap;
pub use cell_map::CellMap;

/// Visualize a map.
pub trait Visualize {
    fn visualize(&self);
}

/// Partitiong the map.
///
/// This trait requires implementing a partitioning algorithm.
/// Upon calling the `partition()` function, the map will be consumed and a new map with updated information will be returned.
pub trait Partition {
    fn partition(self) -> Self;
}

/// Retrieve a subarea of the map.
///
/// Subareas specified through the [`MapState`] enum are automatically implemented. It is only required to provide the `get_map_region()` function.
pub trait Mask {
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
/// For example, in the case of a [`CellMap`] it allows indicating what the state of each cell is. The [`Mask`] trait allows filtering of the map according to these states.
pub enum MapState {
    /// Indicates the location is outside the map region (mostly relevant for non-square maps such as those which can be produced by [`PolygonMap`])
    OutOfMap,
    /// Indicates the location is occupied by another robot
    OtherRobot,
    /// Indicates the location is occupied by the current robot
    MyRobot,
    /// Indicates the location has already been explored by some robot
    Explored,
    /// Indicates the location has not been explored yet
    Unexplored,
    /// Indicates the location is a frontier which marks boundary between [`MapState::Explored`] and [`MapState::Unexplored`]
    Frontier,
    /// Indicates the location is assigned to the current robot
    Assigned,
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
