use crate::{Coords, MapState, CellMapSize};
use matrix::{prelude::Conventional, Size};

/// Describe a map using a 2D grid of cells.
///
/// Note that only the `x` and `y` components of [`Coords`] are used, and the
/// `z` component will be ignored.
///
/// Another important thing to note is that real-world coordinates are to be
/// provided and output from the [`CellMap`].
pub struct CellMap {
    /// A matrix representing the cells along with their states.
    cells: Conventional<MapState>,
    /// Cell resolution, assumed in *pixels per meter*.
    resolution: f64,
    /// Matrices usually cannot have negative indices, which prevents the
    /// representation of negative real-world coordinates. To this end, an
    /// offset is calculated to bring the map's bounding box's bottom left
    /// corner to `Coords { x: 0.0, y: 0.0, z: 0.0 }`. Even positive
    /// coordinates will be shifted as a matter of consistency.
    offset: Coords,
}

impl CellMap {
    /// Create a new [`CellMap`]. It takes 2 [`Coords`] indicating the square
    /// bounding box area. The resolution affects how many pixels/cells per
    /// meter will be generated.
    ///
    /// # Example
    /// ```
    /// use local_robot_map::{CellMap, Coords, MapState};
    /// use matrix::{matrix, Size};
    ///
    /// let point1 = Coords::new(-1.0, -2.0, 0.0);
    /// let point2 = Coords::new(0.5, 1.0, 0.0);
    /// let resolution = 2.0;
    ///
    /// let map = CellMap::new(point1, point2, resolution);
    ///
    /// assert_eq!(map.resolution(), &2.0);
    /// assert_eq!(map.offset(), &Coords::new(-1.0, -2.0, 0.0));
    /// assert_eq!(map.cells().dimensions(), (3, 6));
    /// ```
    pub fn new(point1: Coords, point2: Coords, resolution: f64) -> Self {
        let size = CellMapSize {
            p1: &point1,
            p2: &point2,
            resolution,
        };

        let offset = Coords {
            x: point1.x.min(point2.x),
            y: point1.y.min(point2.y),
            z: point1.z.min(point2.z),
        };

        Self {
            cells: Conventional::new(size),
            resolution,
            offset,
        }
    }

    pub fn resolution(&self) -> &f64 {
        &self.resolution
    }
    pub fn offset(&self) -> &Coords {
        &self.offset
    }
    pub fn cells(&self) -> &Conventional<MapState> {
        &self.cells
    }
}

impl Size for CellMap {
    fn rows(&self) -> usize {
        self.cells.rows()
    }
    fn columns(&self) -> usize {
        self.cells.columns()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_cell_map_one_by_one() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (1, 1));
        assert_eq!(
            map.offset(),
            &Coords {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_one_by_one_negative() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(-1.0, -1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (1, 1));
        assert_eq!(
            map.offset(),
            &Coords {
                x: -1.0,
                y: -1.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_offset() {
        let (x, y) = (14.26, 95.21);
        let map = CellMap::new(
            Coords::new(x, y, 0.0),
            Coords::new(x + 1.0, y + 1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (1, 1));
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_offset_negative() {
        let (x, y) = (-126.83, -7165.1137);
        let map = CellMap::new(
            Coords::new(x, y, 0.0),
            Coords::new(x + 1.0, y + 1.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (1, 1));
        assert_eq!(map.offset(), &Coords { x, y, z: 0.0 });
    }

    #[test]
    fn create_cell_map_resolution() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(1.0, 1.0, 0.0),
            7.0,
        );
        assert_eq!(map.resolution(), &7.0);
        assert_eq!(map.dimensions(), (7, 7));
        assert_eq!(
            map.offset(),
            &Coords {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_resolution_negative() {
        let map = CellMap::new(
            Coords::new(0.0, 0.0, 0.0),
            Coords::new(-1.0, -1.0, 0.0),
            7.0,
        );
        assert_eq!(map.resolution(), &7.0);
        assert_eq!(map.dimensions(), (7, 7));
        assert_eq!(
            map.offset(),
            &Coords {
                x: -1.0,
                y: -1.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_dimension() {
        let map = CellMap::new(
            Coords::new(1.0, 3.0, 0.0),
            Coords::new(10.0, 4.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (9, 1));
        assert_eq!(
            map.offset(),
            &Coords {
                x: 1.0,
                y: 3.0,
                z: 0.0
            }
        );
    }

    #[test]
    fn create_cell_map_dimension_negative() {
        let map = CellMap::new(
            Coords::new(-10.0, -4.0, 0.0),
            Coords::new(1.0, 3.0, 0.0),
            1.0,
        );
        assert_eq!(map.resolution(), &1.0);
        assert_eq!(map.dimensions(), (11, 7));
        assert_eq!(
            map.offset(),
            &Coords {
                x: -10.0,
                y: -4.0,
                z: 0.0
            }
        );
    }
}
