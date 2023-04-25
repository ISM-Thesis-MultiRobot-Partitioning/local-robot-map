use crate::cell_map::CellMap;
use crate::coords::Coords;

/// Describe a map using a polygon.
///
/// The polygon is described using a set of coordinates making up its vertices.
///
/// # Examples
/// ```
/// use local_robot_map::Coords;
/// use local_robot_map::PolygonMap;
///
/// let p1 = Coords::new(0.0, 0.0, 0.0);
/// let p2 = Coords::new(1.0, 1.0, 0.0);
/// let p3 = Coords::new(2.0, 0.0, 0.0);
/// let polygon = PolygonMap::new(vec![p1, p2, p3]);
/// ```
pub struct PolygonMap {
    vertices: Vec<Coords>,
}

impl PolygonMap {
    pub fn new(vertices: Vec<Coords>) -> PolygonMap {
        PolygonMap { vertices }
    }

    /// Convert this map to a [`CellMap`].
    ///
    /// The [`CellMap`] is more straightforward to work with, hence this
    /// function allows to easily make the conversion. The [`PolygonMap`] is
    /// mostly interesting for specifying a map region.
    pub fn to_cell_map(self) -> CellMap {
        todo!("Implement conversion");
    }

    pub fn get_vertices(&self) -> &Vec<Coords> {
        &self.vertices
    }
}
