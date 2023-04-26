use geo::BoundingRect;
use geo_rasterize::BinaryBuilder;
use matrix::prelude::{Compressed, Conventional};
use matrix::{Matrix, Size};

use crate::cell_map::CellMap;
use crate::coords::Coords;
use crate::MapState;

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
    polygon: geo::Polygon,
}

impl PolygonMap {
    pub fn new(vertices: Vec<Coords>) -> PolygonMap {
        let polygon = geo::Polygon::new(
            geo::LineString::from(
                vertices.iter().map(|e| (e.x, e.y)).collect::<Vec<_>>(),
            ),
            vec![],
        );
        PolygonMap { vertices, polygon }
    }

    /// Convert this map to a [`CellMap`].
    ///
    /// The [`CellMap`] is more straightforward to work with, hence this
    /// function allows to easily make the conversion. The [`PolygonMap`] is
    /// mostly interesting for specifying a map region.
    ///
    /// The `resolution` parameter specify the pixels per meter.
    pub fn to_cell_map(self, resolution: f64) -> CellMap {
        CellMap::from_raster(
            self.rasterize_polygon(resolution),
            resolution,
            Coords {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        )
    }

    /// .
    ///
    /// # Panics
    ///
    /// The [`geo`] crate allows to obtain the polygon's *bounding box* as well
    /// *rasterizing* it. Both of these procedures can cause panics, which
    /// is not expected to happen. They boil down to **invalid polygon shapes**
    /// and **NaN or infinite values** in the vertex coordinates.
    ///
    /// - Extract the Bounding Box: will panic if no bounding box can be made. A
    ///   properly formed polygon should always have a properly defined bounding
    ///   box. It should be checked elsewhere that the polygons have a valid
    ///   shape.
    /// - The *BinaryBuilder* used to rasterize the polygon can panic as well if
    ///   there are NaN of infinite values.
    /// - The rasterization itself can panic as well if there are NaN of
    ///   infinite values.
    fn rasterize_polygon(&self, resolution: f64) -> Conventional<MapState> {
        let bbox = match self.polygon.bounding_rect() {
            Some(b) => b,
            None => panic!("No bounding box for polygon"),
        };
        let width = (bbox.width() * resolution) as usize;
        let height = (bbox.height() * resolution) as usize;

        let mut rasterizer = BinaryBuilder::new()
            .width(width)
            .height(height)
            .build()
            .expect("There should be no NaN or infinite values among the polygon vertices");

        rasterizer
            .rasterize(&self.polygon)
            .expect("There should be no NaN of infinite values");

        let mut sparse_matrix = Compressed::zero((width, height));
        println!("{:?}", sparse_matrix.dimensions());
        rasterizer
            .finish()
            .indexed_iter()
            .filter(|(_, v)| !**v)
            .for_each(|((column, row), _)| {
                // The Zero Element is [`MapState::Unknown`], so we set all
                // cells outside the polygon to
                // [`MapState::OutOfMap`].
                sparse_matrix.set((row, column), MapState::OutOfMap);
            });

        Conventional::from(sparse_matrix)
    }

    pub fn get_vertices(&self) -> &Vec<Coords> {
        &self.vertices
    }
}

#[cfg(test)]
mod tests {
    use matrix::matrix;

    use super::*;

    #[test]
    fn polygon_map_to_cell_map_positive() {
        let p1 = Coords::new(0.0, 0.0, 0.0);
        let p2 = Coords::new(4.0, 4.0, 0.0);
        let p3 = Coords::new(8.0, 0.0, 0.0);
        let resolution = 1.0;
        let cellmap = PolygonMap::new(vec![p1, p2, p3]).to_cell_map(resolution);

        assert_eq!(cellmap.dimensions(), (8, 4));

        const OOM: MapState = MapState::OutOfMap;
        const UNE: MapState = MapState::Unexplored;
        assert_eq!(
            cellmap.cells(),
            &Conventional::from_vec(
                (4, 2),
                matrix![
                    OOM, OOM, OOM, UNE, UNE, OOM, OOM, OOM;
                    OOM, OOM, UNE, UNE, UNE, UNE, OOM, OOM;
                    OOM, UNE, UNE, UNE, UNE, UNE, UNE, OOM;
                    UNE, UNE, UNE, UNE, UNE, UNE, UNE, UNE;
                ]
            )
        )
    }

    #[test]
    fn polygon_map_to_cell_map_negative() {
        todo!("Check if conversion works if polygon's vertices only have negative coordinates");
    }

    #[test]
    fn polygon_map_to_cell_map_partly_negative() {
        todo!(
            "Check if conversion works if some of the polygon's vertices have negative coordinates"
        );
    }
}
