use crate::{Coords, MaskMapState, Partition, Visualize};

pub struct LocalMap<T>
where
    T: Partition + MaskMapState + Visualize,
{
    map: T,
    my_position: Coords,
    other_positions: Vec<Coords>,
}

impl<T> LocalMap<T>
where
    T: Partition + MaskMapState + Visualize,
{
    pub fn new(
        map: T,
        my_position: Coords,
        other_positions: Vec<Coords>,
    ) -> Self {
        Self {
            map,
            my_position,
            other_positions,
        }
    }

    pub fn map(&self) -> &T {
        &self.map
    }
    pub fn my_position(&self) -> &Coords {
        &self.my_position
    }
    pub fn other_positions(&self) -> &Vec<Coords> {
        &self.other_positions
    }

    fn update_my_position(&mut self, new_position: Coords) {
        self.my_position = new_position;
        self.map();
    }
}

impl<T> Partition for LocalMap<T>
where
    T: Partition + MaskMapState + Visualize,
{
    fn partition(self) -> Self {
        Self {
            map: self.map.partition(),
            my_position: self.my_position,
            other_positions: self.other_positions,
        }
    }
}

impl<T> Visualize for LocalMap<T>
where
    T: Partition + MaskMapState + Visualize,
{
    type ImageType = <T as Visualize>::ImageType;

    fn as_image(&self) -> Self::ImageType {
        self.map.as_image()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{cell_map::tests::make_map, CellMap, MapState, RealWorldLocation};

    // dummy implementation for testing purposes
    impl Partition for CellMap {
        fn partition(self) -> Self {
            self
        }
    }

    fn make_random_local_map(
        my_position: Coords,
        other_positions: Vec<Coords>,
    ) -> LocalMap<CellMap> {
        let (map, _) = make_map();

        LocalMap::new(map, my_position, other_positions)
    }

    fn make_local_map(
        my_position: Coords,
        other_positions: Vec<Coords>,
    ) -> LocalMap<CellMap> {
        LocalMap::new(
            CellMap::new(
                RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
                RealWorldLocation::from_xyz(10.0, 10.0, 10.0),
                crate::AxisResolution::uniform(1.0),
            ),
            my_position,
            other_positions,
        )
    }

    fn get_mapstate_pos_from_map(map: &CellMap, state: MapState) -> Vec<Coords> {
        map.get_map_state(state)
            .iter()
            .map(|cell| Coords::new(*cell.x(), *cell.y(), 0.0))
            .collect()
    }

    #[test]
    fn get_my_position() {
        let my_position = Coords::new(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let my_map_pos: Vec<Coords> = get_mapstate_pos_from_map(lmap.map(), MapState::MyRobot);

        assert_eq!(my_map_pos.len(), 1, "There should only be 1 position for my robot");
        assert_eq!(lmap.my_position(), &my_map_pos[0]);
    }

    #[test]
    fn create_local_map_other_positions_no_robots() {
        let my_position = Coords::new(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let positions = get_mapstate_pos_from_map(lmap.map(), MapState::OtherRobot);

        assert_eq!(positions.len(), 0, "There should only be no other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn create_local_map_other_positions_one_robots() {
        let my_position = Coords::new(0.0, 0.0, 0.0);
        let other_positions = vec![Coords::new(1.0, 1.0, 0.0)];

        let lmap = make_local_map(my_position, other_positions);
        let positions = get_mapstate_pos_from_map(lmap.map(), MapState::OtherRobot);

        assert_eq!(positions.len(), 1, "There should only be 1 other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn create_local_map_other_positions_multiple_robots() {
        let my_position = Coords::new(0.0, 0.0, 0.0);
        let other_positions = vec![
            Coords::new(1.0, 1.0, 0.0),
            Coords::new(2.0, 2.0, 0.0),
            Coords::new(3.0, 3.0, 0.0),
        ];

        let lmap = make_local_map(my_position, other_positions);
        let positions = get_mapstate_pos_from_map(lmap.map(), MapState::OtherRobot);

        assert_eq!(positions.len(), 3, "There should only be 3 other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn get_map() {
        let lmap = make_random_local_map(Coords::new(0.0, 0.0, 0.0), vec![]);
        let (map, _) = make_map();
        assert_eq!(lmap.map(), &map);
    }

    #[test]
    fn partition_map() {
        let lmap = make_random_local_map(Coords::new(0.0, 0.0, 0.0), vec![]);
        lmap.partition();
    }

    #[test]
    fn call_map_trait_function_visualize() {
        let lmap = make_random_local_map(Coords::new(0.0, 0.0, 0.0), vec![]);
        lmap.map().as_image();
    }

    #[test]
    fn call_map_trait_function_visualize_and_then_save() {
        let lmap = make_random_local_map(Coords::new(0.0, 0.0, 0.0), vec![]);
        lmap.map()
            .as_image()
            .save("test_save_local_map.jpg")
            .unwrap();
    }

    #[test]
    fn call_map_trait_function_mask_mapstate() {
        let lmap = make_random_local_map(Coords::new(0.0, 0.0, 0.0), vec![]);
        lmap.map().get_map_state(MapState::Unexplored);
    }
}
