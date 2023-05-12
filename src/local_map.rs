use crate::{
    Location, LocationError, MapState, MaskMapState, Partition,
    RealWorldLocation, Visualize,
};

pub struct LocalMap<T>
where
    T: Partition + Location + MaskMapState + Visualize,
{
    map: T,
    my_position: RealWorldLocation,
    other_positions: Vec<RealWorldLocation>,
}

impl<T> LocalMap<T>
where
    T: Partition + Location + MaskMapState + Visualize,
{
    /// Create a [`LocalMap`] which does not allow out-of-map robots.
    ///
    /// If a robot happens to be placed outside the map area, it will be
    /// considered [`LocationError::OutOfMap`]. See also [`LoalMap::new_expand`]
    /// which can deal with out-of-map robots.
    ///
    /// # Errors
    ///
    /// If a robot is placed such that a [`LocationError`] occurs, the function
    /// will return both the error in question as well as the provided
    /// coordinate of the offending robot.
    pub fn new_noexpand(
        mut map: T,
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> Result<Self, (LocationError, RealWorldLocation)> {
        if let Err(location_error) =
            map.set_location(&my_position, MapState::MyRobot)
        {
            return Err((location_error, my_position));
        };

        for pos in &other_positions {
            if let Err(location_error) =
                map.set_location(pos, MapState::OtherRobot)
            {
                return Err((location_error, pos.clone()));
            }
        }

        Ok(Self {
            map,
            my_position,
            other_positions,
        })
    }

    pub fn new_expand(
        mut map: T,
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> Self {
        todo!()
    }

    pub fn map(&self) -> &T {
        &self.map
    }
    pub fn my_position(&self) -> &RealWorldLocation {
        &self.my_position
    }
    pub fn other_positions(&self) -> &Vec<RealWorldLocation> {
        &self.other_positions
    }
}

impl<T> Partition for LocalMap<T>
where
    T: Partition + Location + MaskMapState + Visualize,
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
    T: Partition + Location + MaskMapState + Visualize,
{
    type ImageType = <T as Visualize>::ImageType;

    fn as_image(&self) -> Self::ImageType {
        self.map.as_image()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        cell_map::tests::make_map, CellMap, LocationType, RealWorldLocation,
    };

    // dummy implementation for testing purposes
    impl Partition for CellMap {
        fn partition(self) -> Self {
            self
        }
    }

    fn make_random_local_map(
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> LocalMap<CellMap> {
        let (map, _) = make_map();

        LocalMap::new_noexpand(map, my_position, other_positions).unwrap()
    }

    fn make_local_map(
        my_position: RealWorldLocation,
        other_positions: Vec<RealWorldLocation>,
    ) -> LocalMap<CellMap> {
        LocalMap::new_noexpand(
            CellMap::new(
                RealWorldLocation::from_xyz(0.0, 0.0, 0.0),
                RealWorldLocation::from_xyz(10.0, 10.0, 10.0),
                crate::AxisResolution::uniform(1.0),
            ),
            my_position,
            other_positions,
        ).unwrap()
    }

    fn get_mapstate_pos_from_map(
        map: &CellMap,
        state: LocationType,
    ) -> Vec<RealWorldLocation> {
        map.get_map_state(state)
            .iter()
            .map(|cell| cell.location().clone())
            .collect()
    }

    #[test]
    fn get_my_position() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let my_map_pos: Vec<RealWorldLocation> =
            get_mapstate_pos_from_map(lmap.map(), LocationType::MyRobot);

        assert_eq!(
            my_map_pos.len(),
            1,
            "There should only be 1 position for my robot"
        );
        assert_eq!(lmap.my_position(), &my_map_pos[0]);
    }

    #[test]
    fn create_local_map_other_positions_no_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 0, "There should only be no other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn create_local_map_other_positions_one_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![RealWorldLocation::from_xyz(1.0, 1.0, 0.0)];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 1, "There should only be 1 other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn create_local_map_other_positions_multiple_robots() {
        let my_position = RealWorldLocation::from_xyz(0.0, 0.0, 0.0);
        let other_positions = vec![
            RealWorldLocation::from_xyz(1.0, 1.0, 0.0),
            RealWorldLocation::from_xyz(2.0, 2.0, 0.0),
            RealWorldLocation::from_xyz(3.0, 3.0, 0.0),
        ];

        let lmap = make_local_map(my_position, other_positions);
        let positions =
            get_mapstate_pos_from_map(lmap.map(), LocationType::OtherRobot);

        assert_eq!(positions.len(), 3, "There should only be 3 other robots");
        assert_eq!(lmap.other_positions(), &positions);
    }

    #[test]
    fn partition_map() {
        let lmap = make_random_local_map(RealWorldLocation::from_xyz(0.0, 0.0, 0.0), vec![]);
        lmap.partition();
    }

    #[test]
    fn call_map_trait_function_visualize() {
        let lmap = make_random_local_map(RealWorldLocation::from_xyz(0.0, 0.0, 0.0), vec![]);
        lmap.map().as_image();
    }

    #[test]
    fn call_map_trait_function_visualize_and_then_save() {
        let lmap = make_random_local_map(RealWorldLocation::from_xyz(0.0, 0.0, 0.0), vec![]);
        lmap.map()
            .as_image()
            .save("test_save_local_map.jpg")
            .unwrap();
    }

    #[test]
    fn call_map_trait_function_mask_mapstate() {
        let lmap = make_random_local_map(RealWorldLocation::from_xyz(0.0, 0.0, 0.0), vec![]);
        lmap.map().get_map_state(LocationType::Unexplored);
    }
}
