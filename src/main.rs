extern crate nalgebra as na;

use std::collections::HashMap;
use std::default::Default;

use ggez::event::{EventHandler, KeyCode};
use ggez::graphics::Color;
use ggez::graphics::{DrawParam, Image, Rect};
use ggez::input::keyboard;
use ggez::{graphics, Context, ContextBuilder, GameResult};

use na::{DMatrix, Isometry2, Point2, Scalar, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::algebra::Velocity2;
use nphysics2d::object::{
    BodyStatus, Collider, ColliderDesc, DefaultBodyHandle, DefaultColliderHandle, RigidBody,
    RigidBodyDesc,
};
use nphysics2d::{material, object, world};
use std::f32::consts::PI;
use tiled::PropertyValue;

type N = f32;
type TileN = usize;

const IMAGE_WIDTH: N = 250.;
const IMAGE_HEIGHT: N = 250.;

const BACKGROUND_COLOR: (u8, u8, u8) = (152, 152, 152);

// for compatibility with the nalgebra that ggez uses
fn point_to_old<N: Copy + Scalar>(point: Point2<N>) -> ggez::nalgebra::Point2<N> {
    ggez::nalgebra::Point2::new(point.x, point.y)
}
fn vector_to_old<N: Copy + Scalar>(vector: Vector2<N>) -> ggez::nalgebra::Vector2<N> {
    ggez::nalgebra::Vector2::new(vector.x, vector.y)
}

fn isometry_to_point<N: na::RealField + Copy + Scalar>(isometry: Isometry2<N>) -> Point2<N> {
    isometry.translation.vector.into()
}
fn point_to_isometry<N: na::RealField + Copy + Scalar>(point: Point2<N>) -> Isometry2<N> {
    Isometry2::translation(point.x, point.y)
}

fn tuple_to_point<N: Scalar>((x, y): (N, N)) -> Point2<N> {
    Point2::new(x, y)
}
fn point_to_tuple<N: Copy + Scalar>(point: Point2<N>) -> (N, N) {
    (point.x, point.y)
}

fn point_to_vector<N: Copy + Scalar>(point: Point2<N>) -> Vector2<N> {
    Vector2::new(point.x, point.y)
}
fn point_to_velocity<N: na::RealField + Copy + Scalar>(point: Point2<N>) -> Velocity2<N> {
    Velocity2::linear(point.x, point.y)
}

fn add<N: std::ops::Add<Output = N> + Copy + Scalar>(
    point1: Point2<N>,
    point2: Point2<N>,
) -> Point2<N> {
    Point2::new(point1.x + point2.x, point1.y + point2.y)
}

fn sub<N: std::ops::Sub<Output = N> + Copy + Scalar>(
    point1: Point2<N>,
    point2: Point2<N>,
) -> Point2<N> {
    Point2::new(point1.x - point2.x, point1.y - point2.y)
}

type Id = u32;
fn get_id() -> Id {
    use std::sync::atomic;
    static COUNTER: atomic::AtomicUsize = atomic::AtomicUsize::new(1);
    COUNTER.fetch_add(1, atomic::Ordering::Relaxed) as Id
}

#[allow(dead_code)]
fn draw_point(ctx: &mut Context, point: Point2<N>, color: graphics::Color) -> GameResult {
    let circle = graphics::Mesh::new_circle(
        ctx,
        graphics::DrawMode::Fill(graphics::FillOptions::DEFAULT),
        point_to_old(point),
        1.,
        0.,
        color,
    )?;
    graphics::draw(ctx, &circle, DrawParam::new())?;
    Ok(())
}

#[allow(dead_code)]
fn draw_rect(ctx: &mut Context, rect: Rect, color: graphics::Color) -> GameResult {
    let circle = graphics::Mesh::new_rectangle(
        ctx,
        graphics::DrawMode::Stroke(graphics::StrokeOptions::DEFAULT),
        rect,
        color,
    )?;
    graphics::draw(ctx, &circle, DrawParam::new())?;
    Ok(())
}

fn parse_property(tile: &tiled::Tile, name: &str) -> bool {
    *tile
        .properties
        .get(name)
        .and_then(|val| match val {
            PropertyValue::BoolValue(val) => Some(val),
            _ => None,
        })
        .expect("Tile property not found.")
}

#[derive(Debug)]
struct Entity {
    draw_param: DrawParam,
    body_handle: DefaultBodyHandle,
    collider_handle: DefaultColliderHandle,
}

impl Entity {
    pub fn new(
        physics: &mut Physics,
        sprite_pos: Point2<N>,
        pos: Point2<N>,
        size: Point2<N>,
        rigid_body_desc: RigidBodyDesc<N>,
        is_sensor: bool,
        user_data: ObjectType,
        ccd_enabled: bool,
    ) -> Self {
        let draw_param = DrawParam::new()
            .src(Rect::new(
                sprite_pos.x * (1. / IMAGE_WIDTH),
                sprite_pos.y * (1. / IMAGE_HEIGHT),
                size.x / IMAGE_WIDTH,
                size.y / IMAGE_HEIGHT,
            ))
            .offset(point_to_old(Point2::new(0.5, 0.5)));

        let body_handle = physics.build_body(rigid_body_desc.translation(point_to_vector(pos)));

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
            size.x / 2. - 0.01,
            size.y / 2. - 0.01,
        )));

        let collider_handle = physics.build_collider(
            ColliderDesc::new(shape)
                .sensor(is_sensor)
                // increasing linear_prediction from a default of 0.001 is necessary to prevent a bug
                // where the engine doesn't detect the collision between the player and a
                // downwards moving platform
                .linear_prediction(0.01)
                .user_data(user_data),
            body_handle,
            ccd_enabled,
        );

        Entity {
            draw_param,
            body_handle,
            collider_handle,
        }
    }

    pub fn draw(&self, ctx: &mut Context, physics: &Physics, spritesheet: &Image) -> GameResult {
        let pos = self.position(physics);
        graphics::draw(ctx, spritesheet, self.draw_param.dest(point_to_old(pos)))?;
        Ok(())
    }

    pub fn rigid_body<'a>(&self, physics: &'a Physics) -> &'a RigidBody<N> {
        physics.rigid_body(self.body_handle)
    }

    pub fn rigid_body_mut<'a>(&mut self, physics: &'a mut Physics) -> &'a mut RigidBody<N> {
        physics.rigid_body_mut(self.body_handle)
    }

    pub fn position(&self, physics: &Physics) -> Point2<N> {
        isometry_to_point(*self.rigid_body(physics).position())
    }

    pub fn set_position(&mut self, physics: &mut Physics, point: Point2<N>) {
        self.rigid_body_mut(physics)
            .set_position(point_to_isometry(point));
    }

    pub fn velocity(&self, physics: &Physics) -> Point2<N> {
        self.rigid_body(physics).velocity().linear.into()
    }

    pub fn set_velocity(&mut self, physics: &mut Physics, velocity: Point2<N>) {
        self.rigid_body_mut(physics)
            .set_linear_velocity(velocity.coords);
    }
}

#[derive(Debug)]
struct Player {
    entity: Entity,
    has_jumped: bool,
}

impl Player {
    const MOVEMENT_POWER: N = 100.;
    const JUMP_POWER: N = 200.;
    const SIZE: (N, N) = (10., 10.);
    const SPRITE_POS: (N, N) = (0., 0.);
    const MASS: N = 10.;

    pub fn new(physics: &mut Physics, entrance: Point2<TileN>) -> Self {
        let entity = Entity::new(
            physics,
            tuple_to_point(Self::SPRITE_POS),
            Tile::point_to_real(entrance),
            tuple_to_point(Self::SIZE),
            RigidBodyDesc::new().mass(Self::MASS),
            false,
            ObjectType::Player,
            true,
        );
        Self {
            entity,
            has_jumped: false,
        }
    }

    pub fn reset(&mut self, physics: &mut Physics, entrance: Point2<TileN>) {
        self.entity
            .set_position(physics, Tile::point_to_real(entrance));
    }

    pub fn update(&mut self, ctx: &mut Context, physics: &mut Physics, tilemap: &Tilemap) {
        let movement_dir = if keyboard::is_key_pressed(ctx, KeyCode::Left) {
            -1.
        } else if keyboard::is_key_pressed(ctx, KeyCode::Right) {
            1.
        } else {
            0.
        };

        let mut new_velocity = self.entity.velocity(physics);
        new_velocity[physics.gravity_dir.opposite_axis()] = 0.;
        let movement_vector = physics.gravity_dir.inverse().to_point() * movement_dir;

        self.entity.set_velocity(
            physics,
            add(new_velocity, movement_vector * Self::MOVEMENT_POWER),
        );

        let mut on_ground = false;
        for (user_datas, manifold) in physics.collisions(self.entity.collider_handle) {
            match user_datas {
                (ObjectType::Player, other) | (other, ObjectType::Player) => match other {
                    ObjectType::Bullet(_, _) => {
                        self.reset(physics, tilemap.current_level_info().entrance.clone());
                    }
                    other @ ObjectType::Platform(_) | other @ ObjectType::Tile(_, _) => {
                        if physics.on_ground(&manifold) {
                            on_ground = true;
                        }

                        match other {
                            ObjectType::Tile(_, tile_instance_id) => {
                                let collided_tile_instance =
                                    &tilemap.current_level.tiles[&tile_instance_id];
                                let collided_tile = Tilemap::get_tile_from_tile_id(
                                    &tilemap.tiles,
                                    collided_tile_instance.tile_id,
                                );
                                match collided_tile.type_ {
                                    TileType::Gravity => {
                                        let gravity_dir =
                                            collided_tile_instance.direction.opposite();
                                        if gravity_dir != physics.gravity_dir {
                                            physics.set_gravity_dir(gravity_dir, self);
                                        }
                                    }
                                    _ => (),
                                }
                            }
                            ObjectType::Platform(rail_id) => {
                                if on_ground {
                                    let mut platform_velocity = tilemap.current_level.rails
                                        [&rail_id]
                                        .platform
                                        .velocity(physics);

                                    let velocity = self.entity.velocity(physics);
                                    platform_velocity[physics.gravity_dir.opposite_axis()] +=
                                        velocity[physics.gravity_dir.opposite_axis()];

                                    self.entity.set_velocity(physics, platform_velocity);
                                }
                            }
                            _ => (),
                        }
                    }
                    ObjectType::Player => (),
                },
                _ => (),
            }
        }

        // has_jumped ensures that player can jump in the very beginning, when
        // no collisions have been registered yet
        if (on_ground || !self.has_jumped) && keyboard::is_key_pressed(ctx, KeyCode::Up) {
            self.has_jumped = true;
            let jump_vector = physics.gravity_dir.to_point() * -Self::JUMP_POWER;
            let mut velocity = self.entity.velocity(physics);
            velocity[physics.gravity_dir.axis()] = 0.;
            self.entity
                .set_velocity(physics, add(jump_vector, velocity));
        }
    }
}

#[derive(Debug)]
struct Tile {
    type_: TileType,
    info: tiled::Tile,
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum TileType {
    Entrance,
    Exit,
    Spikes,
    Wall,
    Gun,
    Rail,
    Gravity,
    None,
}

#[derive(Debug)]
enum TileData {
    GunData(GunTile),
    RailData(bool),
    None,
}

const WALL_TILE_TYPES: &[TileType] = &[TileType::Wall, TileType::Gun, TileType::Gravity];

impl Tile {
    const SIZE: N = 10.;

    pub fn point_to_real(point: Point2<TileN>) -> Point2<N> {
        Point2::new(point.x as N, point.y as N) * Self::SIZE
    }
}

#[derive(Debug)]
struct TileInstance {
    id: TileInstanceId,
    tile_id: TileId,
    direction: Direction,
    draw_param: DrawParam,
    coords: Point2<usize>,
    extra_data: TileData,
}

impl TileInstance {
    pub fn new(
        physics: &mut Physics,
        coords: Point2<TileN>,
        tile: &Tile,
        tile_id: TileId,
        tile_instance_id: TileInstanceId,
    ) -> Self {
        let real_point = Tile::point_to_real(coords);
        let vector = point_to_vector(real_point);

        let is_solid = parse_property(&tile.info, "is_solid");

        let (shape, translation) = match &tile.info.objectgroup {
            Some(object_group) => {
                let object = &object_group.objects[0];
                match object.shape {
                    tiled::ObjectShape::Rect { width, height } => {
                        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
                            width / 2. - 0.01,
                            height / 2. - 0.01,
                        )));
                        // TODO figure out a better fix than (object.x, object.y) / 2.
                        let translation = vector + Vector2::new(object.x, object.y) / 2.;

                        (shape, translation)
                    }
                    _ => panic!("Unrecognized collider shape."),
                }
            }
            None => {
                let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
                    Tile::SIZE / 2. - 0.01,
                    Tile::SIZE / 2. - 0.01,
                )));
                let translation = vector;

                (shape, translation)
            }
        };

        if WALL_TILE_TYPES.contains(&tile.type_) {
            physics.build_collider(
                ColliderDesc::new(shape)
                    .translation(translation)
                    .user_data(ObjectType::Tile(tile.info.id, tile_instance_id)),
                physics.ground_handle,
                false,
            );
        } else {
            let body_handle = physics.build_body(
                RigidBodyDesc::new()
                    .translation(translation)
                    .status(BodyStatus::Static),
            );
            physics.build_collider(
                ColliderDesc::new(shape)
                    .sensor(!is_solid)
                    .user_data(ObjectType::Tile(tile.info.id, tile_instance_id)),
                body_handle,
                false,
            );
        }

        let tile_spritesheet_width = (IMAGE_WIDTH / Tile::SIZE).floor() as Id;
        let (src_x, src_y) = (
            tile.info.id % tile_spritesheet_width,
            (tile.info.id / tile_spritesheet_width),
        );

        let direction = Direction::from_id(tile_id);
        let rotation = direction.to_rotation();

        let draw_param = graphics::DrawParam::new()
            .src(Rect::new(
                src_x as N * (Tile::SIZE / IMAGE_WIDTH),
                src_y as N * (Tile::SIZE / IMAGE_HEIGHT),
                Tile::SIZE / IMAGE_WIDTH,
                Tile::SIZE / IMAGE_HEIGHT,
            ))
            .offset(point_to_old(Point2::new(0.5, 0.5)))
            .rotation(rotation)
            .dest(point_to_old(real_point.clone()));

        let extra_data = match tile.type_ {
            TileType::Gun => TileData::GunData(GunTile::new()),
            TileType::Rail => TileData::RailData(parse_property(&tile.info, "is_endpoint")),
            _ => TileData::None,
        };

        TileInstance {
            id: get_id(),
            tile_id: tile.info.id,
            draw_param,
            direction,
            coords: coords.clone(),
            extra_data,
        }
    }
}

type BulletId = Id;
#[derive(Debug)]
struct GunTile {
    bullets: HashMap<BulletId, Entity>,
}

impl GunTile {
    const SHOOTING_FREQUENCY: usize = 100;
    const BULLET_SIZE: (N, N) = (4., 4.);
    const BULLET_SPRITE_POS: (N, N) = (10., 0.);
    const BULLET_SPEED: N = 40.;

    pub fn new() -> Self {
        Self {
            bullets: HashMap::new(),
        }
    }

    pub fn shoot(
        &mut self,
        physics: &mut Physics,
        pos: Point2<TileN>,
        dir: Direction,
        tile_instance_id: TileInstanceId,
    ) {
        let bullet_id = get_id();
        let entity = Entity::new(
            physics,
            tuple_to_point(Self::BULLET_SPRITE_POS),
            add(Tile::point_to_real(pos), dir.to_point() * Tile::SIZE),
            tuple_to_point(Self::BULLET_SIZE),
            RigidBodyDesc::new()
                .status(BodyStatus::Dynamic)
                .gravity_enabled(false)
                .velocity(point_to_velocity(dir.to_point() * Self::BULLET_SPEED)),
            false,
            ObjectType::Bullet(tile_instance_id, bullet_id),
            false,
        );
        self.bullets.insert(bullet_id, entity);
    }

    pub fn remove_bullet(&mut self, physics: &mut Physics, bullet_id: BulletId) {
        let bullet = self.bullets.remove(&bullet_id).unwrap();
        physics.body_set.remove(bullet.body_handle);
    }

    pub fn draw(&self, ctx: &mut Context, physics: &Physics, spritesheet: &Image) -> GameResult {
        for bullet in self.bullets.values() {
            bullet.draw(ctx, physics, spritesheet)?;
        }

        Ok(())
    }
}

type RailId = Id;
type TileInstanceId = Id;
#[derive(Debug, Default)]
struct Level {
    tiles: HashMap<TileInstanceId, TileInstance>,
    rails: HashMap<RailId, Rail>,
}

#[derive(Debug, Clone, Copy)]
enum Phase {
    Forwards,
    Backwards,
}

#[derive(Debug)]
struct Rail {
    rail: Vec<Point2<TileN>>,
    platform: Entity,
    phase: Phase,
    rail_i: usize,
}

impl Iterator for Rail {
    type Item = Point2<TileN>;

    fn next(&mut self) -> Option<Self::Item> {
        self.phase = if self.rail_i == 0 {
            Phase::Forwards
        } else if self.rail_i == self.rail.len() - 1 {
            Phase::Backwards
        } else {
            self.phase
        };
        self.rail_i = match self.phase {
            Phase::Forwards => self.rail_i + 1,
            Phase::Backwards => self.rail_i.checked_sub(1).unwrap(),
        };
        Some(self.rail[self.rail_i].clone())
    }
}

impl Rail {
    const PLATFORM_SIZE: (N, N) = (10., 10.);
    const PLATFORM_SPRITE_POS: (N, N) = (14., 0.);
    const PLATFORM_SPEED: N = 20.;
    pub fn new(physics: &mut Physics, rail: Vec<Point2<TileN>>, rail_id: RailId) -> Self {
        let platform = Entity::new(
            physics,
            tuple_to_point(Self::PLATFORM_SPRITE_POS),
            Tile::point_to_real(rail[0].clone()),
            tuple_to_point(Self::PLATFORM_SIZE),
            RigidBodyDesc::new().status(BodyStatus::Kinematic),
            false,
            ObjectType::Platform(rail_id),
            false,
        );
        Self {
            rail,
            platform,
            phase: Phase::Forwards,
            rail_i: 0,
        }
    }

    pub fn update(&mut self, physics: &mut Physics) {
        let dest = self.rail[self.rail_i].clone();
        let pos_diff = sub(self.platform.position(physics), Tile::point_to_real(dest));
        let cutoff = Self::PLATFORM_SPEED / 100.;
        if pos_diff.x.abs() < cutoff && pos_diff.y.abs() < cutoff {
            let next_dest = self.next().unwrap();
            self.platform.rigid_body_mut(physics).set_velocity(
                Velocity2::linear(
                    next_dest.x as N - dest.clone().x as N,
                    next_dest.y as N - dest.clone().y as N,
                ) * Self::PLATFORM_SPEED,
            );
        }
    }

    pub fn draw(&self, ctx: &mut Context, physics: &Physics, spritesheet: &Image) -> GameResult {
        self.platform.draw(ctx, physics, spritesheet)?;
        Ok(())
    }
}

impl Level {
    pub fn new(
        tilematrix: &DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        physics: &mut Physics,
    ) -> Self {
        let tile_instances: HashMap<TileInstanceId, TileInstance> = tilematrix
            .iter()
            .enumerate()
            .filter_map(|(i, tile_id)| {
                let tile = Tilemap::get_tile_from_tile_id(tiles, *tile_id);
                let tile_instance_id = get_id();
                if tile.type_ != TileType::None {
                    Some((
                        tile_instance_id,
                        TileInstance::new(
                            physics,
                            tuple_to_point(tilematrix.vector_to_matrix_index(i)),
                            tile,
                            *tile_id,
                            tile_instance_id,
                        ),
                    ))
                } else {
                    None
                }
            })
            .collect();

        let mut rail_set: Vec<Vec<Point2<TileN>>> = Vec::new();

        for tile_instance in tile_instances.values() {
            if let TileData::RailData(true) = tile_instance.extra_data {
                if !rail_set
                    .iter()
                    .flatten()
                    .collect::<Vec<_>>()
                    .contains(&&tile_instance.coords)
                {
                    let mut rail = Vec::new();
                    Self::build_rail(tilematrix, tiles, &mut rail, tile_instance.coords.clone());
                    rail_set.push(rail);
                }
            }
        }

        let rails = rail_set
            .into_iter()
            .map(|rail| {
                let rail_id = get_id();
                (rail_id, Rail::new(physics, rail, rail_id))
            })
            .collect();

        Level {
            tiles: tile_instances,
            rails,
        }
    }

    fn build_rail(
        tilematrix: &DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        rail: &mut Vec<Point2<TileN>>,
        coords: Point2<TileN>,
    ) {
        rail.push(coords.clone());

        Direction::create_adjascent(coords)
            .into_iter()
            .flatten()
            .find(|coords| {
                !rail.contains(&coords)
                    && Tilemap::get_tile_from_matrix(tilematrix, tiles, (*coords).clone()).type_
                        == TileType::Rail
            })
            .map(|new_coords| Self::build_rail(tilematrix, tiles, rail, new_coords.clone()));
    }

    pub fn update(&mut self, physics: &mut Physics) {
        for (instance_id, tile) in self.tiles.iter_mut() {
            match &mut tile.extra_data {
                TileData::GunData(ref mut gun_tile) => {
                    if physics.ticks % GunTile::SHOOTING_FREQUENCY == 0 {
                        gun_tile.shoot(physics, tile.coords.clone(), tile.direction, *instance_id);
                    }
                }
                _ => (),
            }
        }

        for rail in self.rails.values_mut() {
            rail.update(physics);
        }
    }

    pub fn draw(
        &self,
        ctx: &mut Context,
        physics: &Physics,
        tilesheet: &Image,
        spritesheet: &Image,
    ) -> GameResult {
        for tile in self.tiles.values() {
            graphics::draw(ctx, tilesheet, tile.draw_param)?;
        }
        // draw bullets after all tiles
        for tile in self.tiles.values() {
            match &tile.extra_data {
                TileData::GunData(gun_tile) => gun_tile.draw(ctx, physics, spritesheet)?,
                _ => (),
            }
        }

        for rail in self.rails.values() {
            rail.draw(ctx, physics, spritesheet)?;
        }

        Ok(())
    }
}

#[derive(Debug)]
struct LevelInfo {
    tilematrix: DMatrix<TileId>,
    wallpaper: Vec<Point2<TileN>>,
    entrance: Point2<TileN>,
}

const ID_FOR_NO_TILE: TileId = 0b000100000000;
const FLIP_H: TileId = 0b100000000000;
const FLIP_V: TileId = 0b010000000000;
const FLIP_D: TileId = 0b001000000000;

fn apply_transformations(layer_tile: &tiled::LayerTile) -> TileId {
    let mut id = layer_tile.gid;
    if layer_tile.flip_h {
        id = id | FLIP_H
    };
    if layer_tile.flip_v {
        id = id | FLIP_V
    };
    if layer_tile.flip_d {
        id = id | FLIP_D
    };
    id
}
fn strip_transformations(id: TileId) -> TileId {
    id & !FLIP_H & !FLIP_V & !FLIP_D
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    fn from_id(id: TileId) -> Self {
        use Direction::*;
        match (id & FLIP_H != 0, id & FLIP_V != 0, id & FLIP_D != 0) {
            (false, _, true) => West,
            (true, _, true) => East,

            (_, true, false) => South,
            (_, false, false) => North,
        }
    }

    fn to_rotation(self) -> N {
        use Direction::*;
        match self {
            North => 0.,
            East => PI / 2.,
            South => PI,
            West => 3. * PI / 2.,
        }
    }

    fn to_point(self) -> Point2<N> {
        use Direction::*;
        match self {
            West => Point2::new(-1., 0.),
            North => Point2::new(0., -1.),
            East => Point2::new(1., 0.),
            South => Point2::new(0., 1.),
        }
    }

    fn to_vector(self) -> Vector2<N> {
        point_to_vector(self.to_point())
    }

    fn opposite(self) -> Self {
        use Direction::*;
        match self {
            South => North,
            East => West,
            North => South,
            West => East,
        }
    }

    fn inverse(self) -> Self {
        use Direction::*;
        match self {
            South => East,
            East => North,
            North => West,
            West => South,
        }
    }

    fn axis(self) -> usize {
        use Direction::*;
        match self {
            East | West => 0,
            North | South => 1,
        }
    }

    fn opposite_axis(self) -> usize {
        use Direction::*;
        match self {
            East | West => 1,
            North | South => 0,
        }
    }

    fn create_adjascent(point: Point2<TileN>) -> Vec<Option<Point2<TileN>>> {
        vec![
            point.x.checked_sub(1).map(|x| Point2::new(x, point.y)),
            point.y.checked_sub(1).map(|y| Point2::new(point.x, y)),
            Some(Point2::new(point.x + 1, point.y)),
            Some(Point2::new(point.x, point.y + 1)),
        ]
    }
}

type TileId = Id;
#[derive(Debug)]
struct Tilemap {
    tiles: HashMap<TileId, Tile>,
    level_info: Vec<LevelInfo>,
    current_level: Level,
    current_level_number: usize,
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum ObjectType {
    Player,
    Platform(RailId),
    Bullet(TileInstanceId, BulletId),
    Tile(TileId, TileInstanceId),
}

fn retrieve_user_data(collider: &Collider<N, DefaultBodyHandle>) -> ObjectType {
    *collider
        .user_data()
        .expect("Tile has no user_data.")
        .downcast_ref::<ObjectType>()
        .expect("user_data has an invalid type.")
}

impl Tilemap {
    pub fn new(tilemap: &mut tiled::Map) -> Self {
        let first_gid = tilemap.tilesets[0].first_gid;

        let mut tiles = tilemap
            .tilesets
            .remove(0)
            .tiles
            .into_iter()
            .filter_map(|tile| {
                let tile_type = tile.tile_type.as_ref().map(|tile_type| {
                    use TileType::*;
                    match tile_type.as_str() {
                        "Entrance" => Entrance,
                        "Exit" => Exit,
                        "Spikes" => Spikes,
                        "Wall" => Wall,
                        "Gun" => Gun,
                        "Rail" => Rail,
                        "Gravity" => Gravity,
                        tile_type => panic!("Unknown tile type: {}", tile_type),
                    }
                });
                tile_type.map(|tile_type| {
                    (
                        tile.id,
                        Tile {
                            type_: tile_type,
                            info: tile,
                        },
                    )
                })
            })
            .collect::<HashMap<TileId, Tile>>();

        if tiles.contains_key(&ID_FOR_NO_TILE) {
            panic!("ID_FOR_NO_TILE is not unique.")
        }

        tiles.insert(
            ID_FOR_NO_TILE,
            Tile {
                type_: TileType::None,
                info: tiled::Tile {
                    id: ID_FOR_NO_TILE,
                    images: Vec::new(),
                    properties: HashMap::new(),
                    objectgroup: None,
                    animation: None,
                    tile_type: None,
                    probability: 0.,
                },
            },
        );

        let tilevec = tilemap.layers[0]
            .tiles
            .iter()
            .flatten()
            .map(|layer_tile| apply_transformations(&layer_tile))
            .collect::<Vec<TileId>>();

        let tilematrix =
            na::DMatrix::from_vec(tilemap.width as TileN, tilemap.height as TileN, tilevec).map(
                |id| {
                    id.checked_sub(first_gid)
                        .map(|id| {
                            if tiles.contains_key(&strip_transformations(id)) {
                                Some(id)
                            } else {
                                None
                            }
                        })
                        .flatten()
                        .unwrap_or(ID_FOR_NO_TILE)
                },
            );

        let mut entrances = Self::find_tiles_from_matrix(&tilematrix, &tiles, TileType::Entrance);
        entrances.sort_by_key(|i| i.x);

        let level_info = entrances
            .iter()
            .map(|entrance| {
                let wallpaper = Self::start_build_wallpaper(&tilematrix, &tiles, entrance.clone());

                // we calculate the level size by how far left and right the wallpaper stretches
                let xs = &wallpaper.iter().map(|point| point.x).collect::<Vec<_>>();
                // prevent overflow caused by subtracting 1 from 0
                let x_bounds = (
                    xs.iter().min().unwrap().saturating_sub(1),
                    xs.iter().max().unwrap() + 2,
                );

                let matrix_slice = tilematrix
                    .slice(
                        (x_bounds.0, 0),
                        (x_bounds.1 - x_bounds.0, tilematrix.ncols()),
                    )
                    .into_owned();

                // offset x to account for the local coordinates of the slice
                let wallpaper = wallpaper
                    .iter()
                    .map(|point| Point2::new(point.x - x_bounds.0, point.y))
                    .collect();
                let entrance = Point2::new(entrance.x - x_bounds.0, entrance.y);

                LevelInfo {
                    tilematrix: matrix_slice,
                    wallpaper,
                    entrance,
                }
            })
            .collect();

        Tilemap {
            tiles,
            level_info,
            current_level: Level::default(),
            current_level_number: 0,
        }
    }

    pub fn get_tile_id_from_matrix(tilematrix: &DMatrix<TileId>, coords: Point2<TileN>) -> TileId {
        *tilematrix
            .get(point_to_tuple(coords))
            .expect("Matrix tile id query out of bounds.")
    }

    pub fn get_tile_from_tile_id(tiles: &HashMap<TileId, Tile>, id: TileId) -> &Tile {
        &tiles[&strip_transformations(id)]
    }

    pub fn get_tile_from_matrix<'a>(
        tilematrix: &DMatrix<TileId>,
        tiles: &'a HashMap<TileId, Tile>,
        coords: Point2<TileN>,
    ) -> &'a Tile {
        Self::get_tile_from_tile_id(tiles, Self::get_tile_id_from_matrix(tilematrix, coords))
    }

    fn find_tiles_from_matrix(
        tilematrix: &DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        tile_type: TileType,
    ) -> Vec<Point2<TileN>> {
        tilematrix
            .iter()
            .enumerate()
            .filter_map(|(i, id)| {
                if Self::get_tile_from_tile_id(tiles, *id).type_ == tile_type {
                    Some(tuple_to_point(tilematrix.vector_to_matrix_index(i)))
                } else {
                    None
                }
            })
            .collect()
    }

    fn start_build_wallpaper(
        tilematrix: &na::base::DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        coords: Point2<usize>,
    ) -> Vec<Point2<usize>> {
        let mut wallpaper = Vec::new();
        Self::build_wallpaper(tilematrix, tiles, coords, &mut wallpaper);
        wallpaper
    }

    fn build_wallpaper(
        tilematrix: &na::base::DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        coords: Point2<usize>,
        wallpaper: &mut Vec<Point2<usize>>,
    ) {
        for new_coords in Direction::create_adjascent(coords).iter().flatten() {
            let tile_type =
                Self::get_tile_from_matrix(tilematrix, tiles, (*new_coords).clone()).type_;
            if !wallpaper.contains(new_coords) && tile_type != TileType::Wall {
                wallpaper.push(new_coords.clone());
                // Still place a wallpaper behind all non-Wall wall blocks, since some of them
                // have open spaces that the wallpaper can shine through
                if !WALL_TILE_TYPES.contains(&tile_type) {
                    Tilemap::build_wallpaper(tilematrix, tiles, new_coords.clone(), wallpaper);
                }
            }
        }
    }

    pub fn init_next_level(&mut self, physics: &mut Physics, increment: bool) {
        // the first init_next_level call should not increment
        if increment {
            self.current_level_number += 1;
        }

        physics.ticks = 0;

        match self.level_info.get(self.current_level_number) {
            Some(level_info) => {
                self.current_level = Level::new(&level_info.tilematrix, &self.tiles, physics);
            }
            None => panic!("End of game!"),
        }
    }

    pub fn current_level_info(&self) -> &LevelInfo {
        &self.level_info[self.current_level_number]
    }

    pub fn update(&mut self, physics: &mut Physics) {
        self.current_level.update(physics);
    }

    pub fn draw(
        &self,
        ctx: &mut Context,
        physics: &Physics,
        tilesheet: &Image,
        spritesheet: &Image,
    ) -> GameResult {
        for coords in self.current_level_info().wallpaper.iter() {
            graphics::draw(
                ctx,
                tilesheet,
                DrawParam::new()
                    .src(Rect::new(
                        3. * (Tile::SIZE / IMAGE_WIDTH),
                        0.,
                        Tile::SIZE / IMAGE_WIDTH,
                        Tile::SIZE / IMAGE_HEIGHT,
                    ))
                    .offset(point_to_old(Point2::new(0.5, 0.5)))
                    .dest(point_to_old(Tile::point_to_real(coords.clone()))),
            )?;
        }

        self.current_level
            .draw(ctx, physics, tilesheet, spritesheet)?;

        Ok(())
    }
}

struct Physics {
    mechanical_world: world::DefaultMechanicalWorld<N>,
    geometrical_world: world::DefaultGeometricalWorld<N>,
    body_set: object::DefaultBodySet<N>,
    collider_set: object::DefaultColliderSet<N>,
    joint_constraint_set: nphysics2d::joint::DefaultJointConstraintSet<N>,
    force_generator_set: nphysics2d::force_generator::DefaultForceGeneratorSet<N>,
    ground_handle: DefaultBodyHandle,
    ticks: usize,
    gravity_dir: Direction,
}

impl Physics {
    const GRAVITY: N = 400.;

    pub fn new() -> Self {
        let geometrical_world = world::DefaultGeometricalWorld::new();
        let gravity_dir = Direction::South;
        let gravity = gravity_dir.to_vector() * Self::GRAVITY;
        let mut mechanical_world = world::DefaultMechanicalWorld::new(gravity);
        mechanical_world
            .solver
            .set_contact_model(Box::new(nphysics2d::solver::SignoriniModel::new()));
        let mut body_set = object::DefaultBodySet::new();
        let collider_set = object::DefaultColliderSet::new();

        let joint_constraint_set = nphysics2d::joint::DefaultJointConstraintSet::new();
        let force_generator_set = nphysics2d::force_generator::DefaultForceGeneratorSet::new();

        let ground_handle = body_set.insert(object::Ground::new());

        Self {
            geometrical_world,
            mechanical_world,
            body_set,
            collider_set,
            joint_constraint_set,
            force_generator_set,
            ground_handle,
            gravity_dir,
            ticks: 0,
        }
    }

    pub fn step(&mut self) {
        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.body_set,
            &mut self.collider_set,
            &mut self.joint_constraint_set,
            &mut self.force_generator_set,
        );
    }

    pub fn rigid_body(&self, handle: DefaultBodyHandle) -> &RigidBody<N> {
        self.body_set
            .rigid_body(handle)
            .expect("No rigid body found for handle.")
    }

    pub fn rigid_body_mut(&mut self, handle: DefaultBodyHandle) -> &mut RigidBody<N> {
        self.body_set
            .rigid_body_mut(handle)
            .expect("No rigid body found for handle.")
    }

    pub fn collider(&self, handle: DefaultColliderHandle) -> &Collider<N, DefaultBodyHandle> {
        self.collider_set
            .get(handle)
            .expect("No collider found for handle.")
    }

    pub fn retrieve_user_datas(
        &self,
        coll1_handle: DefaultColliderHandle,
        coll2_handle: DefaultColliderHandle,
    ) -> (ObjectType, ObjectType) {
        (
            retrieve_user_data(self.collider(coll1_handle)),
            retrieve_user_data(self.collider(coll2_handle)),
        )
    }

    pub fn collisions(
        &self,
        handle: DefaultColliderHandle,
    ) -> Vec<(
        (ObjectType, ObjectType),
        ncollide2d::query::ContactManifold<N>,
    )> {
        self.geometrical_world
            .contacts_with(&self.collider_set, handle, true)
            .into_iter()
            .flatten()
            .map(|(handle1, _, handle2, _, _, manifold)| {
                (self.retrieve_user_datas(handle1, handle2), manifold.clone())
            })
            .collect()
    }

    pub fn collision_events(
        &self,
    ) -> Vec<(
        (ObjectType, ObjectType),
        ncollide2d::query::ContactManifold<N>,
    )> {
        self.geometrical_world
            .contact_events()
            .iter()
            .filter_map(|event| match event {
                ncollide2d::pipeline::narrow_phase::ContactEvent::Started(handle1, handle2) => self
                    .geometrical_world
                    .contact_pair(&self.collider_set, *handle1, *handle2, true)
                    .map(|(_, _, _, _, _, manifold)| {
                        (
                            self.retrieve_user_datas(*handle1, *handle2),
                            manifold.clone(),
                        )
                    }),
                _ => None,
            })
            .collect::<Vec<_>>()
            .clone()
    }

    pub fn proximity_events(
        &self,
    ) -> Vec<((ObjectType, ObjectType), ncollide2d::query::Proximity)> {
        self.geometrical_world
            .proximity_events()
            .iter()
            .map(|proximity_event| {
                (
                    self.retrieve_user_datas(proximity_event.collider1, proximity_event.collider2),
                    proximity_event.new_status,
                )
            })
            .collect::<Vec<_>>()
            .clone()
    }

    pub fn build_body(&mut self, body_desc: RigidBodyDesc<N>) -> DefaultBodyHandle {
        let body = body_desc.build();
        self.body_set.insert(body)
    }

    pub fn build_collider(
        &mut self,
        collider_desc: ColliderDesc<N>,
        body_handle: DefaultBodyHandle,
        ccd_enabled: bool,
    ) -> DefaultColliderHandle {
        let collider = collider_desc
            .material(material::MaterialHandle::new(material::BasicMaterial::new(
                0., 0.,
            )))
            .set_ccd_enabled(ccd_enabled)
            .build(object::BodyPartHandle(body_handle, 0));
        self.collider_set.insert(collider)
    }

    #[allow(dead_code)]
    pub fn draw_colliders(&self, ctx: &mut Context) -> GameResult {
        for (_, collider) in self.collider_set.iter() {
            let shape = collider.shape().aabb(collider.position());
            draw_rect(
                ctx,
                Rect::new(
                    shape.mins().x,
                    shape.mins().y,
                    shape.extents().x,
                    shape.extents().y,
                ),
                graphics::BLACK,
            )?;
        }
        Ok(())
    }

    pub fn set_gravity_dir(&mut self, dir: Direction, player: &mut Player) {
        self.gravity_dir = dir;
        self.mechanical_world.gravity = dir.to_vector() * Self::GRAVITY;
        // clearing the velocity is necessary to prevent weird small-jump bug
        player
            .entity
            .rigid_body_mut(self)
            .set_velocity(Velocity2::linear(0., 0.));
    }

    pub fn on_ground(&self, manifold: &ncollide2d::query::ContactManifold<N>) -> bool {
        manifold.contacts().any(|contact| {
            let contact_normal_direction = match self.gravity_dir {
                Direction::South | Direction::East => 1.,
                Direction::North | Direction::West => -1.,
            };
            contact.contact.normal[self.gravity_dir.axis()].round() * contact_normal_direction > 0.
                && contact.contact.normal[self.gravity_dir.opposite_axis()].round() == 0.
        })
    }
}

struct MyGame {
    player: Player,
    physics: Physics,
    tilemap: Tilemap,
    tilesheet_image: Image,
    spritesheet_image: Image,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        graphics::set_default_filter(ctx, graphics::FilterMode::Nearest);

        let mut tilemap = {
            let file = ggez::filesystem::open(ctx, "/tilemap.tmx").unwrap();
            let mut tilemap = tiled::parse(file).unwrap();
            Tilemap::new(&mut tilemap)
        };

        let mut physics = Physics::new();

        let tilesheet_image = Image::new(ctx, "/tilesheet.png").unwrap();
        let spritesheet_image = Image::new(ctx, "/spritesheet.png").unwrap();

        tilemap.init_next_level(&mut physics, false);

        let player = Player::new(&mut physics, tilemap.current_level_info().entrance.clone());

        physics.step();

        MyGame {
            player,
            tilemap,
            physics,
            tilesheet_image,
            spritesheet_image,
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ggez::timer::check_update_time(ctx, 60) {
            self.player.update(ctx, &mut self.physics, &self.tilemap);
            self.tilemap.update(&mut self.physics);

            for (user_datas, _) in self.physics.collision_events() {
                // println!("Collision: {:?}", user_datas);

                match user_datas {
                    (ObjectType::Bullet(gun_id, bullet_id), other)
                    | (other, ObjectType::Bullet(gun_id, bullet_id)) => match other {
                        ObjectType::Bullet(_, _) | ObjectType::Player => (),
                        _ => {
                            if let Some(TileData::GunData(ref mut gun_tile)) = &mut self
                                .tilemap
                                .current_level
                                .tiles
                                .get_mut(&gun_id)
                                .map(|tile| &mut tile.extra_data)
                            {
                                gun_tile.remove_bullet(&mut self.physics, bullet_id);
                            } else {
                                panic!("user_data points to nonexistent gun.")
                            }
                        }
                    },
                    _ => (),
                }
            }

            for (user_datas, _) in self.physics.proximity_events() {
                // println!("Proximity: {:?}", user_datas);

                match user_datas {
                    (ObjectType::Player, other) | (other, ObjectType::Player) => match other {
                        ObjectType::Tile(tile_id, _) => {
                            let collided_tile =
                                Tilemap::get_tile_from_tile_id(&self.tilemap.tiles, tile_id);
                            match collided_tile.type_ {
                                TileType::Spikes => {
                                    self.player.reset(
                                        &mut self.physics,
                                        self.tilemap.current_level_info().entrance.clone(),
                                    );
                                    self.physics
                                        .set_gravity_dir(Direction::South, &mut self.player);
                                }
                                TileType::Exit => {
                                    self.physics = Physics::new();
                                    self.tilemap.init_next_level(&mut self.physics, true);
                                    self.player = Player::new(
                                        &mut self.physics,
                                        self.tilemap.current_level_info().entrance.clone(),
                                    );
                                    self.physics.step();
                                    // return to prevent incrementing physics.ticks
                                    return Ok(());
                                }
                                _ => (),
                            }
                        }
                        _ => (),
                    },
                    _ => (),
                }
            }

            self.physics.step();
            self.physics.ticks += 1;
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        graphics::set_transform(
            ctx,
            DrawParam::new()
                .scale(vector_to_old(Vector2::new(SCALE, SCALE)))
                .dest(point_to_old(add(
                    -self.player.entity.position(&self.physics) * SCALE,
                    Point2::new(WINDOW_SIZE / 2., WINDOW_SIZE / 2.),
                )))
                .to_matrix(),
        );
        graphics::apply_transformations(ctx)?;

        // println!("FPS: {}", ggez::timer::fps(ctx));
        graphics::clear(ctx, Color::from(BACKGROUND_COLOR));
        self.tilemap.draw(
            ctx,
            &self.physics,
            &self.tilesheet_image,
            &self.spritesheet_image,
        )?;
        // self.physics.draw_colliders(ctx)?;
        self.player
            .entity
            .draw(ctx, &self.physics, &self.spritesheet_image)?;
        graphics::present(ctx)
    }
}

const SCALE: N = 2.;
const WINDOW_SIZE: N = 300.;

fn main() {
    let resource_dir = std::path::PathBuf::from("./assets");

    let (mut ctx, mut event_loop) = ContextBuilder::new("ascension-rust", "Anton")
        .add_resource_path(resource_dir)
        .window_setup(
            ggez::conf::WindowSetup::default()
                .title("Ascension")
                .samples(ggez::conf::NumSamples::Zero),
        )
        .window_mode(ggez::conf::WindowMode::default().dimensions(WINDOW_SIZE, WINDOW_SIZE))
        .build()
        .expect("Could not create lgez context.");

    let mut my_game = MyGame::new(&mut ctx);

    match ggez::event::run(&mut ctx, &mut event_loop, &mut my_game) {
        Ok(_) => println!("Exited cleanly."),
        Err(e) => println!("Error occured: {}", e),
    }
}
