extern crate nalgebra as na;

use std::collections::HashMap;
use std::default::Default;

use ggez::event::{EventHandler, KeyCode};
use ggez::graphics::Color;
use ggez::graphics::{DrawParam, Image, Rect};
use ggez::input::{keyboard, mouse};
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

use serde::{Deserialize, Serialize};
use std::io::Read;

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

fn create_sprite_draw_param(pos: Point2<N>, size: Point2<N>, pixel_size: N) -> DrawParam {
    DrawParam::new()
        .src(Rect::new(
            pos.x * (pixel_size / IMAGE_WIDTH),
            pos.y * (pixel_size / IMAGE_HEIGHT),
            size.x / IMAGE_WIDTH,
            size.y / IMAGE_HEIGHT,
        ))
        .offset(point_to_old(Point2::new(0.5, 0.5)))
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

#[derive(PartialEq, Debug, Clone, Copy)]
enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    fn from_id(tile_id: TileId) -> Self {
        use Direction::*;
        let TileId(id) = tile_id;
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

    fn create_adjacent(point: Point2<TileN>) -> Vec<Option<Point2<TileN>>> {
        vec![
            point.x.checked_sub(1).map(|x| Point2::new(x, point.y)),
            point.y.checked_sub(1).map(|y| Point2::new(point.x, y)),
            Some(Point2::new(point.x + 1, point.y)),
            Some(Point2::new(point.x, point.y + 1)),
        ]
    }
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
        // because we don't care about friction
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
        let draw_param = create_sprite_draw_param(sprite_pos.clone(), size.clone(), 1.);

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

        Self {
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
    const JUMP_POWER: N = 204.;
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

    pub fn update(
        &mut self,
        ctx: &Context,
        physics: &mut Physics,
        rails: &HashMap<RailId, Rail>,
    ) -> bool {
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
                    other @ ObjectType::Platform(_) | other @ ObjectType::Tile(_, _) => {
                        if physics.on_ground(&manifold) {
                            on_ground = true;
                        }

                        match other {
                            ObjectType::Platform(rail_id) => {
                                if on_ground {
                                    let mut platform_velocity =
                                        rails[&rail_id].platform.velocity(physics);

                                    let player_velocity = self.entity.velocity(physics);
                                    platform_velocity[physics.gravity_dir.opposite_axis()] +=
                                        player_velocity[physics.gravity_dir.opposite_axis()];

                                    self.entity.set_velocity(physics, platform_velocity);
                                }
                            }
                            _ => (),
                        }
                    }
                    ObjectType::Bullet(_, _) => return true,
                    _ => (),
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

        false
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
    Rail(bool),
    Gravity,
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

#[derive(Debug)]
enum TileData {
    GunData(GunTile),
    GravityData(bool, DrawParam),
    None,
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
                    .user_data(ObjectType::Tile(TileId(tile.info.id), tile_instance_id)),
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
                    .user_data(ObjectType::Tile(TileId(tile.info.id), tile_instance_id)),
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

        let draw_param = create_sprite_draw_param(
            Point2::new(src_x as N, src_y as N),
            Point2::new(Tile::SIZE, Tile::SIZE),
            Tile::SIZE,
        )
        .rotation(rotation)
        .dest(point_to_old(real_point.clone()));

        let extra_data = match tile.type_ {
            TileType::Gun => TileData::GunData(GunTile::new()),
            TileType::Gravity => TileData::GravityData(
                false,
                create_sprite_draw_param(
                    Point2::new((src_x + 1) as N, src_y as N),
                    Point2::new(Tile::SIZE, Tile::SIZE),
                    Tile::SIZE,
                )
                .rotation(rotation)
                .dest(point_to_old(real_point.clone())),
            ),
            _ => TileData::None,
        };

        Self {
            id: TileInstanceId(get_id()),
            tile_id: TileId(tile.info.id),
            draw_param,
            direction,
            coords: coords.clone(),
            extra_data,
        }
    }
}

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash)]
struct BulletId(Id);

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
        let bullet_id = BulletId(get_id());
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

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash)]
struct RailId(Id);

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

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash)]
struct TileInstanceId(Id);

struct Level {
    wallpaper_draw_param: DrawParam,
    tile_instances: HashMap<TileInstanceId, TileInstance>,
    rails: HashMap<RailId, Rail>,
    player: Player,
    physics: Physics,
}

impl Level {
    const WALLPAPER_SPRITE_POS: (N, N) = (3., 0.);

    pub fn new(tiles: &HashMap<TileId, Tile>, level_info: &LevelInfo) -> Self {
        let mut physics = Physics::new();

        let tile_instances: HashMap<TileInstanceId, TileInstance> = level_info
            .tilematrix
            .iter()
            .enumerate()
            .filter_map(|(i, tile_id)| {
                let tile = LevelInfo::get_tile_from_tile_id(tiles, *tile_id);
                let tile_instance_id = TileInstanceId(get_id());
                if tile.type_ != TileType::None {
                    Some((
                        tile_instance_id,
                        TileInstance::new(
                            &mut physics,
                            tuple_to_point(level_info.tilematrix.vector_to_matrix_index(i)),
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
            if LevelInfo::get_tile_from_tile_id(tiles, tile_instance.tile_id).type_
                == TileType::Rail(true)
                && !rail_set
                    .iter()
                    .flatten()
                    .collect::<Vec<_>>()
                    .contains(&&tile_instance.coords)
            {
                let mut rail = Vec::new();
                Self::build_rail(
                    &level_info.tilematrix,
                    tiles,
                    &mut rail,
                    tile_instance.coords.clone(),
                );
                rail_set.push(rail);
            }
        }

        let rails = rail_set
            .into_iter()
            .map(|rail| {
                let rail_id = RailId(get_id());
                (rail_id, Rail::new(&mut physics, rail, rail_id))
            })
            .collect();

        // player needs to be created after the rails, otherwise collisions will register incorrectly
        let player = Player::new(&mut physics, level_info.entrance.clone());

        physics.step();
        physics.ticks = 0;

        Self {
            physics,
            player,
            tile_instances,
            rails,
            wallpaper_draw_param: create_sprite_draw_param(
                tuple_to_point(Self::WALLPAPER_SPRITE_POS),
                Point2::new(Tile::SIZE, Tile::SIZE),
                Tile::SIZE,
            ),
        }
    }

    fn build_rail(
        tilematrix: &DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        rail: &mut Vec<Point2<TileN>>,
        coords: Point2<TileN>,
    ) {
        rail.push(coords.clone());

        // if we are not currently at a rail endpoint (except if it's the first rail)
        if rail.len() == 1
            || LevelInfo::get_tile_from_matrix(tilematrix, tiles, coords.clone()).type_
                != TileType::Rail(true)
        {
            // we give preference to the non-endpoint rail so that we don't choose a neighboring endpoint
            // over an actual part of the rail
            let mut options = Direction::create_adjacent(coords)
                .into_iter()
                .flatten()
                .filter_map(|coords: Point2<TileN>| {
                    let tile = LevelInfo::get_tile_from_matrix(tilematrix, tiles, coords.clone());
                    if !rail.contains(&coords) {
                        match tile.type_ {
                            TileType::Rail(true) => Some((1, coords.clone())),
                            TileType::Rail(false) => Some((0, coords.clone())),
                            _ => None,
                        }
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            options.sort_by_key(|(i, _)| *i);
            options.get(0).map(|(_, new_coords)| {
                Self::build_rail(tilematrix, tiles, rail, new_coords.clone())
            });
        }
    }

    pub fn update(
        &mut self,
        ctx: &Context,
        tiles: &HashMap<TileId, Tile>,
        level_info: &LevelInfo,
    ) -> bool {
        if self.player.update(ctx, &mut self.physics, &self.rails) {
            self.reset_level(level_info);
        }

        for (instance_id, tile) in self.tile_instances.iter_mut() {
            match &mut tile.extra_data {
                TileData::GunData(ref mut gun_tile) => {
                    if self.physics.ticks % GunTile::SHOOTING_FREQUENCY == 0 {
                        gun_tile.shoot(
                            &mut self.physics,
                            tile.coords.clone(),
                            tile.direction,
                            *instance_id,
                        );
                    }
                }
                _ => (),
            }
        }

        for rail in self.rails.values_mut() {
            rail.update(&mut self.physics);
        }

        for (user_datas, _) in self.physics.collision_events() {
            // println!("Collision: {:?}", user_datas);

            match user_datas {
                (ObjectType::Bullet(gun_id, bullet_id), other)
                | (other, ObjectType::Bullet(gun_id, bullet_id)) => match other {
                    ObjectType::Platform(_) | ObjectType::Tile(_, _) => {
                        if let Some(TileData::GunData(ref mut gun_tile)) = &mut self
                            .tile_instances
                            .get_mut(&gun_id)
                            .map(|tile| &mut tile.extra_data)
                        {
                            gun_tile.remove_bullet(&mut self.physics, bullet_id);
                        } else {
                            panic!("user_data points to nonexistent gun.")
                        }
                    }
                    _ => (),
                },
                (ObjectType::Player, other) | (other, ObjectType::Player) => match other {
                    ObjectType::Tile(_, tile_instance_id) => {
                        let collided_tile = LevelInfo::get_tile_from_tile_id(
                            tiles,
                            self.tile_instances[&tile_instance_id].tile_id,
                        );
                        match collided_tile.type_ {
                            TileType::Gravity => {
                                self.turn_off_gravity_machines();
                                let tile_instance =
                                    self.tile_instances.get_mut(&tile_instance_id).unwrap();

                                if let TileData::GravityData(ref mut is_on, _) =
                                    &mut tile_instance.extra_data
                                {
                                    *is_on = true;
                                }
                                let gravity_dir = tile_instance.direction.opposite();
                                if gravity_dir != self.physics.gravity_dir {
                                    self.physics.set_gravity_dir(gravity_dir, &mut self.player);
                                }
                            }
                            _ => (),
                        }
                    }
                    _ => (),
                },
                _ => (),
            }
        }

        for (user_datas, _) in self.physics.proximity_events() {
            // println!("Proximity: {:?}", user_datas);

            match user_datas {
                (ObjectType::Player, other) | (other, ObjectType::Player) => match other {
                    ObjectType::Tile(tile_id, _) => {
                        let collided_tile = LevelInfo::get_tile_from_tile_id(tiles, tile_id);
                        match collided_tile.type_ {
                            TileType::Spikes => self.reset_level(level_info),
                            TileType::Exit => return true,
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

        false
    }

    fn reset_level(&mut self, level_info: &LevelInfo) {
        self.turn_off_gravity_machines();
        self.player.entity.set_position(
            &mut self.physics,
            Tile::point_to_real(level_info.entrance.clone()),
        );
        self.physics
            .set_gravity_dir(Direction::South, &mut self.player)
    }

    fn turn_off_gravity_machines(&mut self) {
        for tile in self.tile_instances.values_mut() {
            if let TileData::GravityData(ref mut is_on, _) = &mut tile.extra_data {
                *is_on = false;
            }
        }
    }

    pub fn draw(
        &self,
        ctx: &mut Context,
        spritesheet: &Image,
        tilesheet: &Image,
        level_info: &LevelInfo,
    ) -> GameResult {
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

        for coords in level_info.wallpaper.iter() {
            graphics::draw(
                ctx,
                tilesheet,
                self.wallpaper_draw_param
                    .dest(point_to_old(Tile::point_to_real(coords.clone()))),
            )?;
        }

        // self.physics.draw_colliders(ctx)?;

        for tile in self.tile_instances.values() {
            match &tile.extra_data {
                TileData::GravityData(turned_on, turned_on_draw_param) => {
                    let draw_param = if *turned_on {
                        *turned_on_draw_param
                    } else {
                        tile.draw_param
                    };
                    graphics::draw(ctx, tilesheet, draw_param)?;
                }
                _ => graphics::draw(ctx, tilesheet, tile.draw_param)?,
            }
        }

        for tile in self.tile_instances.values() {
            match &tile.extra_data {
                TileData::GunData(gun_tile) => gun_tile.draw(ctx, &self.physics, spritesheet)?,
                _ => (),
            }
        }

        for rail in self.rails.values() {
            rail.draw(ctx, &self.physics, spritesheet)?;
        }

        self.player.entity.draw(ctx, &self.physics, spritesheet)?;

        Ok(())
    }
}

struct World {
    current_level: Level,
    current_level_number: usize,
}

impl World {
    pub fn new(
        tiles: &HashMap<TileId, Tile>,
        level_infos: &Vec<LevelInfo>,
        level_number: usize,
    ) -> Self {
        Self {
            current_level: Level::new(tiles, &level_infos[level_number]),
            current_level_number: level_number,
        }
    }

    pub fn update(
        &mut self,
        ctx: &Context,
        tiles: &HashMap<TileId, Tile>,
        level_infos: &Vec<LevelInfo>,
    ) -> bool {
        self.current_level
            .update(ctx, tiles, &level_infos[self.current_level_number])
    }

    pub fn init_next_level(
        &mut self,
        tiles: &HashMap<TileId, Tile>,
        level_infos: &Vec<LevelInfo>,
    ) -> bool {
        self.current_level_number += 1;

        match level_infos.get(self.current_level_number) {
            Some(level_info) => {
                self.current_level = Level::new(tiles, level_info);
                false
            }
            None => true,
        }
    }

    pub fn draw(
        &self,
        ctx: &mut Context,
        spritesheet: &Image,
        tilesheet: &Image,
        level_infos: &Vec<LevelInfo>,
    ) -> GameResult {
        self.current_level.draw(
            ctx,
            spritesheet,
            tilesheet,
            &level_infos[self.current_level_number],
        )?;

        Ok(())
    }
}

#[derive(PartialEq, Eq, Debug, Clone, Copy, Hash)]
struct TileId(Id);

const TILE_ID_FOR_NO_TILE: TileId = TileId(0b000100000000);
const FLIP_H: Id = 0b100000000000;
const FLIP_V: Id = 0b010000000000;
const FLIP_D: Id = 0b001000000000;

fn apply_transformations(layer_tile: &tiled::LayerTile) -> Id {
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
fn strip_transformations(id: Id) -> Id {
    id & !FLIP_H & !FLIP_V & !FLIP_D
}

#[derive(Debug)]
struct LevelInfo {
    tilematrix: DMatrix<TileId>,
    wallpaper: Vec<Point2<TileN>>,
    entrance: Point2<TileN>,
}

impl LevelInfo {
    pub fn new(
        tiles: &HashMap<TileId, Tile>,
        tilemap_layer: &tiled::Layer,
        first_gid: Id,
        tilemap_size: (TileN, TileN),
    ) -> Vec<Self> {
        let tilevec = tilemap_layer
            .tiles
            .iter()
            .flatten()
            .map(|layer_tile| apply_transformations(&layer_tile))
            .collect::<Vec<Id>>();

        let tilematrix = na::DMatrix::from_vec(tilemap_size.0, tilemap_size.1, tilevec).map(|id| {
            id.checked_sub(first_gid)
                .map(|id| {
                    if tiles.contains_key(&TileId(strip_transformations(id))) {
                        Some(TileId(id))
                    } else {
                        None
                    }
                })
                .flatten()
                .unwrap_or(TILE_ID_FOR_NO_TILE)
        });

        let mut entrances = Self::find_tiles_from_matrix(&tilematrix, &tiles, TileType::Entrance);
        entrances.sort_by_key(|i| i.x);

        entrances
            .iter()
            .map(|entrance| {
                let mut wallpaper = Vec::new();
                Self::build_wallpaper(&tilematrix, &tiles, &mut wallpaper, entrance.clone());

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

                Self {
                    tilematrix: matrix_slice,
                    wallpaper,
                    entrance,
                }
            })
            .collect()
    }

    fn build_wallpaper(
        tilematrix: &DMatrix<TileId>,
        tiles: &HashMap<TileId, Tile>,
        wallpaper: &mut Vec<Point2<TileN>>,
        coords: Point2<TileN>,
    ) {
        for new_coords in Direction::create_adjacent(coords).iter().flatten() {
            let tile_type =
                Self::get_tile_from_matrix(&tilematrix, &tiles, (*new_coords).clone()).type_;
            if !wallpaper.contains(new_coords) && tile_type != TileType::Wall {
                wallpaper.push(new_coords.clone());
                // Still place a wallpaper behind all non-Wall wall blocks, since some of them
                // have open spaces that the wallpaper can shine through
                if !WALL_TILE_TYPES.contains(&tile_type) {
                    Self::build_wallpaper(tilematrix, tiles, wallpaper, new_coords.clone());
                }
            }
        }
    }

    pub fn get_tile_id_from_matrix(tilematrix: &DMatrix<TileId>, coords: Point2<TileN>) -> TileId {
        *tilematrix
            .get(point_to_tuple(coords))
            .expect("Matrix tile id query out of bounds.")
    }

    pub fn get_tile_from_tile_id(tiles: &HashMap<TileId, Tile>, tile_id: TileId) -> &Tile {
        let TileId(id) = tile_id;
        &tiles[&TileId(strip_transformations(id))]
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
}

struct LevelSelect {
    box_draw_param: DrawParam,
    locked_box_draw_param: DrawParam,
    title_draw_param: DrawParam,
    boxes: Vec<Rect>,
}

impl LevelSelect {
    const BOX_SPRITE_POS: (N, N) = (0., 48.);
    const LOCKED_BOX_SPRITE_POS: (N, N) = (10., 48.);
    const BOX_SPRITE_SIZE: (N, N) = (10., 10.);
    const BOX_MARGIN: N = 2.;

    const TITLE_SPRITE_POS: (N, N) = (0., 17.);
    const TITLE_SPRITE_SIZE: (N, N) = (81., 31.);

    const SCALE: N = SCALE * 2.;

    pub fn new(levels_info: &Vec<LevelInfo>) -> Self {
        let unscaled_window_size = WINDOW_SIZE / Self::SCALE;
        let box_offset = Point2::new(
            (unscaled_window_size
                - (levels_info.len() as N) * (Self::BOX_SPRITE_SIZE.0 + Self::BOX_MARGIN))
                / 2.,
            unscaled_window_size / 2. + Self::BOX_SPRITE_SIZE.1,
        );

        Self {
            boxes: levels_info
                .iter()
                .enumerate()
                .map(|(i, _)| {
                    Rect::new(
                        (i as N) * (Self::BOX_SPRITE_SIZE.0 + Self::BOX_MARGIN) + box_offset.x,
                        box_offset.y,
                        Self::BOX_SPRITE_SIZE.0,
                        Self::BOX_SPRITE_SIZE.1,
                    )
                })
                .collect(),
            box_draw_param: create_sprite_draw_param(
                tuple_to_point(Self::BOX_SPRITE_POS),
                tuple_to_point(Self::BOX_SPRITE_SIZE),
                1.,
            )
            .offset(ggez::nalgebra::Point2::origin()),
            locked_box_draw_param: create_sprite_draw_param(
                tuple_to_point(Self::LOCKED_BOX_SPRITE_POS),
                tuple_to_point(Self::BOX_SPRITE_SIZE),
                1.,
            )
            .offset(ggez::nalgebra::Point2::origin()),
            title_draw_param: create_sprite_draw_param(
                tuple_to_point(Self::TITLE_SPRITE_POS),
                tuple_to_point(Self::TITLE_SPRITE_SIZE),
                1.,
            )
            .dest(point_to_old(Point2::new(
                unscaled_window_size / 2.,
                unscaled_window_size / 4.,
            ))),
        }
    }

    pub fn update(&self, ctx: &Context) -> Option<usize> {
        if mouse::button_pressed(ctx, mouse::MouseButton::Left) {
            let mouse_pos: ggez::nalgebra::Point2<N> = mouse::position(ctx).into();
            let mouse_pos = mouse_pos / Self::SCALE;
            self.boxes.iter().position(|box_| box_.contains(mouse_pos))
        } else {
            None
        }
    }

    pub fn draw(
        &self,
        ctx: &mut Context,
        spritesheet: &Image,
        unlocked_levels: &Vec<bool>,
    ) -> GameResult {
        graphics::set_transform(
            ctx,
            DrawParam::new()
                .scale(vector_to_old(Vector2::new(Self::SCALE, Self::SCALE)))
                .to_matrix(),
        );
        graphics::apply_transformations(ctx)?;

        graphics::draw(ctx, spritesheet, self.title_draw_param)?;
        for (i, box_) in self.boxes.iter().enumerate() {
            let box_draw_param = if unlocked_levels[i] {
                self.box_draw_param
            } else {
                self.locked_box_draw_param
            };
            graphics::draw(ctx, spritesheet, box_draw_param.dest(box_.point()))?;
        }

        Ok(())
    }
}

enum GameState {
    LevelSelect(LevelSelect),
    Playing(World),
}

#[derive(Serialize, Deserialize, Default)]
struct SavedData {
    unlocked: Vec<bool>,
}

struct Game {
    state: GameState,
    level_infos: Vec<LevelInfo>,
    saved_data: SavedData,
    tiles: HashMap<TileId, Tile>,
    tilesheet_image: Image,
    spritesheet_image: Image,
}

const SAVE_FILE: &str = "/data.json";

impl Game {
    pub fn new(ctx: &mut Context) -> Game {
        graphics::set_default_filter(ctx, graphics::FilterMode::Nearest);

        let tilemap_file = ggez::filesystem::open(ctx, "/tilemap.tmx").unwrap();
        let mut tilemap = tiled::parse(tilemap_file).unwrap();

        let tileset = tilemap.tilesets.remove(0);
        let first_gid = tileset.first_gid;
        let tiles = Game::build_tiles(tileset);
        let level_infos = LevelInfo::new(
            &tiles,
            &tilemap.layers[0],
            first_gid,
            (tilemap.width as TileN, tilemap.height as TileN),
        );

        let tilesheet_image = Image::new(ctx, "/tilesheet.png").unwrap();
        let spritesheet_image = Image::new(ctx, "/spritesheet.png").unwrap();

        let saved_data = ggez::filesystem::open(ctx, SAVE_FILE)
            .map(|mut file| {
                let mut contents = String::new();
                file.read_to_string(&mut contents).unwrap();
                serde_json::from_str(&contents).map(|res: SavedData|
                    // if the number of saved levels is not the same as the number of current levels
                    // invalidate the save
                    if res.unlocked.len() == level_infos.len() {
                        Some(res)
                    } else { None }
                ).unwrap_or(None)
            })
            .unwrap_or(None)
            .unwrap_or(SavedData {
                unlocked: {
                    let mut vec = vec![false; level_infos.len()];
                    vec[0] = true;
                    vec
                },
            });

        Self {
            state: GameState::LevelSelect(LevelSelect::new(&level_infos)),
            level_infos,
            tiles,
            tilesheet_image,
            spritesheet_image,
            saved_data,
        }
    }

    pub fn build_tiles(tileset: tiled::Tileset) -> HashMap<TileId, Tile> {
        let mut tiles = tileset
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
                        "Rail" => Rail(parse_property(&tile, "is_endpoint")),
                        "Gravity" => Gravity,
                        tile_type => panic!("Unknown tile type: {}", tile_type),
                    }
                });
                tile_type.map(|tile_type| {
                    (
                        TileId(tile.id),
                        Tile {
                            type_: tile_type,
                            info: tile,
                        },
                    )
                })
            })
            .collect::<HashMap<TileId, Tile>>();

        if tiles.contains_key(&TILE_ID_FOR_NO_TILE) {
            panic!("ID_FOR_NO_TILE is not unique.")
        }

        {
            let TileId(id) = TILE_ID_FOR_NO_TILE;
            tiles.insert(
                TILE_ID_FOR_NO_TILE,
                Tile {
                    type_: TileType::None,
                    info: tiled::Tile {
                        id,
                        images: Vec::new(),
                        properties: HashMap::new(),
                        objectgroup: None,
                        animation: None,
                        tile_type: None,
                        probability: 0.,
                    },
                },
            );
        };

        tiles
    }

    fn write_data(ctx: &mut Context, saved_data: &SavedData) {
        match ggez::filesystem::create(ctx, SAVE_FILE) {
            Ok(file) => serde_json::to_writer(file, &saved_data).unwrap(),
            Err(e) => println!("Could not write save file: {}", e),
        }
    }
}

impl EventHandler for Game {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        while ggez::timer::check_update_time(ctx, 60) {
            match &mut self.state {
                GameState::LevelSelect(ref mut level_select) => {
                    if let Some(level_number) = level_select.update(ctx) {
                        if self.saved_data.unlocked[level_number] {
                            self.state = GameState::Playing(World::new(
                                &self.tiles,
                                &self.level_infos,
                                level_number,
                            ));
                        }
                    }
                }
                GameState::Playing(ref mut world) => {
                    if world.update(ctx, &self.tiles, &self.level_infos) {
                        if world.init_next_level(&self.tiles, &self.level_infos) {
                            self.state =
                                GameState::LevelSelect(LevelSelect::new(&self.level_infos));
                        } else {
                            self.saved_data.unlocked[world.current_level_number] = true;
                            Self::write_data(ctx, &self.saved_data);
                        }
                    }
                }
            };
        }
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        graphics::clear(ctx, Color::from(BACKGROUND_COLOR));
        // println!("FPS: {}", ggez::timer::fps(ctx));
        match &mut self.state {
            GameState::LevelSelect(ref level_select) => {
                level_select.draw(ctx, &self.spritesheet_image, &self.saved_data.unlocked)?
            }
            GameState::Playing(ref world) => world.draw(
                ctx,
                &self.spritesheet_image,
                &self.tilesheet_image,
                &self.level_infos,
            )?,
        };
        graphics::present(ctx)
    }
}

const SCALE: N = 2.;
const WINDOW_SIZE: N = 600.;

fn main() {
    let mut cb = ContextBuilder::new("ascension-rust", "Anton")
        .window_setup(ggez::conf::WindowSetup::default().title("Ascension"))
        .window_mode(ggez::conf::WindowMode::default().dimensions(WINDOW_SIZE, WINDOW_SIZE));

    if let Ok(manifest_dir) = std::env::var("CARGO_MANIFEST_DIR") {
        let mut path = std::path::PathBuf::from(manifest_dir);
        path.push("resources");
        println!("Adding path {:?}", path);
        cb = cb.add_resource_path(path);
    }

    let (mut ctx, mut event_loop) = cb.build().expect("Could not create ggez context.");

    let mut my_game = Game::new(&mut ctx);

    match ggez::event::run(&mut ctx, &mut event_loop, &mut my_game) {
        Ok(_) => println!("Exited cleanly."),
        Err(e) => println!("Error occurred: {}", e),
    }
}
