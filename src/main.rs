extern crate nalgebra as na;

use ggez::event::{EventHandler, KeyCode};
use ggez::graphics::{DrawParam, Rect};
use ggez::input::keyboard;
use ggez::{graphics, Context, ContextBuilder, GameResult};

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::algebra::{Force2, ForceType};
use nphysics2d::object::{
    Body, ColliderDesc, DefaultBodyHandle, DefaultColliderHandle, RigidBody, RigidBodyDesc,
};
use nphysics2d::{material, object, world};

type N = f32;

const IMAGE_WIDTH: f32 = 250.;
const IMAGE_HEIGHT: f32 = 250.;

// for compatibility
fn point2(point: Point2<N>) -> ggez::nalgebra::Point2<N> {
    ggez::nalgebra::Point2::new(point.x, point.y)
}
fn vector2(vector: Vector2<N>) -> ggez::nalgebra::Vector2<N> {
    ggez::nalgebra::Vector2::new(vector.x, vector.y)
}

fn draw_point(ctx: &mut Context, point: Point2<N>, color: graphics::Color) {
    let circle = graphics::Mesh::new_circle(
        ctx,
        graphics::DrawMode::Fill(graphics::FillOptions::DEFAULT),
        point2(point),
        1.,
        0.,
        color,
    )
    .unwrap();
    graphics::draw(ctx, &circle, DrawParam::new());
}

struct Player {
    draw_param: DrawParam,
    body_handle: DefaultBodyHandle,
    collider_handle: DefaultColliderHandle,
    on_ground: bool,
}

impl Player {
    const X_POWER: N = 100.;
    const Y_POWER: N = 100.;
    const SIZE: N = 10.;
    const MASS: N = 10.;

    pub fn new(physics: &mut Physics) -> Self {
        let draw_param = DrawParam::new().src(Rect::new(
            0.,
            0.,
            Self::SIZE / IMAGE_WIDTH,
            Self::SIZE / IMAGE_HEIGHT,
        ));

        let body_handle = physics.build_body(
            RigidBodyDesc::new()
                .mass(Self::MASS)
                .translation(Vector2::new(40., 10.)),
        );

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
            Player::SIZE / 2. - 0.01,
            Player::SIZE / 2. - 0.01,
        )));

        let collider_handle = physics.build_collider(ColliderDesc::new(shape), body_handle, true);

        Player {
            draw_param,
            body_handle,
            collider_handle,
            on_ground: false,
        }
    }

    pub fn update(&mut self, ctx: &mut Context, physics: &mut Physics) {
        let rigid_body = physics.rigid_body_mut(self.body_handle);
        if self.on_ground && keyboard::is_key_pressed(ctx, KeyCode::Up) {
            self.on_ground = false;
            rigid_body.apply_force(
                0,
                &Force2::linear(Vector2::new(0., -Player::Y_POWER)),
                ForceType::VelocityChange,
                true,
            );
        }

        let direction = if keyboard::is_key_pressed(ctx, KeyCode::Left) {
            -1.
        } else if keyboard::is_key_pressed(ctx, KeyCode::Right) {
            1.
        } else {
            0.
        };
        rigid_body.set_linear_velocity(Vector2::new(
            direction * Self::X_POWER,
            rigid_body.velocity().linear[1],
        ));

        self.on_ground = physics.collisions(self.collider_handle).any(|manifold| {
            // for contact in manifold.contacts() {
            //     MyGame::draw_point(ctx, contact.contact.world1, Color::new(0., 1., 0., 1.));
            // }
            manifold.contacts().any(|contact| {
                contact.contact.normal.y.round() > 0. && contact.contact.normal.x.round() == 0.
            })
        });
    }

    pub fn draw(
        &mut self,
        ctx: &mut Context,
        physics: &mut Physics,
        spritesheet: &graphics::Image,
    ) -> GameResult {
        let rigid_body = physics.rigid_body(self.body_handle);
        let coords = rigid_body.position() * Point2::origin();

        graphics::draw(ctx, spritesheet, self.draw_param.dest(point2(coords)))?;

        Ok(())
    }
}

enum TileType {
    Entrance,
    Exit,
    Spikes,
    Wall,
}

struct Tile {
    type_: TileType,
    draw_param: DrawParam,
}

impl Tile {
    const SIZE: N = 10.;

    pub fn new(physics: &mut Physics, coords: Point2<N>, tile: &tiled::Tile) -> Self {
        use TileType::*;
        let type_ = match tile
            .tile_type
            .as_ref()
            .unwrap_or(&String::default())
            .as_str()
        {
            "Entrance" => Entrance,
            "Exit" => Exit,
            "Spikes" => Spikes,
            "Wall" => Wall,
            _ => panic!("Unknown tile type."),
        };

        match type_ {
            Wall => {
                let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
                    Self::SIZE / 2. - 0.01,
                    Self::SIZE / 2. - 0.01,
                )));
                let translation = Vector2::new(coords.x, coords.y) * Tile::SIZE;
                physics.build_collider(
                    ColliderDesc::new(shape).translation(translation),
                    physics.ground_handle,
                    false,
                );
            }
            _ => match &tile.objectgroup {
                Some(object_group) => {
                    let object = &object_group.objects[0];
                    match object.shape {
                        tiled::ObjectShape::Rect { width, height } => {
                            let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
                                width / 2. - 0.01,
                                height / 2. - 0.01,
                            )));
                            let translation = Vector2::new(coords.x, coords.y) * Tile::SIZE
                                + Vector2::new(object.x, object.y);

                            let body_handle = physics.build_body(
                                RigidBodyDesc::new()
                                    .translation(translation)
                                    .status(object::BodyStatus::Static),
                            );
                            physics.build_collider(ColliderDesc::new(shape), body_handle, false);
                        }
                        _ => (),
                    }
                }
                None => (),
            },
        }

        let tile_spritesheet_width = (IMAGE_WIDTH / Self::SIZE).floor();
        let tile_id = tile.id as f32;
        let (src_x, src_y) = (
            tile_id % tile_spritesheet_width,
            (tile_id / tile_spritesheet_width).floor(),
        );

        let draw_param = graphics::DrawParam::new()
            .src(Rect::new(
                src_x * (Self::SIZE / IMAGE_WIDTH),
                src_y * (Self::SIZE / IMAGE_HEIGHT),
                Self::SIZE / IMAGE_WIDTH,
                Self::SIZE / IMAGE_HEIGHT,
            ))
            .dest(point2(coords * Tile::SIZE));

        Tile { type_, draw_param }
    }
}

struct Tilemap {
    tilemap: tiled::Map,
    tiles: na::DMatrix<u32>,
    level_size: (usize, usize),
    level_tiles: Vec<Tile>,
}

impl Tilemap {
    pub fn new(tilemap: tiled::Map, level_size: (usize, usize)) -> Self {
        let tiles = tilemap.layers[0]
            .tiles
            .iter()
            .flatten()
            .map(|layer_tile| layer_tile.gid)
            .collect::<Vec<u32>>();
        let tiles =
            na::DMatrix::from_row_slice(tilemap.width as usize, tilemap.height as usize, &tiles);

        Tilemap {
            tilemap,
            tiles,
            level_size,
            level_tiles: Vec::new(),
        }
    }

    pub fn level_slice(&self, level_num: usize) -> na::DMatrixSlice<u32> {
        self.tiles.slice(
            self.tiles.vector_to_matrix_index(level_num),
            self.level_size,
        )
    }

    pub fn init_level(&mut self, level_num: usize, physics: &mut Physics) {
        let level_slice = self.level_slice(level_num);
        let tiled::Tileset {
            tiles, first_gid, ..
        } = &self.tilemap.tilesets[0];
        self.level_tiles = level_slice
            .iter()
            .enumerate()
            .filter_map(|(i, val)| {
                match tiles.binary_search_by_key(val, |tile| tile.id + *first_gid) {
                    Ok(tile_i) => Some({
                        let (y, x) = level_slice.vector_to_matrix_index(i);
                        let coords = Point2::new(x as f32, y as f32);

                        Tile::new(physics, coords, &tiles[tile_i])
                    }),
                    Err(_) => None,
                }
            })
            .collect();
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
}

impl Physics {
    const GRAVITY: f32 = 400.;

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
        self.body_set.rigid_body(handle).unwrap()
    }

    pub fn rigid_body_mut(&mut self, handle: DefaultBodyHandle) -> &mut RigidBody<N> {
        self.body_set.rigid_body_mut(handle).unwrap()
    }

    pub fn collisions(
        &self,
        handle: DefaultColliderHandle,
    ) -> impl Iterator<Item = &ncollide2d::query::ContactManifold<N>> {
        self.geometrical_world
            .contacts_with(&self.collider_set, handle, true)
            .into_iter()
            .flatten()
            .map(|(_, _, _, _, _, manifold)| manifold)
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
}

struct MyGame {
    player: Player,
    physics: Physics,
    tilemap: Tilemap,
    tilesheet_image: graphics::Image,
    spritesheet_image: graphics::Image,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        graphics::set_transform(
            ctx,
            DrawParam::new()
                .scale(vector2(Vector2::new(2., 2.)))
                .to_matrix(),
        );
        graphics::apply_transformations(ctx);
        graphics::set_default_filter(ctx, graphics::FilterMode::Nearest);

        let mut tilemap = {
            let file = ggez::filesystem::open(ctx, "/tilemap.tmx").unwrap();
            let tilemap = tiled::parse(file).unwrap();
            let level_size = {
                let level_size = &["level_width", "level_height"]
                    .iter()
                    .map(|property| {
                        if let tiled::PropertyValue::IntValue(val) =
                            tilemap.properties.get(*property).unwrap()
                        {
                            *val as usize
                        } else {
                            panic!("Tiled property has wrong type!")
                        }
                    })
                    .collect::<Vec<usize>>();
                (level_size[0], level_size[1])
            };

            Tilemap::new(tilemap, level_size)
        };

        let image_src = &tilemap.tilemap.tilesets[0].images[0].source;
        let tilesheet_image =
            graphics::Image::new(ctx, std::path::Path::new("/").join(image_src)).unwrap();

        let mut physics = {
            let geometrical_world = world::DefaultGeometricalWorld::new();
            let gravity = Vector2::y() * Physics::GRAVITY;
            let mechanical_world = world::DefaultMechanicalWorld::new(gravity);
            let mut body_set = object::DefaultBodySet::new();
            let collider_set = object::DefaultColliderSet::new();

            let joint_constraint_set = nphysics2d::joint::DefaultJointConstraintSet::new();
            let force_generator_set = nphysics2d::force_generator::DefaultForceGeneratorSet::new();

            let ground_handle = body_set.insert(object::Ground::new());

            Physics {
                geometrical_world,
                mechanical_world,
                body_set,
                collider_set,
                joint_constraint_set,
                force_generator_set,
                ground_handle,
            }
        };

        tilemap.init_level(0, &mut physics);

        let spritesheet_image = graphics::Image::new(ctx, "/spritesheet.png").unwrap();
        let player = Player::new(&mut physics);

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
        self.physics.step();
        self.player.update(ctx, &mut self.physics);
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        graphics::clear(ctx, graphics::WHITE);
        for tile in self.tilemap.level_tiles.iter() {
            graphics::draw(ctx, &self.tilesheet_image, tile.draw_param)?;
        }
        self.player
            .draw(ctx, &mut self.physics, &self.spritesheet_image)?;
        graphics::present(ctx)
    }
}

fn main() {
    let resource_dir = std::path::PathBuf::from("./assets");

    // Make a Context.
    let (mut ctx, mut event_loop) = ContextBuilder::new("ascension-rust", "Anton")
        .add_resource_path(resource_dir)
        .window_setup(
            ggez::conf::WindowSetup::default()
                .title("Ascension")
                .samples(ggez::conf::NumSamples::Zero),
        )
        .window_mode(ggez::conf::WindowMode::default().dimensions(300., 300.))
        .build()
        .expect("Could not create ggez context.");

    let mut my_game = MyGame::new(&mut ctx);

    match ggez::event::run(&mut ctx, &mut event_loop, &mut my_game) {
        Ok(_) => println!("Exited cleanly."),
        Err(e) => println!("Error occured: {}", e),
    }
}
