extern crate nalgebra as na;

use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::{DrawParam, Rect};
use ggez::input::keyboard;
use ggez::{graphics, Context, ContextBuilder, GameResult};

use ggez::graphics::spritebatch::SpriteBatch;
use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::algebra::{Force2, ForceType, Velocity2};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::math::{Inertia, Velocity};
use nphysics2d::object::{
    Body, BodyHandle, BodyPart, BodyPartHandle, BodySet, BodyStatus, ColliderDesc, ColliderHandle,
    ColliderSet, DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet,
    Ground, RigidBody, RigidBodyDesc,
};
use nphysics2d::world::{
    DefaultGeometricalWorld, DefaultMechanicalWorld, GeometricalWorld, MechanicalWorld,
};
use nphysics2d::{material, object};
use std::convert::{TryFrom, TryInto};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use tiled::parse;

type N = f32;

fn main() {
    let resource_dir = std::path::PathBuf::from("./assets");

    // Make a Context.
    let (mut ctx, mut event_loop) = ContextBuilder::new("Test", "Anton")
        .add_resource_path(resource_dir)
        .window_setup(ggez::conf::WindowSetup::default().title("Game!"))
        .window_mode(ggez::conf::WindowMode::default().dimensions(100., 100.))
        .build()
        .expect("Could not create ggez context!");

    // Create an instance of your event handler.
    // Usually, you should provide it with the Context object to
    // use when setting your game up.
    let mut my_game = MyGame::new(&mut ctx);

    // Run!
    match event::run(&mut ctx, &mut event_loop, &mut my_game) {
        Ok(_) => println!("Exited cleanly."),
        Err(e) => println!("Error occured: {}", e),
    }
}

struct Player {
    body_handle: DefaultBodyHandle,
    is_jumping: bool,
    collider_handle: DefaultColliderHandle,
}

impl Player {
    const X_POWER: N = 200.;
    const Y_POWER: N = 200.;
    const SIZE: N = 10.;

    pub fn update(&mut self, ctx: &mut Context, physics: &mut Physics) {
        let rigid_body = physics.body_set.rigid_body_mut(self.body_handle).unwrap();
        if !self.is_jumping && keyboard::is_key_pressed(ctx, KeyCode::Up) {
            self.is_jumping = true;
            rigid_body.apply_force(
                0,
                &Force2::linear(Vector2::new(0., -Player::Y_POWER)),
                ForceType::VelocityChange,
                true,
            );
        }

        let x_dir = if keyboard::is_key_pressed(ctx, KeyCode::Left) {
            -1.
        } else if keyboard::is_key_pressed(ctx, KeyCode::Right) {
            1.
        } else {
            0.
        };

        rigid_body.set_linear_velocity(Vector2::new(0., rigid_body.velocity().linear[1]));
        rigid_body.apply_force(
            0,
            &Force2::linear(Vector2::new(Player::X_POWER * x_dir, 0.)),
            ForceType::VelocityChange,
            true,
        );

        // physics
        //     .geometrical_world
        //     .contacts_with(&physics.collider_set, self.collider_handle, true)
        //     .map(|mut iter| {
        //         iter.any(|(_, _, _, _, _, manifold)| {
        //             println!("{:?}", manifold);
        //             false
        //         })
        //     });
    }

    pub fn draw(&mut self, ctx: &mut Context, physics: &mut Physics) -> GameResult {
        let rigid_body = physics.body_set.rigid_body(self.body_handle).unwrap();
        let coords = rigid_body.position() * Point2::origin();

        let rect = graphics::Rect::new(coords.x, coords.y, Player::SIZE, Player::SIZE);
        let r1 =
            graphics::Mesh::new_rectangle(ctx, graphics::DrawMode::fill(), rect, graphics::BLACK)?;
        graphics::draw(ctx, &r1, DrawParam::default())?;

        Ok(())
    }

    pub fn init(physics: &mut Physics) -> Self {
        let body_handle = physics.body_set.insert(
            RigidBodyDesc::new()
                .mass(10.)
                .translation(Vector2::new(40., 10.))
                .build(),
        );

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(4.99, 4.99)));
        let collider = ColliderDesc::new(shape).build(BodyPartHandle(body_handle, 0));
        let collider_handle = physics.collider_set.insert(collider);

        Player {
            body_handle,
            collider_handle,
            is_jumping: false,
        }
    }
}

struct Physics {
    mechanical_world: DefaultMechanicalWorld<N>,
    geometrical_world: DefaultGeometricalWorld<N>,
    body_set: DefaultBodySet<N>,
    collider_set: DefaultColliderSet<N>,
    joint_constraint_set: DefaultJointConstraintSet<N>,
    force_generator_set: DefaultForceGeneratorSet<N>,
    ground_handle: DefaultBodyHandle,
}

impl Physics {
    const GRAVITY: f32 = 200.;

    pub fn step(&mut self) {
        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.body_set,
            &mut self.collider_set,
            &mut self.joint_constraint_set,
            &mut self.force_generator_set,
        );
    }
}

enum TileType {
    Wall,
}

struct Tile {
    type_: TileType,
}

impl Tile {
    const SIZE: N = 10.;

    pub fn new(physics: &mut Physics, spritebatch: &mut SpriteBatch, coords: Point2<N>) -> Self {
        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
            Tile::SIZE / 2. - 0.01,
            Tile::SIZE / 2. - 0.01,
        )));

        let collider = ColliderDesc::new(shape)
            .material(material::MaterialHandle::new(material::BasicMaterial::new(
                0., 0.,
            )))
            .translation(Vector2::new(coords.x, coords.y))
            .build(BodyPartHandle(physics.ground_handle, 0));
        physics.collider_set.insert(collider);

        let p = graphics::DrawParam::new()
            .src(Rect::new(0., 0., 10. / 256., 10. / 256.))
            .dest(ggez::nalgebra::Point2::new(coords.x, coords.y));
        spritebatch.add(p);

        Tile {
            type_: TileType::Wall,
        }
    }
}

struct Tilemap {
    tilemap: tiled::Map,
    tiles: na::DMatrix<u32>,
    level_size: (usize, usize),
}

impl Tilemap {
    pub fn level_slice(&self, level_num: usize) -> na::DMatrixSlice<u32> {
        self.tiles.slice(
            self.tiles.vector_to_matrix_index(level_num),
            self.level_size,
        )
    }

    pub fn init_level(
        &mut self,
        level_num: usize,
        physics: &mut Physics,
        spritebatch: &mut SpriteBatch,
    ) {
        let level_slice = self.level_slice(level_num);
        for (i, val) in level_slice.iter().enumerate() {
            if *val != 0 {
                let (x, y) = level_slice.vector_to_matrix_index(i);
                let (x, y) = (y as f32, x as f32);
                Tile::new(
                    physics,
                    spritebatch,
                    Point2::new(x * Tile::SIZE, y * Tile::SIZE),
                );
            }
        }
    }
}

struct MyGame {
    player: Player,
    physics: Physics,
    tilemap: Tilemap,
    spritebatch: graphics::spritebatch::SpriteBatch,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        let mut tilemap = {
            let file = File::open(&Path::new("assets/tilemap.tmx")).unwrap();
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

            let tiles = tilemap.layers[0]
                .tiles
                .iter()
                .flatten()
                .copied()
                .collect::<Vec<u32>>();
            let tiles = na::DMatrix::from_row_slice(
                tilemap.width as usize,
                tilemap.height as usize,
                &tiles,
            );

            Tilemap {
                tilemap,
                tiles,
                level_size,
            }
        };

        let image = graphics::Image::new(ctx, "/sprite_sheet.png").unwrap();
        let mut spritebatch = graphics::spritebatch::SpriteBatch::new(image);

        let mut physics = {
            let geometrical_world = DefaultGeometricalWorld::new();
            let gravity = Vector2::y() * Physics::GRAVITY;
            let mechanical_world = DefaultMechanicalWorld::new(gravity);
            let mut body_set = DefaultBodySet::new();
            let mut collider_set = DefaultColliderSet::new();

            let joint_constraint_set = DefaultJointConstraintSet::new();
            let force_generator_set = DefaultForceGeneratorSet::new();

            let ground_handle = body_set.insert(Ground::new());

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

        tilemap.init_level(0, &mut physics, &mut spritebatch);

        let player = Player::init(&mut physics);

        MyGame {
            player,
            tilemap,
            physics,
            spritebatch,
        }
    }
}

impl EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        self.player.update(ctx, &mut self.physics);
        self.physics.step();
        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        graphics::clear(ctx, graphics::WHITE);
        graphics::draw(ctx, &self.spritebatch, graphics::DrawParam::new())?;
        self.player.draw(ctx, &mut self.physics)?;
        graphics::present(ctx)
    }
}
