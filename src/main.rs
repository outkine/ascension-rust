extern crate nalgebra as na;

use ggez::event::{self, EventHandler, KeyCode, KeyMods};
use ggez::graphics::DrawParam;
use ggez::input::keyboard;
use ggez::{graphics, Context, ContextBuilder, GameResult};

use na::{Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::algebra::{ForceType, Velocity2};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::math::{Inertia, Velocity};
use nphysics2d::object::{
    Body, BodyHandle, BodyPart, BodyPartHandle, BodySet, BodyStatus, ColliderDesc, ColliderHandle,
    ColliderSet, DefaultBodyHandle, DefaultBodySet, DefaultColliderSet, Ground, RigidBody,
    RigidBodyDesc,
};
use nphysics2d::world::{
    DefaultGeometricalWorld, DefaultMechanicalWorld, GeometricalWorld, MechanicalWorld,
};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use tiled::parse;

fn main() {
    // Make a Context.
    let (mut ctx, mut event_loop) = ContextBuilder::new("Test", "Anton")
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
}

impl Player {
    const X_POWER: f32 = 100.;
    const SIZE: f32 = 10.;

    pub fn update(&mut self, ctx: &mut Context, physics: &mut Physics) {
        let x_dir = if keyboard::is_key_pressed(ctx, KeyCode::Left) {
            -1.
        } else if keyboard::is_key_pressed(ctx, KeyCode::Right) {
            1.
        } else {
            0.
        };

        let rigid_body = physics.body_set.rigid_body_mut(self.body_handle).unwrap();
        rigid_body.set_linear_velocity(Vector2::new(
            Player::X_POWER * x_dir,
            rigid_body.velocity().linear[1],
        ))
    }

    pub fn draw(&mut self, ctx: &mut Context, physics: &mut Physics) {
        let rigid_body = physics.body_set.rigid_body(self.body_handle).unwrap();
        let coords = rigid_body.position() * Point2::origin();

        let rect = graphics::Rect::new(coords.x, coords.y, Player::SIZE, Player::SIZE);
        let r1 =
            graphics::Mesh::new_rectangle(ctx, graphics::DrawMode::fill(), rect, graphics::BLACK)
                .unwrap();
        graphics::draw(ctx, &r1, DrawParam::default()).unwrap();
    }
}

struct Physics {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    body_set: DefaultBodySet<f32>,
    collider_set: DefaultColliderSet<f32>,
    joint_constraint_set: DefaultJointConstraintSet<f32>,
    force_generator_set: DefaultForceGeneratorSet<f32>,
}

impl Physics {
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

struct MyGame {
    player: Player,
    physics: Physics,
}

impl MyGame {
    pub fn new(_ctx: &mut Context) -> MyGame {
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

        let levels = tilemap.layers[0]
            .tiles
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<u32>>();
        let levels =
            na::DMatrix::from_row_slice(tilemap.width as usize, tilemap.height as usize, &levels);
        println!("{}", levels.slice((0, 0), level_size));

        let geometrical_world = DefaultGeometricalWorld::new();
        let gravity = Vector2::y() * 100.;
        let mechanical_world = DefaultMechanicalWorld::new(gravity);
        let mut body_set = DefaultBodySet::new();
        let mut collider_set = DefaultColliderSet::new();

        let player_handle = body_set.insert(RigidBodyDesc::new().mass(10.).build());

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(10., 10.)));
        let collider = ColliderDesc::new(shape).build(BodyPartHandle(player_handle, 0));
        collider_set.insert(collider);

        let joint_constraint_set = DefaultJointConstraintSet::new();
        let force_generator_set = DefaultForceGeneratorSet::new();

        MyGame {
            player: Player {
                body_handle: player_handle,
            },
            physics: Physics {
                geometrical_world,
                mechanical_world,
                body_set,
                collider_set,
                joint_constraint_set,
                force_generator_set,
            },
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

        self.player.draw(ctx, &mut self.physics);

        graphics::present(ctx)
    }
}
