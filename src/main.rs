use ggez::event::KeyCode;
use ggez::graphics::spritebatch::SpriteBatch;
use ggez::graphics::{DrawParam, Rect};
use ggez::input::keyboard;
use ggez::nalgebra::{Point2, Vector2};
use ggez::{event, graphics, input, GameResult};
use ggez::{nalgebra as na, Context};

type N = f32;

fn main() {
    let resource_dir = std::path::PathBuf::from("./assets");

    // Make a Context.
    let (mut ctx, mut event_loop) = ggez::ContextBuilder::new("Test", "Anton")
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

struct Physics {
    pos: Point2<N>,
    size: Point2<N>,
}

impl Physics {
    fn new(pos: Point2<N>, size: Point2<N>) -> Self {
        Physics { pos, size }
    }

    fn collided(&self, other: &Physics) -> bool {
        self.pos.x < other.pos.x + other.size.x
            && other.pos.x < self.pos.x + self.size.x
            && self.pos.y < other.pos.y + other.size.y
            && other.pos.y < self.pos.y + self.size.y
    }
}

struct Player {
    physics: Physics,
    velocity: Point2<N>,
    on_ground: bool,
}

impl Player {
    const SIZE: N = 10.;
    const JUMP_POWER: N = 10.;
    const MOVEMENT_POWER: N = 5.;
    const GRAVITY: N = 1.;

    pub fn new() -> Self {
        Player {
            physics: Physics::new(
                Point2::new(30., 30.),
                Point2::new(Player::SIZE, Player::SIZE),
            ),
            velocity: Point2::origin(),
            on_ground: true,
        }
    }

    fn draw(&self, ctx: &mut Context) -> GameResult {
        let rect = graphics::Rect::new(
            self.physics.pos.x,
            self.physics.pos.y,
            Player::SIZE,
            Player::SIZE,
        );
        let r1 =
            graphics::Mesh::new_rectangle(ctx, graphics::DrawMode::fill(), rect, graphics::BLACK)?;
        graphics::draw(ctx, &r1, DrawParam::default())?;

        Ok(())
    }
}

struct Tile {
    physics: Physics,
    type_: TileType,
}

enum TileType {
    Wall,
}

impl Tile {
    const SIZE: N = 10.;

    pub fn new(spritebatch: &mut SpriteBatch, coords: Point2<N>) -> Self {
        let p = graphics::DrawParam::new()
            .src(Rect::new(0., 0., 10. / 256., 10. / 256.))
            .dest(coords);
        spritebatch.add(p);

        Tile {
            physics: Physics::new(coords.clone(), Point2::new(Tile::SIZE, Tile::SIZE)),
            type_: TileType::Wall,
        }
    }
}

struct Tilemap {
    tilemap: tiled::Map,
    raw_tiles: na::DMatrix<u32>,
    level_size: (usize, usize),
    current_level: Vec<Tile>,
}

impl Tilemap {
    pub fn new(tilemap: tiled::Map) -> Self {
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
        let tiles =
            na::DMatrix::from_row_slice(tilemap.width as usize, tilemap.height as usize, &tiles);

        Tilemap {
            tilemap,
            raw_tiles: tiles,
            level_size,
            current_level: Vec::new(),
        }
    }

    pub fn level_slice(&self, level_num: usize) -> na::DMatrixSlice<u32> {
        self.raw_tiles.slice(
            self.raw_tiles.vector_to_matrix_index(level_num),
            self.level_size,
        )
    }

    pub fn init_level(&mut self, level_num: usize, spritebatch: &mut SpriteBatch) {
        let level_slice = self.level_slice(level_num);
        self.current_level = level_slice
            .iter()
            .enumerate()
            .filter(|(_, v)| **v != 0)
            .map(|(i, v)| {
                let (x, y) = level_slice.vector_to_matrix_index(i);
                let (x, y) = (y as f32, x as f32);
                Tile::new(spritebatch, Point2::new(x * Tile::SIZE, y * Tile::SIZE))
            })
            .collect();
    }
}

struct MyGame {
    player: Player,
    tilemap: Tilemap,
    spritebatch: graphics::spritebatch::SpriteBatch,
}

impl MyGame {
    pub fn new(ctx: &mut Context) -> MyGame {
        let mut tilemap = {
            let file = std::fs::File::open(&std::path::Path::new("assets/tilemap.tmx")).unwrap();
            let tilemap = tiled::parse(file).unwrap();
            Tilemap::new(tilemap)
        };

        let image = graphics::Image::new(ctx, "/sprite_sheet.png").unwrap();
        let mut spritebatch = graphics::spritebatch::SpriteBatch::new(image);

        tilemap.init_level(0, &mut spritebatch);

        let player = Player::new();

        MyGame {
            player,
            tilemap,
            spritebatch,
        }
    }
}

impl event::EventHandler for MyGame {
    fn update(&mut self, ctx: &mut Context) -> GameResult {
        if !self.player.on_ground && keyboard::is_key_pressed(ctx, KeyCode::Up) {
            self.player.on_ground = false;
            self.player.velocity.y = -Player::JUMP_POWER;
        }
        self.player.velocity.y += Player::GRAVITY;

        let x_dir = if keyboard::is_key_pressed(ctx, KeyCode::Left) {
            -1.
        } else if keyboard::is_key_pressed(ctx, KeyCode::Right) {
            1.
        } else {
            0.
        };
        self.player.velocity.x = x_dir * Player::MOVEMENT_POWER;

        for axis in &[0, 1] {
            let mut new_pos = self.player.physics.pos.clone();
            new_pos[*axis] += self.player.velocity[*axis];
            if self.player.physics.collided()
        }

        for tile in &self.tilemap.current_level {
            if self.player.physics.collided(&tile.physics) {
                self.player.velocity = Point2::origin();
            }
        }

        self.player.physics.pos =
            (&self.player.velocity.coords + &self.player.physics.pos.coords).into();

        Ok(())
    }

    fn draw(&mut self, ctx: &mut Context) -> GameResult {
        graphics::clear(ctx, graphics::WHITE);
        graphics::draw(ctx, &self.spritebatch, graphics::DrawParam::new())?;
        self.player.draw(ctx)?;
        graphics::present(ctx);

        ggez::timer::yield_now();

        Ok(())
    }
}
