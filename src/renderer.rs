use constants::{
    BLOCKSIZE,
    WIN_HEIGHT,
    WIN_WIDTH
};
use feature::Feature;
use im::RgbaImage;
use piston_window::{
    image as render_image,
    rectangle as render_rectangle,
    G2dTexture,
    PistonWindow,
    Texture,
    TextureSettings,
    WindowSettings,
};
use state::{AppState, SharedAppState};
use std::sync::Mutex;

pub struct Renderer {
    app_state: SharedAppState,
    image_texture: Option<G2dTexture>,
}

impl Renderer {
    pub fn new (app_state: SharedAppState) -> Renderer {
        Renderer { app_state, image_texture: None }
    }

    fn set_texture_from_image (&mut self, window: &mut PistonWindow, image: &RgbaImage) {
        if let Some(ref mut texture_content) = self.image_texture {
            let _ = texture_content.update(&mut window.encoder, &image);
        } else {
            self.image_texture = Texture::from_image(
                &mut window.factory,
                &image,
                &TextureSettings::new(),
            ).ok();
        }
    }

    pub fn start (&mut self) {
        let mut window: PistonWindow =
            WindowSettings::new("Hello Piston!", [WIN_WIDTH, WIN_HEIGHT])
                .exit_on_esc(true)
                .build()
                .unwrap();

        let mut opt_corner_list: Option<Vec<Feature>> = None;

        while let Some(event) = window.next() {
            let lock_results;
            {
                let mut mutex = Mutex::lock(&self.app_state).unwrap();
                lock_results = mutex.take();
            }

            if let Some(app_state) = lock_results {
                match app_state {
                    AppState::FeatureSearchState { image, features } => {
                        opt_corner_list = Some(features);
                        self.set_texture_from_image(&mut window, &image);
                    },
                    AppState::InitState { image } => {
                        self.set_texture_from_image(&mut window, &image);
                    },
                };
            }

            window.draw_2d(&event, |context, graphics| {
                if let Some(ref texture_content) = self.image_texture {
                    render_image(texture_content, context.transform, graphics);
                }

                if let Some(good_corners) = &opt_corner_list {
                    let half_len = (BLOCKSIZE as f64) / 2.0;
                    for Feature { x, y, .. } in good_corners {
                        render_rectangle(
                            [0.0, 0.0, 0.0, 1.0],
                            [(*x as f64) - half_len, (*y as f64) - half_len, half_len, half_len],
                            context.transform,
                            graphics,
                        );
                    }
                }
            });
        }
    }
}
