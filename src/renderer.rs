use constants::{
    BLOCKSIZE,
    WIN_HEIGHT,
    WIN_WIDTH
};
use feature::Feature;
use piston_window::{
    image as render_image,
    rectangle as render_rectangle,
    PistonWindow,
    Texture,
    TextureSettings,
    WindowSettings,
};
use state::{AppState, SharedAppState};
use std::sync::Mutex;

pub struct Renderer {
    app_state: SharedAppState,
}

impl Renderer {
    pub fn new (app_state: SharedAppState) -> Renderer {
        Renderer { app_state }
    }

    pub fn start (&self) {
        let mut window: PistonWindow =
            WindowSettings::new("Hello Piston!", [WIN_WIDTH, WIN_HEIGHT])
                .exit_on_esc(true)
                .build()
                .unwrap();

        let mut texture: Option<Texture<_>> = None;
        let mut opt_corner_list: Option<Vec<Feature>> = None;

        while let Some(event) = window.next() {
            let mut mutex = Mutex::lock(&self.app_state).unwrap();
            let lock_results = mutex.take();

            if let Some(AppState { image, features }) = lock_results {
                opt_corner_list = Some(features);
                if let Some(ref mut texture_content) = texture {
                    let _ = texture_content.update(&mut window.encoder, &image);
                } else {
                    texture = Texture::from_image(
                        &mut window.factory,
                        &image,
                        &TextureSettings::new(),
                    ).ok();
                }
            }

            window.draw_2d(&event, |context, graphics| {
                if let Some(ref texture_content) = texture {
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
