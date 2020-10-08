use im::RgbaImage;
use piston_window::{
    image as render_image,
    rectangle as render_rectangle,
    line as render_line,
    Event,
    G2dTexture,
    PistonWindow,
    Texture,
    TextureSettings,
    WindowSettings,
};
use shared_buffer::SharedBuffer;

pub struct Renderer {
    image_buffer: SharedBuffer<RgbaImage>,
    image_texture: Option<G2dTexture>,
    landmark_buffer: SharedBuffer<Vec<(u32, u32)>>,
    landmarks: Vec<(u32, u32)>,
    window: PistonWindow,
}

impl Renderer {
    pub fn new (width: u32, height: u32, landmark_buffer: SharedBuffer<Vec<(u32, u32)>>) -> Renderer {
        Renderer {
            image_buffer: SharedBuffer::new(),
            image_texture: None,
            landmark_buffer,
            landmarks: vec![],
            window: WindowSettings::new("MonoSLAM", [width, height])
                .exit_on_esc(true)
                .build()
                .unwrap(),
        }
    }

    fn set_texture_from_image (&mut self, image: &RgbaImage) {
        if let Some(ref mut texture_content) = self.image_texture {
            let _ = texture_content.update(&mut self.window.encoder, &image);
        } else {
            self.image_texture = Texture::from_image(
                &mut self.window.factory,
                &image,
                &TextureSettings::new(),
            ).ok();
        }
    }

    fn preprocess (&mut self) {
        let maybe_image = self.image_buffer.take();

        if let Some(image) = maybe_image {
            self.set_texture_from_image(&image);
        }

        let maybe_landmarks = self.landmark_buffer.take();

        if let Some(landmarks) = maybe_landmarks {
            self.landmarks = landmarks;
        }
    }

    fn render (&mut self, event: Event) {
        let image_texture = &self.image_texture;
        let landmarks_iter = self.landmarks.iter();
        self.window.draw_2d(&event, |context, graphics| {
            if let Some(ref texture_content) = image_texture {
                render_image(texture_content, context.transform, graphics);
            }

            for landmark in landmarks_iter {
                let (u, v) = landmark;

                let color = [0.0, 1.0, 0.0, 1.0];
                let thickness = 1.0;

                let p1x = (*u as f64) - 5.0;
                let p1y = (*v as f64) - 5.0;
                let p2x = (*u as f64) - 5.0;
                let p2y = (*v as f64) + 5.0;
                let p3x = (*u as f64) + 5.0;
                let p3y = (*v as f64) + 5.0;
                let p4x = (*u as f64) + 5.0;
                let p4y = (*v as f64) - 5.0;

                render_line(
                    color,
                    thickness,
                    [p1x, p1y, p2x, p2y],
                    context.transform,
                    graphics,
                );

                render_line(
                    color,
                    thickness,
                    [p2x, p2y, p3x, p3y],
                    context.transform,
                    graphics,
                );

                render_line(
                    color,
                    thickness,
                    [p3x, p3y, p4x, p4y],
                    context.transform,
                    graphics,
                );

                render_line(
                    color,
                    thickness,
                    [p4x, p4y, p1x, p1y],
                    context.transform,
                    graphics,
                );
            }
        });
    }

    pub fn start (&mut self) {
        while let Some(event) = self.window.next() {
            self.preprocess();
            self.render(event);
        }
    }

    pub fn get_image_buffer (&self) -> SharedBuffer<RgbaImage> {
        self.image_buffer.clone()
    }
}
