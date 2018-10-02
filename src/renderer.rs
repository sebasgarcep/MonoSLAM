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
    window: PistonWindow,
}

impl Renderer {
    pub fn new (width: u32, height: u32) -> Renderer {
        Renderer {
            image_buffer: SharedBuffer::new(),
            image_texture: None,
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
    }

    fn render (&mut self, event: Event) {
        let image_texture = &self.image_texture;
        self.window.draw_2d(&event, |context, graphics| {
            if let Some(ref texture_content) = image_texture {
                render_image(texture_content, context.transform, graphics);
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
