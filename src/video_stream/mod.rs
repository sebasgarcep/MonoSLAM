mod base;
mod camera_params;
mod mock_stream;
// mod standard_camera;
// mod uvc_stream;
mod wide_angle_camera;

pub use video_stream::base::{Camera, VideoStream};
pub use video_stream::mock_stream::MockStream;
// pub use video_stream::uvc_stream::UVCStream;
