/*
Code to capture images to be used in Matlab for camera calibration.
For more info: https://www.mathworks.com/help/vision/ref/cameracalibrator-app.html

let _stream = streamh
    .start_stream(move |frame, user_data| {
        let width = frame.width();
        let height = frame.height();

        if let Ok(new_frame) = frame.to_rgb() {
            let data = new_frame.to_bytes();
            if let Some(image) = RgbImage::from_raw(width, height, data.to_vec()) {
                let mut frame_num = Mutex::lock(&user_data).unwrap();
                if *frame_num % 100 == 0 {
                    let idx = *frame_num / 100;
                    let  _ = image.save(format!("./image-{}.jpg", idx));
                }
                *frame_num = *frame_num + 1;
                if *frame_num > 2000 {
                    panic!();
                }
            }
        }

    }, Arc::new(Mutex::new(0)))
    .unwrap();

loop {}
*/
