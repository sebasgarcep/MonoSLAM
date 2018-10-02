impl Renderer {
    pub fn start (&mut self) {
        let mut opt_corner_list: Option<Vec<Feature>> = None;
        let mut frames: Vec<Feature> = vec![];

        while let Some(event) = window.next() {
            window.draw_2d(&event, |context, graphics| {
                if let Some(good_corners) = &opt_corner_list {
                    let half_len = (BLOCKSIZE as f64) / 2.0;
                    for Feature { u, v, .. } in good_corners {
                        render_rectangle(
                            [0.0, 0.0, 0.0, 1.0],
                            [(*u as f64) - half_len, (*v as f64) - half_len, half_len, half_len],
                            context.transform,
                            graphics,
                        );
                    }
                }

                for Feature { u, v, .. } in &frames {
                    let color = [0.0, 1.0, 0.0, 1.0];
                    let thickness = 1.0;

                    let p1x = (*u as f64) - 20.0;
                    let p1y = (*v as f64) - 20.0;
                    let p2x = (*u as f64) - 20.0;
                    let p2y = (*v as f64) + 20.0;
                    let p3x = (*u as f64) + 20.0;
                    let p3y = (*v as f64) + 20.0;
                    let p4x = (*u as f64) + 20.0;
                    let p4y = (*v as f64) - 20.0;

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
    }
}
