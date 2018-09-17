use constants::{
    BLOCKSIZE,
    NUM_SIGMA,
};
use std;
use std::collections::HashMap;
use typedefs::{Matrix, Vector};

fn inside_relative (si_inv: &Matrix, uu: u32, uv: u32) -> bool {
        let u = uu as f64;
        let v = uv as f64;
        let pos = si_inv[[0, 0]] * u.powi(2) +
            2.0 * si_inv[[0, 1]] * u * v +
            si_inv[[1, 1]] * v.powi(2);
        pos < NUM_SIGMA.powi(2)
    }

fn get_image_correlation (
    prev_image_normalized: &Matrix,
    prev_ui: usize,
    prev_vi: usize,
    prev_uf: usize,
    prev_vf: usize,
    curr_image_normalized: &Matrix,
    curr_ui: usize,
    curr_vi: usize,
    curr_uf: usize,
    curr_vf: usize,
) -> f64 {
    let prev_image_win = prev_image_normalized.slice(
        s![prev_ui..(prev_uf + 1), prev_vi..(prev_vf + 1)]
    );
    let curr_image_win = curr_image_normalized.slice(
        s![curr_ui..(curr_uf + 1), curr_vi..(curr_vf + 1)]
    );
    let mut diff = &prev_image_win - &curr_image_win;
    diff.mapv_inplace(|e| e.powi(2));
    diff.scalar_sum() // FIXME: return sdimage
}

fn normalize_image (
    image: &Matrix,
) -> Matrix {
    let mean = image.scalar_sum() / (image.len() as f64);
    let mut result = image.clone();
    result.mapv_inplace(|e| e - mean);
    let norm = result.mapv(|e| e.powi(2)).scalar_sum().sqrt();
    result.mapv_inplace(|e| e / norm);
    result
}

pub fn ellipse_search (
    prev_image: &Matrix,
    curr_image: &Matrix,
    ellipses: &Vec<(Vector, Matrix, Matrix, f64)>,
) ->  Vec<(u32, u32)> {
    let shape = prev_image.shape();
    let width = shape[0] as i32;
    let height = shape[1] as i32;
    let halfblocksize = (BLOCKSIZE / 2) as i32;
    let totalblocksize = BLOCKSIZE as i32;
    let mut correlation_map = HashMap::new();
    let prev_image_normalized = normalize_image(prev_image);
    let curr_image_normalized = normalize_image(curr_image);
    let mut result = vec![];

    for ellipse in ellipses {
        let (hi, _, si_inv, _) = ellipse;
        let si_diag_prod = si_inv[[0, 1]].powi(2);
        let halfwidth = (NUM_SIGMA / (si_inv[[0, 0]] - si_diag_prod / si_inv[[1, 1]]).sqrt()) as i32;
        let halfheight = (NUM_SIGMA / (si_inv[[1, 1]] - si_diag_prod / si_inv[[0, 0]]).sqrt()) as i32;
        let mut urelstart = - halfwidth;
        let mut urelfinish = halfwidth;
        let mut vrelstart = - halfheight;
        let mut vrelfinish = halfheight;
        let ucentre = hi[0] as i32;
        let vcentre = hi[1] as i32;

        if ucentre + urelstart - halfblocksize < 0 {
            urelstart = halfblocksize - ucentre;
        }

        if ucentre + urelfinish - halfblocksize > width - totalblocksize {
            urelfinish = width - totalblocksize - ucentre + halfblocksize;
        }

        if vcentre + vrelstart - halfblocksize < 0 {
            vrelstart = halfblocksize - vcentre;
        }

        if vcentre + vrelfinish - halfblocksize > height - totalblocksize {
            vrelfinish = height - totalblocksize - vcentre + halfblocksize;
        }

        let mut corrmin = std::f64::INFINITY;
        let mut res_u = 0;
        let mut res_v = 0;

        /* Do the search */
        for urel in urelstart..(urelfinish + 1) {
            for vrel in vrelstart..(vrelfinish + 1) {
                if inside_relative(si_inv, urel as u32, vrel as u32) {
                    // We are inside ellipse
                    // Has this place been searched before?
                    let pos = (ucentre + urel, vcentre + vrel);
                    let computation_performed = correlation_map.contains_key(&pos);

                    let corr: f64 = if computation_performed {
                        *correlation_map.get(&pos).unwrap()
                    } else {
                        let corr_result = get_image_correlation(
                            &prev_image_normalized,
                            (ucentre - halfblocksize) as usize,
                            (vcentre - halfblocksize) as usize,
                            (ucentre + halfblocksize) as usize,
                            (vcentre + halfblocksize) as usize,
                            &curr_image_normalized,
                            (ucentre + urel - halfblocksize) as usize,
                            (vcentre + vrel - halfblocksize) as usize,
                            (ucentre + urel + halfblocksize) as usize,
                            (vcentre + vrel + halfblocksize) as usize,
                        );

                        correlation_map.insert(pos, corr_result);

                        /* FIXME: ADD THIS
                        if sdimage < CORRELATION_SIGMA_THRESHOLD {
                            corr_result += LOW_SIGMA_PENALTY;
                        }
                        */

                        corr_result
                    };

                    if corr <= corrmin {
                        corrmin = corr;
                        res_u = urel + ucentre;
                        res_v = vrel + vcentre;
                    }
                }
            }
        }

        /* FIXME: THIS HAS NOT BEEN PORTED
        if corrmax > CORRELATION_THRESHOLD {
            Correlation not good enough.
        } else {
            Correlation good enough.
        }
        */

        result.push((res_u as u32, res_v as u32));
    }

    result
}
