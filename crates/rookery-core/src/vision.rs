//! Counting bodies in a cell's depth view (row 11.4).
//!
//! ```text
//!   heights (m above the box floor, NaN where the ray hit nothing)
//!     ─▶ band [lo, hi]: 1 where a body lies on the floor   (the handle is far above)
//!     ─▶ 4-connected components by breadth-first search
//!     ─▶ drop the ones smaller than min_px (a stray pixel at a wall's edge)
//! ```
//!
//! Two bodies that touch merge into one component, and the count reads one:
//! the camera then misses the molt until they part. The shell lies 1 cm from
//! the crawfish, about 3 pixels at 32 × 32, so in this sim they don't touch.

use std::collections::VecDeque;

/// A depth view turned into heights above the box floor, row-major.
#[derive(Debug, Clone, PartialEq)]
pub struct HeightImage {
    pub w: usize,
    pub h: usize,
    /// `w · h` heights in metres; NaN where the ray hit nothing.
    pub z: Vec<f64>,
}

/// One connected body in a mask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Component {
    /// Pixels in it.
    pub area: usize,
}

/// Pixels whose height lies in `[lo, hi]`, as a row-major mask.
pub fn band(img: &HeightImage, lo: f64, hi: f64) -> Vec<bool> {
    img.z.iter().map(|&z| z >= lo && z <= hi).collect()
}

/// 4-connected components of `mask` (`w` wide), largest first.
pub fn components(mask: &[bool], w: usize) -> Vec<Component> {
    let h = mask.len().checked_div(w).unwrap_or(0);
    let mut seen = vec![false; mask.len()];
    let mut out = Vec::new();
    for start in 0..w * h {
        if !mask[start] || seen[start] {
            continue;
        }
        seen[start] = true;
        let mut queue = VecDeque::from([start]);
        let mut area = 0;
        while let Some(i) = queue.pop_front() {
            area += 1;
            let (x, y) = (i % w, i / w);
            let next = [
                (x > 0).then(|| i - 1),
                (x + 1 < w).then(|| i + 1),
                (y > 0).then(|| i - w),
                (y + 1 < h).then(|| i + w),
            ];
            for j in next.into_iter().flatten() {
                if mask[j] && !seen[j] {
                    seen[j] = true;
                    queue.push_back(j);
                }
            }
        }
        out.push(Component { area });
    }
    out.sort_by_key(|c| std::cmp::Reverse(c.area));
    out
}

/// How many bodies lie on the floor: components in the band with at least
/// `min_px` pixels.
pub fn count_bodies(img: &HeightImage, lo: f64, hi: f64, min_px: usize) -> usize {
    components(&band(img, lo, hi), img.w)
        .iter()
        .filter(|c| c.area >= min_px)
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `w × h` image of floor (0.0) with rectangles of `height` painted on.
    fn img(w: usize, h: usize, rects: &[(usize, usize, usize, usize, f64)]) -> HeightImage {
        let mut z = vec![0.0; w * h];
        for &(x0, y0, x1, y1, v) in rects {
            for y in y0..y1 {
                for x in x0..x1 {
                    z[y * w + x] = v;
                }
            }
        }
        HeightImage { w, h, z }
    }

    #[test]
    fn one_blob_is_one_body() {
        let i = img(32, 32, &[(4, 10, 27, 17, 0.02)]);
        assert_eq!(count_bodies(&i, 0.003, 0.04, 10), 1);
    }

    #[test]
    fn two_separated_blobs_are_two_bodies() {
        let i = img(32, 32, &[(4, 10, 27, 17, 0.02), (6, 20, 23, 25, 0.015)]);
        assert_eq!(count_bodies(&i, 0.003, 0.04, 10), 2);
        assert_eq!(
            components(&band(&i, 0.003, 0.04), 32)
                .iter()
                .map(|c| c.area)
                .collect::<Vec<_>>(),
            [161, 85]
        );
    }

    #[test]
    fn touching_blobs_read_as_one_documented() {
        // Diagonal neighbours don't connect (4-connectivity), side neighbours do.
        let touching = img(32, 32, &[(4, 10, 27, 17, 0.02), (6, 17, 23, 22, 0.015)]);
        assert_eq!(
            count_bodies(&touching, 0.003, 0.04, 10),
            1,
            "touching bodies merge"
        );
        let mut diag = vec![false; 4];
        diag[0] = true;
        diag[3] = true;
        assert_eq!(components(&diag, 2).len(), 2, "corners alone don't connect");
    }

    #[test]
    fn the_band_drops_the_floor_the_handle_and_specks() {
        // Something high (11 cm up: a post at the box's end), a 4-pixel speck
        // and NaN don't count. (The rays see through the handle bar itself,
        // so it never cuts a body in two.)
        let mut i = img(
            32,
            32,
            &[
                (4, 10, 27, 17, 0.02),
                (15, 20, 17, 32, 0.11),
                (30, 30, 32, 32, 0.02),
            ],
        );
        i.z[0] = f64::NAN;
        assert_eq!(count_bodies(&i, 0.003, 0.04, 10), 1);
        assert_eq!(
            count_bodies(&img(8, 8, &[]), 0.003, 0.04, 1),
            0,
            "an empty box"
        );
    }
}
