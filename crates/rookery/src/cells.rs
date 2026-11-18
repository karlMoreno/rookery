//! Cells of the farm, on every tray (Phase 10): their order, names and
//! places in qpos, in one place.
//!
//! ```text
//!   index(t, r, c) = t·rows·cols + r·cols + c        tray-major, row-major
//!   qpos:  [ boxes 0..n | crawfish 0..n | shells 0..n ] × 7 values, then the crane
//!   names: box_r_c on tray 0 (every gate's names), box_t_r_c above it
//! ```

use mast::SceneMeta;
use rookery_core::CellId;

/// Every cell, tray-major then row-major: the builder's order, and so the
/// World's and qpos's.
pub fn all(meta: &SceneMeta) -> Vec<CellId> {
    let per = meta.rows * meta.cols;
    (0..meta.trays.max(1) * per)
        .map(|i| CellId {
            tray: (i / per) as u8,
            r: (i % per / meta.cols) as u8,
            c: (i % meta.cols) as u8,
        })
        .collect()
}

/// How many cells the farm has.
pub fn count(meta: &SceneMeta) -> usize {
    meta.trays.max(1) * meta.rows * meta.cols
}

/// Where `cell` is in [`all`] (and its box's block in qpos, times 7).
pub fn index(meta: &SceneMeta, cell: CellId) -> usize {
    usize::from(cell.tray) * meta.rows * meta.cols
        + usize::from(cell.r) * meta.cols
        + usize::from(cell.c)
}

/// The name suffix of `cell`: `r_c` on tray 0, `t_r_c` above it.
pub fn tag(cell: CellId) -> String {
    if cell.tray == 0 {
        format!("{}_{}", cell.r, cell.c)
    } else {
        format!("{}_{}_{}", cell.tray, cell.r, cell.c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_rack_is_tray_major_and_names_follow_the_builder() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../scene/generated/rack3.json"
        ))
        .unwrap();
        let m = SceneMeta::from_json(&text).unwrap();
        let cells = all(&m);
        assert_eq!(cells.len(), 12);
        assert_eq!(count(&m), 12);
        assert_eq!(
            cells[5],
            CellId {
                tray: 1,
                r: 0,
                c: 1
            }
        );
        for (i, c) in cells.iter().enumerate() {
            assert_eq!(index(&m, *c), i);
        }
        assert_eq!(tag(cells[2]), "0_2");
        assert_eq!(tag(cells[5]), "1_0_1");
    }

    #[test]
    fn the_rack_fills_end_slots_first_and_blocks_only_bottom_rows() {
        use crate::bt_hooks::{eta, row_blocked, slot_order};
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../scene/generated/rack3.json"
        ))
        .unwrap();
        let m = SceneMeta::from_json(&text).unwrap();
        let order = slot_order(&m);
        assert_eq!(
            &order[..2],
            &[4, 5],
            "the ±0.375 m slots block no row: {order:?}"
        );
        // A box resting in slot 0 (y = -0.075) blocks bottom-shelf cell (0, 0, 1)
        // (y = -0.075) only.
        let mut q = vec![0.0; 7 * 36 + 4];
        let s0 = m.derived.exit_slots[0];
        let boxed = CellId {
            tray: 2,
            r: 0,
            c: 3,
        };
        let i = 7 * index(&m, boxed);
        q[i..i + 3].copy_from_slice(&s0);
        let slots = [(boxed, 0)];
        assert!(row_blocked(
            &q,
            &m,
            &slots,
            CellId {
                tray: 0,
                r: 0,
                c: 1
            }
        ));
        assert!(!row_blocked(
            &q,
            &m,
            &slots,
            CellId {
                tray: 0,
                r: 0,
                c: 0
            }
        ));
        assert!(
            !row_blocked(
                &q,
                &m,
                &slots,
                CellId {
                    tray: 1,
                    r: 0,
                    c: 1
                }
            ),
            "upper shelves pass above"
        );
        let e = eta(
            mast::kinematics::home(),
            CellId {
                tray: 2,
                r: 0,
                c: 0,
            },
            &m,
        );
        assert!(e.is_finite() && e > 0.0);
    }
}
