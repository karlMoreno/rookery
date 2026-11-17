//! Row 7.3: the molt state machine.

use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rookery_core::{CellId, Crawfish, DAY, Event, HOUR, LifecycleParams, MoltStage, World};

fn cell(i: u8) -> CellId {
    CellId {
        tray: 0,
        r: i / 4,
        c: i % 4,
    }
}

#[test]
fn the_cycle_visits_every_stage_in_order() {
    let p = LifecycleParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(1);
    let mut a = Crawfish::new(&p, &mut rng);
    let mut seen = vec![a.stage];
    for _ in 0..(60.0 * DAY / (10.0 * 60.0)) as usize {
        a.step(600.0, &p, &mut rng);
        if *seen.last().unwrap() != a.stage {
            seen.push(a.stage);
        }
    }
    let cycle = [
        MoltStage::Intermolt,
        MoltStage::Premolt,
        MoltStage::Molting,
        MoltStage::SoftShell,
        MoltStage::Hardened,
    ];
    for w in seen.windows(2) {
        assert_eq!(w[0].next(), w[1], "{seen:?}");
    }
    assert!(cycle.iter().all(|s| seen.contains(s)), "{seen:?}");
}

#[test]
fn molted_is_reported_when_premolt_ends_even_inside_one_long_step() {
    let p = LifecycleParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(2);
    let mut a = Crawfish {
        stage: MoltStage::Premolt,
        t_in_stage: 0.0,
        next_transition: HOUR,
    };
    // A 3 h step crosses Premolt→Molting (Molted) and maybe Molting→SoftShell.
    assert!(matches!(
        a.step(3.0 * HOUR, &p, &mut rng),
        Some(Event::Molted(_))
    ));
}

#[test]
fn the_window_closing_comes_when_soft_shell_ends() {
    let p = LifecycleParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(3);
    let mut a = Crawfish {
        stage: MoltStage::SoftShell,
        t_in_stage: 0.0,
        next_transition: HOUR,
    };
    assert!(matches!(
        a.step(1.5 * HOUR, &p, &mut rng),
        Some(Event::WindowClosing(_))
    ));
    assert_eq!(a.stage, MoltStage::Hardened);
}

#[test]
fn draws_are_positive_with_the_configured_mean() {
    let p = LifecycleParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(4);
    let n = 20_000;
    let xs: Vec<f64> = (0..n)
        .map(|_| p.draw(MoltStage::Intermolt, &mut rng))
        .collect();
    assert!(xs.iter().all(|&x| x > 0.0));
    let mean = xs.iter().sum::<f64>() / n as f64;
    assert!(
        (mean - p.intermolt_mean).abs() < 0.02 * p.intermolt_mean,
        "{}",
        mean / DAY
    );
}

#[test]
fn the_same_seed_gives_the_same_history_and_events_carry_their_cell() {
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let run = |seed| {
        let mut w = World::new(LifecycleParams::default(), seed, &cells);
        (0..24 * 60).flat_map(|_| w.step(HOUR)).collect::<Vec<_>>()
    };
    let a = run(9);
    assert_eq!(a, run(9));
    assert_ne!(a, run(10));
    assert!(a.iter().all(|e| match e {
        Event::Molted(c) | Event::WindowClosing(c) => cells.contains(c),
        _ => true,
    }));
}

#[test]
fn the_bus_logs_and_notifies_subscribers() {
    use std::cell::RefCell;
    use std::rc::Rc;
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let mut w = World::new(LifecycleParams::default(), 11, &cells);
    let seen = Rc::new(RefCell::new(0usize));
    let s = seen.clone();
    w.bus.subscribe(Box::new(move |_, _| *s.borrow_mut() += 1));
    let mut returned = 0;
    for _ in 0..24 * 60 {
        returned += w.step(HOUR).len();
    }
    assert!(returned > 0);
    assert_eq!(*seen.borrow(), returned);
    assert_eq!(w.bus.log().len(), returned);
}

#[test]
fn fast_forward_and_fine_steps_give_each_cell_the_same_history() {
    // Row 7.2: jumping to the next transition (fast-forward) and stepping
    // every minute must produce the same events per cell, because each
    // animal draws from its own random stream.
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let horizon = 20.0 * DAY;
    let per_cell = |events: Vec<Event>| {
        let mut m: std::collections::BTreeMap<CellId, Vec<&'static str>> = Default::default();
        for e in events {
            let (c, k) = match e {
                Event::Molted(c) => (c, "molted"),
                Event::WindowClosing(c) => (c, "window"),
                _ => continue,
            };
            m.entry(c).or_default().push(k);
        }
        m
    };
    let mut fine = World::new(LifecycleParams::default(), 5, &cells);
    let mut fine_events = Vec::new();
    for _ in 0..(horizon / 60.0) as usize {
        fine_events.extend(fine.step(60.0));
    }
    let mut ff = World::new(LifecycleParams::default(), 5, &cells);
    let (mut t, mut ff_events) = (0.0, Vec::new());
    while t < horizon {
        let dt = (ff.time_to_next_transition() + 1e-6).min(horizon - t);
        ff_events.extend(ff.step(dt));
        t += dt;
    }
    let (a, b) = (per_cell(fine_events), per_cell(ff_events));
    assert!(!a.is_empty());
    assert_eq!(a, b);
}
