//! Rows 7.3 and 7.5: the molt state machine, shells and the world.

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
    let events = a.step(3.0 * HOUR, &p, &mut rng);
    assert!(
        matches!(events.first(), Some(Event::Molted(_))),
        "{events:?}"
    );
}

#[test]
fn the_window_warns_before_it_ends_and_an_animal_left_in_its_box_misses_it() {
    // Row 8.2: WindowClosing is the warning, `window_warning` (1 h) before
    // the soft-shell window ends; WindowMissed is the end itself.
    let p = LifecycleParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(3);
    let mut a = Crawfish {
        stage: MoltStage::SoftShell,
        t_in_stage: 0.0,
        next_transition: 6.0 * HOUR,
    };
    assert!(a.step(4.9 * HOUR, &p, &mut rng).is_empty());
    assert!(
        (a.time_to_next(&p) - 0.1 * HOUR).abs() < 1e-6,
        "the warning is next"
    );
    assert_eq!(
        a.step(0.2 * HOUR, &p, &mut rng),
        vec![Event::WindowClosing(CellId::NONE)]
    );
    assert_eq!(
        a.step(HOUR, &p, &mut rng),
        vec![Event::WindowMissed(CellId::NONE)]
    );
    assert_eq!(a.stage, MoltStage::Hardened);
    // One long step reports both, in order.
    let mut b = Crawfish {
        stage: MoltStage::SoftShell,
        t_in_stage: 0.0,
        next_transition: 6.0 * HOUR,
    };
    assert_eq!(
        b.step(7.0 * HOUR, &p, &mut rng)[..2],
        [
            Event::WindowClosing(CellId::NONE),
            Event::WindowMissed(CellId::NONE)
        ]
    );
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
                Event::ShellEaten(c) => (c, "eaten"),
                Event::WindowMissed(c) => (c, "missed"),
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

#[test]
fn every_shell_is_eaten_shell_delay_after_its_molt() {
    // Row 7.5: Molted starts the shell's clock, and ShellEaten follows
    // exactly `shell_delay` later, to within one step.
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let p = LifecycleParams::default();
    let mut w = World::new(p, 11, &cells);
    let dt = 60.0;
    let mut molted_at = std::collections::HashMap::new();
    let (mut molts, mut eaten) = (0, 0);
    for i in 0..(60.0 * DAY / dt) as usize {
        let t = (i + 1) as f64 * dt;
        for e in w.step(dt) {
            match e {
                Event::Molted(c) => {
                    assert!(
                        molted_at.insert(c, t).is_none(),
                        "a second molt before the shell was eaten"
                    );
                    molts += 1;
                }
                Event::ShellEaten(c) => {
                    let m = molted_at.remove(&c).expect("a shell eaten without a molt");
                    assert!(
                        ((t - m) - p.shell_delay).abs() <= dt,
                        "eaten {} s after the molt",
                        t - m
                    );
                    eaten += 1;
                }
                _ => {}
            }
        }
        for (k, (c, _)) in w.animals().iter().enumerate() {
            assert_eq!(w.has_shell(k), molted_at.contains_key(c));
        }
    }
    assert!(
        molts >= 16 && eaten >= molts - 16,
        "{molts} molts, {eaten} eaten"
    );
}

#[test]
fn fast_forward_stops_for_a_shell_being_eaten() {
    let cells = [cell(0)];
    let p = LifecycleParams::default();
    let mut w = World::new(p, 3, &cells);
    // Jump until the molt.
    while !w
        .step(w.time_to_next_transition() + 1e-6)
        .contains(&Event::Molted(cell(0)))
    {}
    assert!(w.has_shell(0));
    // From here the next things due are Molting's end (an hour), the soft
    // window... and the shell, a day after the molt. Fast-forward never
    // jumps past it.
    let mut t = 0.0;
    loop {
        let dt = w.time_to_next_transition() + 1e-6;
        t += dt;
        if w.step(dt).contains(&Event::ShellEaten(cell(0))) {
            break;
        }
        assert!(t < p.shell_delay, "jumped past the shell");
    }
    assert!((t - p.shell_delay).abs() < 1e-3, "eaten at {t}");
    assert!(!w.has_shell(0));
}

#[test]
fn every_cell_has_water_that_stays_in_bounds_for_a_year() {
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let mut w = World::new(LifecycleParams::default(), 2, &cells);
    let mut t = 0.0;
    while t < 365.0 * DAY {
        let dt = w.time_to_next_transition() + 1e-6;
        w.step(dt);
        t += dt;
    }
    let p = w.bio_params;
    assert_eq!(w.bio().len(), 16);
    for b in w.bio() {
        assert!(
            (0.0..=p.o2_sat).contains(&b.o2()) && (0.0..=p.waste_max).contains(&b.waste()),
            "{b:?}"
        );
        // With one animal per cell, oxygen settles at its equilibrium.
        assert!(
            (b.o2() - (p.o2_sat - p.o2_uptake / p.k_air)).abs() < 1e-6,
            "{b:?}"
        );
    }
}

#[test]
fn a_harvested_cell_is_lifted_once_and_never_misses_its_window() {
    use rookery_core::{CellObs, MoltDetector, OracleDetector};
    let cells = [cell(0), cell(1)];
    let mut w = World::new(LifecycleParams::default(), 9, &cells);
    let mut oracle = OracleDetector;
    let obs = CellObs::default();
    let (mut lifted, mut events) = (false, Vec::new());
    let mut t = 0.0;
    while t < 120.0 * DAY {
        let dt = w.time_to_next_transition() + 1e-6;
        let step = w.step(dt);
        t += dt;
        for e in &step {
            if let Event::Molted(c) = e {
                // Detected from the first step after the molt.
                assert!(oracle.detect(*c, &obs, &w).is_some(), "{c:?} not detected");
                if *c == cell(0) && !lifted {
                    assert_eq!(w.harvest(cell(0)), Some(Event::BoxLifted(cell(0))));
                    assert_eq!(w.harvest(cell(0)), None, "lifted once");
                    assert!(oracle.detect(cell(0), &obs, &w).is_none(), "gone");
                    lifted = true;
                }
            }
        }
        events.extend(step);
    }
    assert!(lifted);
    let ev = |c: CellId, k: fn(CellId) -> Event| events.iter().filter(|e| **e == k(c)).count();
    assert_eq!(
        ev(cell(0), Event::WindowMissed),
        0,
        "a harvested animal never misses"
    );
    assert_eq!(ev(cell(0), Event::Molted), 1, "and never molts again");
    assert!(
        ev(cell(1), Event::WindowMissed) >= 2,
        "cell 1 is never harvested"
    );
    assert_eq!(
        ev(cell(1), Event::WindowMissed),
        ev(cell(1), Event::WindowClosing)
    );
    assert!(
        w.bus
            .log()
            .iter()
            .any(|(_, e)| *e == Event::BoxLifted(cell(0)))
    );
    assert!(w.window_left(0).is_none() && w.is_harvested(0));
}

#[test]
fn detector_kind_parses_oracle_and_camera() {
    use rookery_core::DetectorKind;
    assert_eq!("oracle".parse::<DetectorKind>(), Ok(DetectorKind::Oracle));
    assert_eq!("camera".parse::<DetectorKind>(), Ok(DetectorKind::Camera));
    assert!(
        "lidar"
            .parse::<DetectorKind>()
            .unwrap_err()
            .contains("camera")
    );
}

#[test]
fn a_faster_molt_rate_shortens_the_cycle_but_not_the_window() {
    // Row 12.2.
    let p = LifecycleParams::default();
    let q = p.scaled(3.0);
    assert!((q.intermolt_mean - p.intermolt_mean / 3.0).abs() < 1e-9);
    assert!((q.premolt_mean - p.premolt_mean / 3.0).abs() < 1e-9);
    assert_eq!(
        (q.molting_mean, q.soft_window, q.shell_delay),
        (p.molting_mean, p.soft_window, p.shell_delay)
    );
    assert_eq!(p.scaled(0.0), p);
    assert_eq!(p.scaled(f64::NAN), p);
    // About three times the molts in the same time.
    let cells: Vec<CellId> = (0..16).map(cell).collect();
    let count = |params: LifecycleParams| {
        let mut w = World::new(params, 4, &cells);
        let mut n = 0;
        for _ in 0..(120.0 * DAY / HOUR) as usize {
            n += w
                .step(HOUR)
                .iter()
                .filter(|e| matches!(e, Event::Molted(_)))
                .count();
        }
        n
    };
    let (slow, fast) = (count(p), count(q));
    assert!(fast as f64 > 2.2 * slow as f64, "{slow} vs {fast}");
}
