"""Row 7.7: scene.plots molt_hist."""

import json

import pytest

from scene import plots

DAY = 86_400.0


def summary(tmp_path, molts, mean=29.0, sigma=4.0):
    events = [{"t": t * DAY, "kind": k, "tray": 0, "r": r, "c": c} for (t, k, r, c) in molts]
    s = {"events": events, "lifecycle": {"interval_mean": mean * DAY, "interval_sigma": sigma * DAY, "seed": 1}}
    (tmp_path / "summary.json").write_text(json.dumps(s))
    return tmp_path


def test_intervals_are_per_cell_and_ignore_other_events(tmp_path):
    run = summary(tmp_path, [(1, "Molted", 0, 0), (2, "Molted", 0, 1), (30, "Molted", 0, 0),
                             (31, "ShellEaten", 0, 0), (33, "Molted", 0, 1), (61, "Molted", 0, 0)])
    xs = sorted(x / DAY for x in plots.intervals(json.loads((run / "summary.json").read_text())))
    assert xs == pytest.approx([29, 31, 31])


def test_molt_hist_writes_the_same_png_twice_and_reports_the_stats(tmp_path):
    molts = [(t, "Molted", 0, c) for c in range(4) for t in (c, c + 28, c + 58, c + 87)]
    run = summary(tmp_path, molts)
    out, line = plots.molt_hist(run)
    first = out.read_bytes()
    assert first[:8] == b"\x89PNG\r\n\x1a\n"
    assert "12 intervals" in line and "configured 29.00" in line
    plots.molt_hist(run)
    assert out.read_bytes() == first, "same input, same bytes"


def test_too_few_intervals_or_an_old_summary_is_an_error(tmp_path):
    run = summary(tmp_path, [(1, "Molted", 0, 0)])
    with pytest.raises(ValueError, match="run longer"):
        plots.molt_hist(run)
    (tmp_path / "summary.json").write_text(json.dumps({"events": []}))
    with pytest.raises(ValueError, match="no lifecycle"):
        plots.molt_hist(tmp_path)
    assert plots.main(["molt_hist", str(tmp_path)]) == 1


def test_the_lognormal_has_the_configured_mean():
    xs = [i * 0.05 for i in range(1, 2000)]
    m = sum(x * plots.lognormal_pdf(x, 29.0, 4.0) * 0.05 for x in xs)
    assert m == pytest.approx(29.0, rel=1e-3)
