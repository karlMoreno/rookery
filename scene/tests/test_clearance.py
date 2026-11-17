"""Row 5.1: hook and handle parameters, contact settings, and the clearance check."""

import mujoco
import pytest

from scene import ContactParams, SceneSpec, SpecError
from scene.builder import build_scene, spec_from_json, spec_json


def test_the_default_hook_clears_the_handle_by_at_least_4_mm():
    s = SceneSpec()
    assert s.hook_envelope == pytest.approx(0.023)
    assert s.hook_clearance >= 0.004


def test_the_engaged_hook_clears_box_and_handle_in_the_mjcf():
    """At the engage height (envelope centred in the gap), the finger's
    underside is above the box top and the tip's top is below the handle's
    underside, each by at least 4 mm, measured on the built model."""
    s = SceneSpec()
    m = mujoco.MjModel.from_xml_string(build_scene(s))
    g = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
    handle_r = m.geom_size[g("handle_1_1")][0]
    hook_r = m.geom_size[g("hook")][0]
    box_top = s.box_size[2]
    handle_bottom = s.handle_top_z - 2 * handle_r
    centre = (box_top + handle_bottom) / 2
    finger_z = centre - s.hook_envelope / 2 + hook_r
    assert (finger_z - hook_r) - box_top >= 0.004 - 1e-9
    assert handle_bottom - (finger_z + s.hook_tip_rise + hook_r) >= 0.004 - 1e-9


def test_a_handle_too_low_for_the_hook_is_refused():
    with pytest.raises(SpecError) as e:
        SceneSpec(handle_height=0.03)
    assert any("hook" in p and "handle_height" in p for p in e.value.problems)


def test_a_hook_finger_longer_than_half_the_box_is_refused():
    with pytest.raises(SpecError):
        SceneSpec(hook_finger=0.07)


def test_default_contact_writes_nothing():
    xml = build_scene(SceneSpec())
    for attr in ("condim=", "solref=", "solimp=", "friction="):
        assert attr not in xml


def test_contact_params_land_on_hook_handles_and_box_floors_only():
    c = ContactParams(condim=4, friction=(1.0, 0.01, 0.001), solref=(0.004, 1.0),
                      solimp=(0.9, 0.95, 0.001, 0.5, 2.0))
    m = mujoco.MjModel.from_xml_string(build_scene(SceneSpec(contact=c)))
    g = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)  # noqa: E731
    for n in ("hook", "hook_tip", "handle_0_0", "box_3_3_floor"):
        assert m.geom_condim[g(n)] == 4, n
        assert m.geom_solref[g(n)][0] == pytest.approx(0.004), n
        assert m.geom_friction[g(n)][0] == pytest.approx(1.0), n
    assert m.geom_condim[g("box_0_0_wall_xneg")] == 3  # untouched


@pytest.mark.parametrize("bad", [ContactParams(condim=2), ContactParams(solref=(0.01,)),
                                 ContactParams(friction=(1.0, 0.0))])
def test_bad_contact_params_are_refused(bad):
    with pytest.raises(SpecError):
        SceneSpec(contact=bad)


def test_contact_round_trips_through_json():
    s = SceneSpec(contact=ContactParams(condim=4, solref=(0.004, 1.0)))
    assert spec_from_json(spec_json(s)) == s
