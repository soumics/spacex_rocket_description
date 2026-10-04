"""Physical sanity of generated vehicles. Run: python3 -m pytest tools/tests"""
from pathlib import Path

import numpy as np
import pytest
import yaml

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import vehicle_model as vm  # noqa: E402

VEH_DIR = Path(__file__).resolve().parents[2] / "vehicles"
SPECS = sorted(VEH_DIR.glob("*.yaml"))


@pytest.fixture(params=SPECS, ids=[p.stem for p in SPECS])
def vehicle(request):
    return vm.build(request.param)


def test_every_link_inertia_is_physical(vehicle):
    for l in vehicle.links:
        if l.inertial:
            assert l.inertial.m > 0, l.name
            assert not vm.check_inertia(l.inertial), (l.name, vm.check_inertia(l.inertial))


def test_mass_budget_matches_spec(vehicle):
    s = vehicle.spec
    expected = sum(st["dry_mass"] + st["propellant_mass"] for st in s["stages"]) \
        + s["fairing"]["mass"] + s["payload"]["mass"]
    assert vehicle.total_mass() == pytest.approx(expected, rel=1e-9)


def test_gross_mass_within_5pct_of_published(vehicle):
    ref = vehicle.spec["reference"]["gross_liftoff_mass"]
    assert abs(vehicle.total_mass() - ref) / ref < 0.05


def test_tanks_not_overfilled(vehicle):
    for st, rep in vehicle.report.items():
        for tank in ("lox_tank", "rp1_tank"):
            assert 0.9 < rep[tank]["fill"] <= 1.0, (st, tank, rep[tank]["fill"])


def test_stack_height_matches_published(vehicle):
    s = vehicle.spec
    eng = s["engine_models"][s["stages"][0]["engines"]["model"]]
    bottom = s["stages"][0]["engines"]["gimbal_z"] - eng["length"]
    top = s["fairing"]["base_z"] + s["fairing"]["height"]
    assert top - bottom == pytest.approx(s["reference"]["total_height"], abs=0.2)


def test_com_on_axis_and_below_mid_height(vehicle):
    B = vm.vehicle_frame_com(vehicle)
    assert np.hypot(B.c[0], B.c[1]) < 1e-3
    # fully fuelled: dense propellant low in the stack keeps CoM below half height
    assert B.c[2] < vehicle.spec["reference"]["total_height"] / 2


def test_structure_mass_positive(vehicle):
    for st, rep in vehicle.report.items():
        assert rep["structure"] > 0, st


def _walk_numbers(node, path=""):
    """Yield (path, value) for every leaf; strings that look numeric are a YAML trap (e.g. 845.0e3)."""
    if isinstance(node, dict):
        for k, v in node.items():
            yield from _walk_numbers(v, f"{path}.{k}")
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _walk_numbers(v, f"{path}[{i}]")
    else:
        yield path, node


def test_no_numbers_parsed_as_strings(vehicle):
    import re
    for path, val in _walk_numbers(vehicle.spec):
        if isinstance(val, str):
            assert not re.fullmatch(r"[-+]?\d+(\.\d*)?([eE][-+]?\d+)?", val.strip()), \
                f"{path} = {val!r} parsed as a string (write exponents as e+3 or plain digits)"
