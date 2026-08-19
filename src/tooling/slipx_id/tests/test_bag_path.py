# Copyright 2026 The SlipX Authors
# SPDX-License-Identifier: Apache-2.0

"""The self-test, end to end through the real bag path (M6.3).

Synthetic manoeuvres become actual rosbag2 recordings on disk, a session
file names them the way a team's would, the staged fits run off the bags,
and the emitted car directory loads back through the ordinary loader. The
assertions at the end are against the numbers the synthetic car was built
from, so the whole chain is on trial: encoder, container, decoder,
reconstruction, fits, emission, schema, loader.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest
import yaml

import slipx
from slipx_id import emit, rosbag, session, synthetic


def _write_session(tmp_path: Path, provenance: dict) -> Path:
    params = slipx.VehicleParams()  # current-limited launch, deliberately

    bags = {}
    bags["coast_high"] = synthetic.coastdown(params, 9.0, duration=6.0)
    bags["coast_low"] = synthetic.coastdown(params, 4.0, duration=6.0)
    bags["launch"] = synthetic.launch(params, duration=5.0)
    for speed in (1.5, 2.5, 3.5):
        bags[f"skidpad_{speed:g}"] = synthetic.skidpad(
            params, speed=speed, steer=0.107, duration=6.0
        )
    bags["ramp_left"] = synthetic.ramp_steer(params, 3.0, rate=0.01, peak=0.35)
    bags["ramp_right"] = synthetic.ramp_steer(params, 3.0, rate=-0.01, peak=0.35)
    heavy = synthetic.ballasted(params, 0.35)
    bags["circle_heavy"] = synthetic.ramp_steer(heavy, 3.0, rate=0.01, peak=0.35)
    bags["step_slow"] = synthetic.step_steer(params, 2.0, 0.10)
    bags["step_fast"] = synthetic.step_steer(params, 6.0, 0.05)
    bags["step_big"] = synthetic.step_steer(params, 5.0, 0.25)
    bags["slalom"] = synthetic.slalom(params, 3.0, amplitude=0.15, duration=6.0)

    for name, recording in bags.items():
        rosbag.write_recording(recording, tmp_path / name)

    manoeuvres = [
        {"bag": "coast_high", "role": "coastdown"},
        {"bag": "coast_low", "role": "coastdown"},
        {"bag": "launch", "role": "launch"},
        {"bag": "skidpad_1.5", "role": "skidpad"},
        {"bag": "skidpad_2.5", "role": "skidpad"},
        {"bag": "skidpad_3.5", "role": "skidpad"},
        {"bag": "ramp_left", "role": "ramp_steer"},
        {"bag": "ramp_right", "role": "ramp_steer"},
        {"bag": "circle_heavy", "role": "circle_to_slip", "ballast_mass": 0.35},
        {"bag": "step_slow", "role": "step_steer"},
        {"bag": "step_fast", "role": "step_steer"},
        {"bag": "step_big", "role": "step_steer"},
    ]

    document = {
        "car": {
            "name": "bag_path_car",
            "surface": "carpet",
            "compound": "sponge",
            "bench": {
                "mass": params.mass,
                "lf": params.lf,
                "lr": params.lr,
                "h_cog": params.h_cog,
                "track_front": params.track_front,
                "track_rear": params.track_rear,
                "wheel_radius": params.wheel_radius,
                "izz": params.izz,
                "length": 0.55,
                "width": 0.30,
            },
        },
        "fixed": {
            "ixx": params.ixx,
            "iyy": params.iyy,
            "steer_max": params.steer_max,
            "steer_rate_max": params.steer_rate_max,
            "accel_max": params.accel_max,
            "decel_max": params.decel_max,
            "torque_per_amp": params.torque_per_amp,
            "drive_efficiency": params.drive_efficiency,
            "pack_nominal_v": params.pack_nominal_v,
            "pack_v_full": params.pack_v_full,
            "pack_v_empty": params.pack_v_empty,
            "pack_capacity_ah": params.pack_capacity_ah,
            "pack_internal_resistance": params.pack_internal_resistance,
            "regen_current_max": params.regen_current_max,
            "v_max": params.v_max,
            "layout": "2WD_rear",
            "differential": "open",
        },
        "topics": {
            "pose": "/pose",
            "imu": "/imu",
            "wheels": "/joint_states",
            "drive": "/drive",
            "wheel_names": {
                "FL": "wheel_front_left",
                "FR": "wheel_front_right",
                "RL": "wheel_rear_left",
                "RR": "wheel_rear_right",
            },
        },
        "manoeuvres": manoeuvres,
        "validation": [{"bag": "slalom"}],
        "provenance": provenance,
        "output": "fitted_car",
    }
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


PROVENANCE = {
    "contributor": "The SlipX Authors (synthetic self-test)",
    "source": "Synthetic recordings generated from known parameters",
    "method": "The manoeuvre library of docs/identification, simulated",
    "date": "2026-08-19",
}


@pytest.fixture(scope="module")
def emitted(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("bagpath")
    spec = session.load_session(_write_session(tmp_path, PROVENANCE))
    outcome = session.run_session(spec, sample_stride=3)
    result = emit.emit_car_directory(outcome)
    return outcome, result, spec


def test_the_emitted_car_loads_and_reaches_l2(emitted) -> None:
    outcome, result, spec = emitted
    car = slipx.load_car(result.directory)
    assert "IDENTIFIED" in car.summary(), "the label leads, printed not implied"
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)
    assert params.mass == pytest.approx(3.5)


def test_the_fitted_numbers_survive_the_files(emitted) -> None:
    # Through emission, schema validation, the loader and the ADR-0039
    # restatement (which is the identity here: the files are stated at this
    # car's own static loads).
    outcome, result, spec = emitted
    truth = slipx.VehicleParams()
    car = slipx.load_car(result.directory)
    params = car.params_for_tier(slipx.Tier.L2_DoubleTrack)

    assert params.c_alpha_f == pytest.approx(truth.c_alpha_f, rel=0.03)
    assert params.c_alpha_r == pytest.approx(truth.c_alpha_r, rel=0.03)
    assert params.tyre_front.mu_y0 == pytest.approx(
        truth.tyre_front.mu_y0, rel=0.03
    )
    assert params.tyre_front.k_mu == pytest.approx(
        truth.tyre_front.k_mu, rel=0.25
    )
    assert params.c_kappa == pytest.approx(truth.c_kappa, rel=0.08)
    assert params.roll_resist == pytest.approx(truth.roll_resist, rel=0.02)
    assert params.drag_coeff == pytest.approx(truth.drag_coeff, rel=0.02)
    # The launch was current limited, so the cap identified the ESC:
    assert params.current_max == pytest.approx(truth.current_max, rel=0.05)
    assert params.v_max == pytest.approx(truth.v_max, rel=0.05)
    # ... and mu_x0 was NOT identifiable, so it must be the provisional
    # default, not a number smeared out of unsaturated data.
    assert params.tyre_front.mu_x0 == truth.tyre_front.mu_x0
    assert any("mu_x0" in note for note in result.notes)
    # The transient stage's numbers survive the trip through float32
    # commands, nanosecond stamps and the YAML files.
    assert params.steer_bandwidth == pytest.approx(
        truth.steer_bandwidth, rel=0.10
    )
    assert params.steer_damping == pytest.approx(
        truth.steer_damping, rel=0.15
    )
    assert params.tyre_front.relax_length == pytest.approx(
        truth.tyre_front.relax_length, rel=0.20
    )


def test_residuals_travel_with_the_car(emitted) -> None:
    outcome, result, spec = emitted
    document = yaml.safe_load(
        (result.directory / "provenance.yaml").read_text(encoding="utf-8")
    )
    assert document["label"] == "identified"
    residuals = document["residuals"]
    for name in ("c_alpha_f", "mu_y0", "k_mu", "roll_resist", "torque_stall"):
        assert name in residuals, name
        assert residuals[name]["stddev"] >= 0.0
    assert "mu_x0" not in residuals, "an unidentified parameter has no residual"


def test_emission_refuses_an_empty_provenance(tmp_path) -> None:
    spec = session.load_session(
        _write_session(tmp_path, {"contributor": "", "source": "x",
                                  "method": "y", "date": "2026-08-19"})
    )
    with pytest.raises(ValueError, match="contributor"):
        emit.check_provenance(spec.provenance)


def test_an_implausible_identified_value_warns(emitted, tmp_path) -> None:
    # Doctor the lateral fit into claiming a peak friction of 2.4 and emit:
    # the read-back through slipx_schema must surface the plausibility
    # warning rather than let the number ship quietly.
    outcome, _, _spec = emitted
    report = outcome.lateral.report
    index = report.names.index("mu_y0")
    values = list(report.values)
    values[index] = 2.4
    doctored_report = dataclasses.replace(report, values=tuple(values))
    doctored = dataclasses.replace(
        outcome,
        lateral=dataclasses.replace(outcome.lateral, report=doctored_report),
        session=dataclasses.replace(
            outcome.session, output=tmp_path / "implausible"
        ),
    )
    result = emit.emit_car_directory(doctored)
    assert any("mu_y0" in warning for warning in result.warnings)


def test_the_cli_refuses_before_it_fits(tmp_path) -> None:
    # A session nobody can emit fails in milliseconds, naming the field,
    # instead of after a minute of optimisation. The bags here do not even
    # exist; the refusal must arrive before anything tries to read them.
    from slipx_id import cli

    document = {
        "car": {
            "name": "x", "surface": "carpet", "compound": "sponge",
            "bench": {
                "mass": 3.5, "lf": 0.16, "lr": 0.16, "h_cog": 0.06,
                "track_front": 0.24, "track_rear": 0.24,
                "wheel_radius": 0.05, "izz": 0.05,
                "length": 0.55, "width": 0.30,
            },
        },
        "fixed": {
            "ixx": 0.025, "iyy": 0.07, "steer_max": 0.4,
            "steer_rate_max": 10.0, "accel_max": 8.0, "decel_max": 12.0,
            "torque_per_amp": 0.01, "drive_efficiency": 0.85,
            "pack_nominal_v": 11.1, "pack_v_full": 12.6,
            "pack_v_empty": 9.9, "pack_capacity_ah": 5.2,
            "pack_internal_resistance": 0.02, "regen_current_max": 40.0,
            "layout": "2WD_rear", "differential": "open",
        },
        "topics": {
            "pose": "/pose", "imu": "/imu", "wheels": "/joint_states",
            "drive": "/drive",
            "wheel_names": {"FL": "a", "FR": "b", "RL": "c", "RR": "d"},
        },
        "manoeuvres": [{"bag": "never_recorded", "role": "coastdown"}],
        "provenance": {"source": "x", "method": "y", "date": "2026-08-19"},
        "output": "out",
    }
    path = tmp_path / "session.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    import time

    started = time.time()
    code = cli.main([str(path)])
    assert code == 2
    assert time.time() - started < 5.0, "the refusal must not wait for a fit"


def test_ballast_changes_the_mass_and_only_the_mass() -> None:
    # The heavy run's reconstruction runs at the ballasted mass; everything
    # else about the bench is the same car. A wrong mass here biases one
    # recording's forces by the ballast fraction and then hides in the
    # ensemble, which is exactly how it escaped a mutation pass once.
    from slipx_id.reconstruct import Bench
    from slipx_id.session import _ballasted_bench

    bench = Bench(3.5, 0.16, 0.16, 0.06, 0.24, 0.24, 0.05, 0.05)
    heavy = _ballasted_bench(bench, 0.35)
    assert heavy.mass == pytest.approx(3.85)
    for name in ("lf", "lr", "h_cog", "track_front", "track_rear",
                 "wheel_radius", "izz"):
        assert getattr(heavy, name) == getattr(bench, name), name


def test_the_validation_report_closes_the_loop(emitted) -> None:
    # The emitted car replays the validation bag the fit never consumed,
    # the report lands beside the car, and the provenance already names it,
    # which is what the registry's acceptance bar checks for.
    from slipx_id import report
    from slipx_id.rosbag import read_recording

    outcome, result, spec = emitted
    recordings = [
        read_recording(bag, spec.bench, topic_map=spec.topics)
        for bag in spec.validation
    ]
    path, worst = report.generate(
        result.directory,
        recordings,
        result.directory / "validation.svg",
        date="2026-08-19",
    )
    assert path.exists()
    assert worst < 6.0, (
        f"the fitted car diverged {worst:.1f}% on a manoeuvre the fit never "
        f"saw"
    )
    provenance = yaml.safe_load(
        (result.directory / "provenance.yaml").read_text(encoding="utf-8")
    )
    assert provenance["validation_report"] == "validation.svg"
    assert "IDENTIFIED" in path.read_text(encoding="utf-8")


def test_the_registry_bar_accepts_the_emitted_car(emitted, tmp_path) -> None:
    # The registry's acceptance bar, pointed at the self-test's own
    # emission: the whole contribution flow in one assertion. The bar is
    # `check_registry_submission` in this tree; the registry repository
    # (github.com/ibrahimsel/slipx_registry) runs a thin CI runner over
    # exactly this check plus the report-file rule replicated below, so
    # what is asserted here is what its CI enforces. The report is
    # (re)generated first because the provenance promises it.
    import shutil

    import yaml

    import slipx
    from slipx_schema.rules import check_registry_submission

    from slipx_id import report
    from slipx_id.rosbag import read_recording

    outcome, result, spec = emitted
    recordings = [
        read_recording(bag, spec.bench, topic_map=spec.topics)
        for bag in spec.validation
    ]
    report.generate(
        result.directory,
        recordings,
        result.directory / "validation.svg",
        date="2026-08-19",
    )

    entry = tmp_path / "cars" / "selftest__bag_path_car__carpet"
    shutil.copytree(result.directory, entry)

    # Loads cleanly, and the tooling prints the label (NFR-08).
    car = slipx.load_car(entry)
    assert "IDENTIFIED" in car.summary()

    provenance = yaml.safe_load(
        (entry / "provenance.yaml").read_text(encoding="utf-8")
    )
    assert check_registry_submission(provenance) == []

    # The runner's one rule beyond the bar: the named report file really is
    # in the entry, not just named.
    named = str(provenance["validation_report"])
    assert (entry / named).is_file()

    # And the bar refuses by name: a submission with no author is a story
    # nobody signed.
    anonymous = dict(provenance)
    anonymous.pop("contributor", None)
    errors = [str(error) for error in check_registry_submission(anonymous)]
    assert any("contributor" in error for error in errors)
