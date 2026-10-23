"""The bthome_mithermometer alias must produce the same build as the bthome config it stands for."""

from collections.abc import Callable
from pathlib import Path

import pytest

from esphome.config import read_config
from esphome.core import CORE


def test_alias_matches_hand_written_bthome_config(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
    caplog: pytest.LogCaptureFixture,
) -> None:
    alias_main = generate_main(component_config_path("alias.yaml"))
    assert "deprecated, use 'bthome'" in caplog.text
    assert "2027.5.0" in caplog.text
    # The warning carries the pasteable replacement; the key is dumped redacted.
    assert "platform: bthome\n" in caplog.text
    assert "\n      key: " in caplog.text
    assert "bthome_mithermometer" not in CORE.config["sensor"][0]["platform"]
    CORE.reset()

    bthome_main = generate_main(component_config_path("bthome.yaml"))
    assert alias_main == bthome_main
    assert "bthome::client::DeviceListener<1>" in alias_main
    assert "bthome::client::RemoteDevice<5>" in alias_main


def test_alias_joins_existing_bthome_hub(
    generate_main: Callable[[str | Path], str],
    component_config_path: Callable[[str], Path],
) -> None:
    alias_main = generate_main(component_config_path("alias_with_hub.yaml"))
    devices = CORE.config["bthome"]["remote_devices"]
    assert [str(d["id"]) for d in devices] == ["plant", "thermometer"]
    CORE.reset()

    assert alias_main == generate_main(component_config_path("bthome_with_hub.yaml"))


def test_alias_rejects_unknown_option(
    component_config_path: Callable[[str], Path],
    capsys: pytest.CaptureFixture[str],
) -> None:
    CORE.config_path = component_config_path("alias_unknown_key.yaml")
    assert read_config({}) is None
    assert (
        "[pressure] is an invalid option for [sensor.bthome_mithermometer]"
        in capsys.readouterr().out
    )


def test_alias_rejects_hub_binding_conflict(
    component_config_path: Callable[[str], Path],
    capsys: pytest.CaptureFixture[str],
) -> None:
    CORE.config_path = component_config_path("alias_hub_conflict.yaml")
    assert read_config({}) is None
    assert "'ble_hub_id: other_tracker' differs" in capsys.readouterr().out
