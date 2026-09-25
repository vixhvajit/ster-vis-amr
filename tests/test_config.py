import pytest

from amr.config import Config, load


def test_toml_overrides(tmp_path):
    path = tmp_path / "amr.toml"
    path.write_text('max_speed = 0.2\n\n[motors.rear_right]\ninvert = true\nenable = 26\n', encoding="utf-8")
    config = load(path)
    assert config.max_speed == 0.2
    assert config.motors["rear_right"].invert is True
    assert config.motors["rear_right"].enable == 26
    assert config.motors["rear_right"].forward == Config().motors["rear_right"].forward


def test_unknown_setting_is_an_error(tmp_path):
    path = tmp_path / "amr.toml"
    path.write_text("max_sped = 0.2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="max_sped"):
        load(path)


def test_example_config_matches_defaults():
    from pathlib import Path

    example = Path(__file__).parents[1] / "deploy" / "pi" / "amr.toml"
    assert load(example) == Config()
