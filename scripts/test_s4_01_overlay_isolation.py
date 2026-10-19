#!/usr/bin/env python3
"""Targeted check of the production manual diagnostic overlay preparation."""

import importlib.util
import tempfile
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("manual_ab_compare.py")
SPEC = importlib.util.spec_from_file_location("manual_ab_compare", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def main():
    with tempfile.TemporaryDirectory(prefix="s4_01_overlay_") as temporary:
        root = Path(temporary)
        source = root / "installed"
        config = source / "share/go2_description/config/abs/config.yaml"
        config.parent.mkdir(parents=True)
        installed_config = root / "original_config.yaml"
        original = (
            '  switching:\n'
            '    group: "A"\n'
            '    entry_threshold: -0.05\n'
            '  goal_x: 1.0\n'
            '  goal_y: 2.0\n'
        )
        installed_config.write_text(original, encoding="utf-8")
        config.symlink_to(installed_config)

        for group, expected_hold in (("A", None), ("C", "    recovery_min_hold_s: 0.30\n"),
                                     ("D", "    recovery_min_hold_s: 0.30\n")):
            destination = root / f"overlay_{group}"
            MODULE.prepare_overlay(source, destination, group)
            prepared = destination / "share/go2_description/config/abs/config.yaml"
            text = prepared.read_text(encoding="utf-8")
            assert not prepared.is_symlink(), f"{group}: overlay config remains a symlink"
            assert text.count('    group: "' + group + '"') == 1
            assert "    entry_threshold: -0.05\n" in text
            assert (expected_hold is not None) == ("recovery_min_hold_s" in text)
            if expected_hold:
                assert expected_hold in text
                if group == "D":
                    assert "    exit_threshold: -0.08\n" in text
                else:
                    assert "exit_threshold" not in text
            else:
                assert "exit_threshold" not in text
            assert "  goal_x: 7.0\n" in text and "  goal_y: 0.0\n" in text
            assert prepared.resolve() != installed_config.resolve()
            assert installed_config.read_text(encoding="utf-8") == original

    print("S4-01 overlay isolation checks PASS")


if __name__ == "__main__":
    main()
