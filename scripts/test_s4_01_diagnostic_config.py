#!/usr/bin/env python3
"""Targeted S4-01 check for C hold config spelling and startup summaries."""

from s1_run_diagnostic import hold_setting_matches


def main():
    assert hold_setting_matches("    recovery_min_hold_s: 0.30\n", 0.3)
    assert hold_setting_matches("    recovery_min_hold_s: 0.300 # seconds\n", 0.3)
    assert not hold_setting_matches("", 0.3)
    assert not hold_setting_matches("    recovery_min_hold_s: .nan\n", 0.3)
    assert not hold_setting_matches("    recovery_min_hold_s: 0.31\n", 0.3)
    print("S4-01 diagnostic config checks PASS")


if __name__ == "__main__":
    main()
