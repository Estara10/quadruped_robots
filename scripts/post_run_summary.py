#!/usr/bin/env python3
"""Post-run summary CLI for a saved run record (P1-09 runtime record).

Reads ONLY the per-run record file produced by ``record_runtime_run.py`` /
``run_record.RunRecordRecorder`` and reports the summary computed from that
record. It never inspects live shared memory and never produces a formal VALID
artifact. Result determination comes from the record itself.
"""

from __future__ import annotations

import argparse
import json
import sys

from run_record import report_record, summarize_record


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", help="saved per-run record path (.jsonl)")
    parser.add_argument("--json", action="store_true", help="emit the summary as JSON instead of the text report")
    args = parser.parse_args()
    if args.json:
        print(json.dumps(summarize_record(args.record), indent=2, sort_keys=True))
        return
    report_record(args.record)


if __name__ == "__main__":
    main()
