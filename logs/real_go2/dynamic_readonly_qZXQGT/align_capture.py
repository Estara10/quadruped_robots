"""Match camera receive times to the nearest read-only robot state samples."""
import bisect
import csv
import json
from pathlib import Path
from statistics import median

run = Path(__file__).resolve().parent
probes = json.loads((run / "clock.json").read_text())
probe = min(probes, key=lambda p: p["local_after_ns"] - p["local_before_ns"])
offset = ((probe["local_before_ns"] + probe["local_after_ns"]) / 2 - probe["remote_ns"]) / 1e9
network_uncertainty = (probe["local_after_ns"] - probe["local_before_ns"]) / 2e9
states = [json.loads(line) for line in (run / "state.jsonl").read_text().splitlines() if line.startswith("{")]
samples = {
    topic: sorted((state["time"] - state[topic]["age"], index)
                  for index, state in enumerate(states) if state.get(topic))
    for topic in ("low", "sport")
}
sample_times = {topic: [row[0] for row in values] for topic, values in samples.items()}
capture = run / "capture"
meta = dict(line.split("=", 1) for line in (capture / "metadata.txt").read_text().splitlines())
start = float(meta["capture_start_steady_s"]) + offset
with (capture / "frames.csv").open() as stream:
    frames = list(csv.DictReader(stream))
gaps = {topic: [] for topic in samples}
with (run / "aligned_index.jsonl").open("w") as output:
    for frame in frames:
        time = start + float(frame["host_s"])
        matches = {}
        for topic, times in sample_times.items():
            i = bisect.bisect_left(times, time)
            options = samples[topic][max(0, i - 1):i + 1]
            if options:
                sample_time, index = min(options, key=lambda sample: abs(sample[0] - time))
                gap = abs(sample_time - time)
                if gap <= .10:
                    matches[topic] = dict(state_index=index, signed_gap_ms=(sample_time - time) * 1000)
                    gaps[topic].append(gap * 1000)
        output.write(json.dumps(dict(frame=int(frame["i"]), time_pc_steady_s=time, matches=matches)) + "\n")
summary = dict(camera_frames=len(frames), state_snapshots=len(states),
               estimated_clock_rtt_half_ms=network_uncertainty * 1000,
               note="Host receipt times; this is not hardware-synchronized sensor sampling.",
               matched={topic: dict(frames=len(values), median_gap_ms=median(values) if values else None,
                                    max_gap_ms=max(values) if values else None)
                        for topic, values in gaps.items()})
(run / "alignment.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
