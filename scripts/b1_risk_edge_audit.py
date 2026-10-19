#!/usr/bin/env python3
"""Rebuild a compact, independent tri-state risk audit for the original B1 runs."""
from __future__ import annotations

import glob
import json
from pathlib import Path

from s1_analyze_run import risk_edge_counts


ROOT = Path(__file__).resolve().parents[1]


def audit_run(record: Path) -> dict:
    frames = []
    with record.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("kind") == "frame" and isinstance(row.get("payload"), dict):
                frames.append(row["payload"])
    counts = risk_edge_counts(frames)
    transitions = []
    paired = 0
    for prev, cur in zip(frames, frames[1:]):
        before, after = prev.get("mode_after"), cur.get("mode_after")
        if before in (0, 1) and after in (0, 1) and before != after:
            transitions.append((before, after, cur))
            if (before == 0 and after == 1 and cur.get("risk_condition_entered") == 1):
                paired += 1
    metrics_path = record.with_name("metrics.json")
    if not metrics_path.exists():
        metrics_path = record.parent / "metrics.json"
    metrics = json.loads(metrics_path.read_text(encoding="utf-8")) if metrics_path.exists() else {}
    response = metrics.get("risk_to_recovery_response", {})
    return {"run": record.parents[3].name,
            "frames": len(frames), **counts,
            "risk_condition_met_true": sum(p.get("risk_condition_met") is True for p in frames),
            "agile_to_recovery": sum(a == 0 and b == 1 for a, b, _ in transitions),
            "recovery_to_agile": sum(a == 1 and b == 0 for a, b, _ in transitions),
            "paired_risk_to_agile_recovery": paired,
            "metrics_risk_edge_count": response.get("risk_edge_count"),
            "metrics_paired_count": response.get("paired_Agile_to_Recovery_count"),
            "metrics_match": (response.get("risk_edge_count") == counts["true"] and
                              response.get("paired_Agile_to_Recovery_count") == paired)}


def main() -> None:
    records = sorted(Path(p) for p in glob.glob(str(ROOT / "logs/b1/runs/*/*/rep_01/*/runtime_record.jsonl")))
    rows = [audit_run(path) for path in records]
    lines = ["# B1 风险枚举与配对复核（原8条派生）", "",
             "原始 JSONL 未修改。边沿仅按枚举值 `== 1` 计为 true；`0`、`2`分别列为 false、UNKNOWN。模式边沿只按相邻保存帧核验；配对要求同一帧记录真实风险进入且该帧为 Agile→Recovery。内部延迟来自既有 production metrics，仅表示控制器内部决策间隔。", "",
             "| run | 帧数 | edge true | false | UNKNOWN | invalid | risk_condition_met=true帧 | 保存帧 A→R / R→A | 配对风险→A→R | metrics风险/配对 | 一致 |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for row in rows:
        lines.append(f"| {row['run']} | {row['frames']} | {row['true']} | {row['false']} | {row['unknown']} | {row['invalid']} | {row['risk_condition_met_true']} | {row['agile_to_recovery']} / {row['recovery_to_agile']} | {row['paired_risk_to_agile_recovery']} | {row['metrics_risk_edge_count']} / {row['metrics_paired_count']} | {'是' if row['metrics_match'] else '否/无metrics'} |")
    lines += ["", "本表只说明原8条中已持久化帧的计数与配对，不补缺失周期；risk事件完整性仍受本轮周期历史修复前的记录边界限制。"]
    output = ROOT / "docs/thesis_project/evidence/B1/B1_RISK_EDGE_AUDIT.md"
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {output} ({len(rows)} runs); all metrics match={all(row['metrics_match'] for row in rows)}")


if __name__ == "__main__":
    main()
