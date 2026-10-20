"""Validate complete, parseable fault history produced by the native fixture."""
import json
import pathlib
import sys

files = list(pathlib.Path(sys.argv[1]).glob("fault-*.jsonl"))
assert len(files) == 1
rows = [json.loads(line) for line in files[0].read_text().splitlines()]
assert rows[0]["schema"] == "polymetis.franka.flight/v1"
assert rows[0]["reason"] == 'test "quoted"\nreason'
assert rows[0]["samples"] == 2
assert [row["robot_state"]["q"][0] for row in rows[1:]] == [2, 3]
assert all(row["period_ms"] == 1 and row["complete"] for row in rows[1:])
assert rows[1]["host_monotonic_ns"] <= rows[2]["host_monotonic_ns"]
print("Fault JSON, ring order, bounded capacity and timing fields verified")
