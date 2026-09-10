"""Read opt-in, truncation-proof full-task records without reward inference."""
import json
import math
from pathlib import Path

STAGES = ("picked", "placed_v2", "contact", "contact_push", "slide_success", "nested")
# Only implications guaranteed by the actual predicates. placed_v2 is NOT a
# prerequisite of bare contact or nesting; contact_push uses a different side
# test from bare contact, so neither contact predicate implies the other.
IMPLICATIONS = (("contact", "picked"), ("contact_push", "picked"),
                ("nested", "contact"), ("slide_success", "picked"))


def read_records(path, stage, online=False):
    if stage not in STAGES:
        raise ValueError("Unknown within-episode stage: " + stage)
    origin = 0
    if online:
        contract = json.loads((Path(path).parent / 'step_contract.json').read_text())
        origin = int(contract['prefill_counter_origin'])
        if origin < 0:
            raise ValueError('Invalid online step origin')
    steps, values = [], []
    with open(path) as handle:
        for line_no, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_no}: incomplete/malformed JSON; use a closed snapshot") from exc
            if "episode/score" not in row:
                continue
            prefix = "episode/train_ep_"
            if row.get(prefix + "record_valid") != 1:
                raise ValueError(f"{path}:{line_no}: missing/invalid episode record; never substitute reward or zero")
            hits = {k: row.get(prefix + k) for k in STAGES}
            if any(v not in (0, 1) for v in hits.values()):
                raise ValueError(f"{path}:{line_no}: incomplete or non-binary stages")
            for harder, easier in IMPLICATIONS:
                if hits[harder] > hits[easier]:
                    raise ValueError(f"{path}:{line_no}: {harder} without {easier}")
            step = float(row["step"]) - origin
            if not math.isfinite(step) or step < 0:
                raise ValueError(f"{path}:{line_no}: invalid step")
            steps.append(step)
            values.append(float(hits[stage]))
    return steps, values
