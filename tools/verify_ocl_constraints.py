#!/usr/bin/env python3
"""Run the complete executable check for Table 1's OCL constraints."""

from __future__ import annotations

import json
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.ocl_constraints import CONSTRAINT_IDS, evaluate_constraints, paper_example


def table_ids() -> tuple[str, ...]:
    text = (ROOT / "sections" / "03_metamodels.tex").read_text(encoding="utf-8")
    start = text.index(r"\label{tbl:ocl}")
    end = text.index(r"\end{table*}", start)
    ids = re.findall(r"\\textbf\{([IP]\d+)\}", text[start:end])
    return tuple(ids)


def use_model_ids() -> tuple[str, ...]:
    text = (ROOT / "verification" / "ocl" / "crash2openx.use").read_text(
        encoding="utf-8"
    )
    found = set(re.findall(r"\bcontext\s+\w+\s+inv\s+([IP]\d+)\s*:", text))
    return tuple(identifier for identifier in CONSTRAINT_IDS if identifier in found)


def main() -> int:
    discovered = table_ids()
    native_discovered = use_model_ids()
    if discovered != CONSTRAINT_IDS or native_discovered != CONSTRAINT_IDS:
        print(
            "FAIL: paper/checker/USE constraint catalogs differ: "
            f"paper={discovered}, checker={CONSTRAINT_IDS}, "
            f"USE={native_discovered}",
            file=sys.stderr,
        )
        return 1

    suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"), pattern="test_ocl_constraints.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    road, scene = paper_example()
    outcomes = evaluate_constraints(road, scene)
    summary = {
        "catalog_match": True,
        "catalogs": ["paper_table", "python_oracle", "use_ocl_model"],
        "constraint_count": len(discovered),
        "paper_example": outcomes,
        "tests_run": result.testsRun,
        "tests_successful": result.wasSuccessful(),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if result.wasSuccessful() and all(outcomes.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
