"""crash2openx fuzzer — MAP-Elites style scenario mutation on frozen seeds.

Public entry: ``tools.fuzzer.runner.run_fuzz`` and the CLI wrapper
``tools/fuzzer/runner.py``.
"""
from tools.fuzzer.mutator import mutate_scene, MUTATORS
from tools.fuzzer.evaluator import score_offline
from tools.fuzzer.archive import MapElitesArchive

__all__ = ["mutate_scene", "MUTATORS", "score_offline", "MapElitesArchive"]
