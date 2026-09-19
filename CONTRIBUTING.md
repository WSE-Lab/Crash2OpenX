# Contributing

Clone with `git clone --recurse-submodules https://github.com/WSE-Lab/Crash2OpenX.git`,
then run `uv sync --locked` and `uv run --locked python -m pytest -q`.
The static suite needs neither CARLA nor an API key.

Keep scenario changes in the seed model or compiler. Include a small regression
case when changing geometry, actor placement, triggers or runtime semantics.
Describe the source input, expected behavior and actual evidence in bug reports.
Do not include API keys, SSH credentials, private reports or machine-specific
configuration. Copy `.env.example` to `.env.local` for local settings.

For simulation changes, record CARLA and PCLA versions, seed and artifact hashes,
the ADS identity, video, event stream and trajectory. Distinguish a generated test
variant from reconstruction of the source accident. Use a dedicated run directory
on shared machines. If space is insufficient, stop; never delete another user's
files or prune shared Docker resources.
