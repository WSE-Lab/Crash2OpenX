# Third-party components

Crash2OpenX's original code is MIT-licensed. Existing copyright headers remain
in files adapted from other projects. The following components retain their
own licenses; the top-level MIT license does not relicense them.

| Component | Revision | License |
|---|---|---|
| pyoscx/scenariogeneration | debb300ec25c14cf16aae390f399a7311bd209f1 | MPL-2.0 |
| carla-simulator/scenario_runner | 94ff3b8af752bad2b9d464ad5105868906aa34c0 | MIT |
| MasoudJTehrani/PCLA | e3050bd2d83257db3d9db42ef7349e29f93091e1 | Apache-2.0 |

These projects are Git submodules with their license files. The runtime patch
under `runner/patches/` modifies ScenarioRunner and PCLA files. Their existing
headers and licenses continue to apply. `tools.prepare_runtime` preserves each
upstream license in the isolated runtime copy.

CARLA, pretrained ADS weights and model APIs are obtained separately under their
providers' terms. Public source accident reports retain their source attribution
and are not relicensed as software. Generated scenario parameters and recorded
single-run results describe the documented tests. Refer to the paper and evidence
manifest when interpreting them.
