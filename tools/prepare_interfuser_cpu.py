#!/usr/bin/env python3
"""Create a task-local, device-selectable InterFuser agent without editing PCLA."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    args = parser.parse_args()
    link = args.runtime / "PCLA"
    if not link.is_symlink():
        raise SystemExit("Expected the task runtime's PCLA symlink.")
    source = link.resolve()
    destination = args.runtime.parent / "PCLA_device_selectable"
    if destination.exists():
        raise SystemExit("Device-selectable copy already exists.")
    relative = Path("pcla_agents/interfuser/interfuser_agent.py")

    def mirror(src, dst, suffix):
        dst.mkdir()
        for child in src.iterdir():
            if child.name == "__pycache__":
                continue
            target = dst / child.name
            if child.name == suffix.parts[0]:
                if len(suffix.parts) > 1:
                    mirror(child, target, Path(*suffix.parts[1:]))
                else:
                    target.write_bytes(child.read_bytes())
            else:
                target.symlink_to(child)

    mirror(source, destination, relative)
    agent = destination / relative
    original = agent.read_text()
    assert original.count(".cuda()") == 9
    modified = original.replace(".cuda()", ".to(self.inference_device)")
    modified = modified.replace("torch.load(path_to_model_file)", "torch.load(path_to_model_file, map_location=self.inference_device)")
    modified = modified.replace("torch.load(path_to_model_file, weights_only=False)", "torch.load(path_to_model_file, map_location=self.inference_device, weights_only=False)")
    setup = "    def setup(self, path_to_conf_file):\n"
    modified = modified.replace(setup, setup + '''        self.inference_device = torch.device(os.environ.get("PCLA_INFERENCE_DEVICE", "cuda"))
        if self.inference_device.type == "cpu":
            torch.set_num_threads(int(os.environ.get("PCLA_CPU_THREADS", "8")))
        print("PCLA_INFERENCE_DEVICE=" + str(self.inference_device), flush=True)
''', 1)
    marker = "        self.softmax = torch.nn.Softmax(dim=1)"
    modified = modified.replace(marker, '''        loaded_net = self.nets[0] if self.ensemble else self.net
        print("PCLA_MODEL_PARAMETER_DEVICE=" + str(next(loaded_net.parameters()).device), flush=True)
''' + marker, 1)
    compile(modified, str(agent), "exec")
    agent.write_text(modified)
    manifest = {"original_pcla": str(source), "task_pcla": str(destination),
                "modified_file": str(relative),
                "original_sha256": hashlib.sha256(original.encode()).hexdigest(),
                "modified_sha256": hashlib.sha256(modified.encode()).hexdigest(),
                "scope": "Device selection, checkpoint map_location, CPU thread limit and explicit device logging only. Network, weights, preprocessing, sensors and controller remain unchanged."}
    (destination / "device_selection_provenance.json").write_text(json.dumps(manifest, indent=2))
    next_link = link.with_name("PCLA.next")
    next_link.symlink_to(destination)
    os.replace(next_link, link)
    print(json.dumps(manifest))


if __name__ == "__main__":
    main()
