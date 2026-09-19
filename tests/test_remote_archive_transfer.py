from dataclasses import replace

from tools.carla_remote import CarlaRemoteClient, RemoteCfg


def test_compact_archive_keeps_video_logs_and_timestamp_evidence(tmp_path):
    source = tmp_path / "remote with spaces" / "outputs"
    (source / "rgb_frames").mkdir(parents=True)
    for name in ["carla_rgb.mp4", "events.jsonl", "rgb_frames/frame_000000.jpg", "rgb_frames/timestamps.jsonl", "rgb_frames/camera.json"]:
        (source / name).write_bytes(b"evidence")
    target = tmp_path / "local"
    client = CarlaRemoteClient(replace(RemoteCfg(), host="test", compact_artifacts=True, keep_remote_runs=True))
    client._ssh_base = lambda: ["sh", "-c"]
    client._pull_tree(str(source), target)
    assert (target / "outputs/carla_rgb.mp4").read_bytes() == b"evidence"
    assert (target / "outputs/events.jsonl").exists()
    assert (target / "outputs/rgb_frames/timestamps.jsonl").exists()
    assert (target / "outputs/rgb_frames/camera.json").exists()
    assert not (target / "outputs/rgb_frames/frame_000000.jpg").exists()
    assert (source / "rgb_frames/frame_000000.jpg").exists()
