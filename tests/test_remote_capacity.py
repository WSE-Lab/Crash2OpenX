import json
import subprocess
from dataclasses import replace

import pytest

from tools.carla_remote import CarlaRemoteClient, CarlaRemoteError, RemoteCfg


def client_with_capacity(free_bytes, free_inodes=2000):
    client = CarlaRemoteClient(replace(RemoteCfg(), host="test", min_free_gib=10))
    commands = []

    def ssh(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess([], 0, json.dumps({
            "free_bytes": free_bytes, "free_inodes": free_inodes,
            "total_inodes": 10000,
        }), "")

    client._ssh = ssh
    return client, commands


@pytest.mark.parametrize("free_bytes,inodes", [(9 * 1024**3, 2000), (20 * 1024**3, 0)])
def test_extraction_stops_before_any_remote_write(tmp_path, free_bytes, inodes):
    client, commands = client_with_capacity(free_bytes, inodes)
    source = tmp_path / "map.xodr"
    source.write_text("<OpenDRIVE/>")
    client._push = lambda *args: pytest.fail("uploaded with insufficient space")
    with pytest.raises(CarlaRemoteError, match="No files were deleted"):
        client.extract_roadgraph(source, force=True, out_root=tmp_path / "cache")
    assert len(commands) == 1
    assert "statvfs" in commands[0]
    assert "mkdir" not in commands[0] and "rm " not in commands[0]


def test_reserve_includes_the_input_upload():
    client, _ = client_with_capacity(10 * 1024**3)
    with pytest.raises(CarlaRemoteError):
        client.check_capacity(upload_bytes=1)
    assert client.check_capacity()["free_bytes"] == 10 * 1024**3


def test_unreadable_probe_fails_closed():
    client, _ = client_with_capacity(20 * 1024**3)
    client._ssh = lambda *args, **kwargs: subprocess.CompletedProcess([], 0, "invalid", "")
    with pytest.raises(CarlaRemoteError, match="Cannot verify"):
        client.check_capacity()


def test_remote_evidence_is_preserved_by_default(monkeypatch):
    monkeypatch.delenv("CARLA_REMOTE_KEEP_RUNS", raising=False)
    assert RemoteCfg.from_env().keep_remote_runs
