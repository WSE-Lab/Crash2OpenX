import threading
import time
from concurrent.futures import ThreadPoolExecutor

import pytest

from tools.run_fresh_framework_batch import SerializedCarlaClient


def test_independent_clients_do_not_overlap_map_load_and_run(tmp_path):
    state = {"active": 0, "maximum": 0}
    mutex = threading.Lock()

    class Client:
        def extract_roadgraph(self, *, force):
            assert force
            return self.run_scenario()

        def run_scenario(self):
            with mutex:
                state["active"] += 1
                state["maximum"] = max(state["maximum"], state["active"])
            time.sleep(0.04)
            with mutex:
                state["active"] -= 1

    clients = [SerializedCarlaClient(Client(), tmp_path / "rpc.lock") for _ in range(2)]
    with ThreadPoolExecutor(2) as pool:
        results = [pool.submit(clients[0].extract_roadgraph), pool.submit(clients[1].run_scenario)]
        for future in results:
            future.result()
    assert state["maximum"] == 1


def test_failed_rpc_releases_lock(tmp_path):
    class Client:
        def run_scenario(self):
            raise RuntimeError("simulator failed")

        def extract_roadgraph(self, *, force):
            return "next call succeeded"

    client = SerializedCarlaClient(Client(), tmp_path / "rpc.lock")
    with pytest.raises(RuntimeError, match="simulator failed"):
        client.run_scenario()
    assert client.extract_roadgraph() == "next call succeeded"
