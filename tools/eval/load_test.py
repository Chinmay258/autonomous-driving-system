"""Concurrency measurements: HTTP planning load, WebSocket drive load and the drive-cap race.

Starts the real service under uvicorn (twice: once with the drive cap set to 2 to probe the
check-then-act race, once uncapped for load), then measures:

- planning latency and throughput at 1, 4, 16 and 64 concurrent POST /plan requests;
- telemetry rate, frame-interval percentiles, server CPU and memory with 1, 50, 200 and 500
  simultaneous WebSocket drives.

Run from the repository root (Windows: process CPU and memory are read with the Win32 API):

    uv run python tools/eval/load_test.py

Results are printed and written to tools/eval/results/load_test.json.
"""

from __future__ import annotations

import asyncio
import ctypes
import itertools
import json
import math
import os
import random
import statistics as stats
import subprocess
import sys
import time
from ctypes import wintypes as wt
from pathlib import Path

import httpx
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed

REPO = Path(__file__).resolve().parents[2]
ART = str(REPO / "maps" / "barcelona_eixample.osm")
RESULTS = Path(__file__).resolve().parent / "results"
RES: dict[str, object] = {}
rng = random.Random(7)

if sys.platform != "win32":
    raise SystemExit("load_test.py reads process CPU and memory through the Win32 API")

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
psapi = ctypes.WinDLL("psapi", use_last_error=True)
k32.OpenProcess.restype = wt.HANDLE
k32.OpenProcess.argtypes = [wt.DWORD, wt.BOOL, wt.DWORD]


class FILETIME(ctypes.Structure):
    _fields_ = [("lo", wt.DWORD), ("hi", wt.DWORD)]


class PMC(ctypes.Structure):
    _fields_ = [
        ("cb", wt.DWORD),
        ("PageFaultCount", wt.DWORD),
        ("PeakWorkingSetSize", ctypes.c_size_t),
        ("WorkingSetSize", ctypes.c_size_t),
        ("a", ctypes.c_size_t),
        ("b", ctypes.c_size_t),
        ("c", ctypes.c_size_t),
        ("d", ctypes.c_size_t),
        ("PagefileUsage", ctypes.c_size_t),
        ("PeakPagefileUsage", ctypes.c_size_t),
    ]


def proc_stats(pid):
    """(working set in MB, total CPU seconds) of a process."""
    h = k32.OpenProcess(0x0400 | 0x0010, False, pid)
    c, e, kt, ut = FILETIME(), FILETIME(), FILETIME(), FILETIME()
    k32.GetProcessTimes(h, ctypes.byref(c), ctypes.byref(e), ctypes.byref(kt), ctypes.byref(ut))
    pmc = PMC()
    pmc.cb = ctypes.sizeof(PMC)
    psapi.GetProcessMemoryInfo(h, ctypes.byref(pmc), pmc.cb)
    k32.CloseHandle(h)
    cpu = ((kt.hi << 32 | kt.lo) + (ut.hi << 32 | ut.lo)) / 1e7
    return pmc.WorkingSetSize / 1e6, cpu


def real_pid(pid):
    """The uvicorn worker is a child of the launcher process on Windows."""
    query = (
        f"(Get-CimInstance Win32_Process -Filter 'ParentProcessId={pid}' "
        "| Select-Object -ExpandProperty ProcessId) -join ' '"
    )
    r = subprocess.run(
        ["powershell", "-NoProfile", "-Command", query], capture_output=True, text=True, check=False
    )
    kids = [int(x) for x in r.stdout.split() if x.isdigit()]
    return kids[0] if kids else pid


def pct(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, math.ceil(q / 100 * len(s)) - 1))] if s else None


def show(key, value):
    RES[key] = value
    print(key, "=", json.dumps(value), flush=True)


def start_server(port, **env_extra):
    env = dict(os.environ)
    env.update({k: str(v) for k, v in env_extra.items()})
    log_path = RESULTS / f"server_{port}.log"
    log = open(log_path, "wb")  # noqa: SIM115 - kept open for the server's lifetime
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "uvicorn",
            "webdemo_backend.app:create_app",
            "--factory",
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--log-level",
            "warning",
            "--no-access-log",
        ],
        cwd=str(REPO),
        env=env,
        stdout=log,
        stderr=log,
    )
    t0 = time.perf_counter()
    while True:
        if proc.poll() is not None:
            raise RuntimeError(log_path.read_text(errors="replace")[-1200:])
        try:
            if httpx.get(f"http://127.0.0.1:{port}/healthz", timeout=1.0).status_code == 200:
                return proc, time.perf_counter() - t0
        except httpx.HTTPError:
            pass
        time.sleep(0.05)


def sample_points(port, k=400):
    roads = httpx.get(f"http://127.0.0.1:{port}/map/roads", timeout=60).json()
    coords = [
        c
        for f in roads["features"]
        if f["properties"]["kind"] == "lane"
        for c in f["geometry"]["coordinates"]
    ]
    return rng.sample(coords, min(k, len(coords)))


def body(a, b):
    return {"start": {"lat": a[1], "lng": a[0]}, "goal": {"lat": b[1], "lng": b[0]}}


async def http_load(port, conc, total, pts):
    lat, codes = [], []
    sem = asyncio.Semaphore(conc)
    limits = httpx.Limits(max_connections=conc + 2, max_keepalive_connections=conc + 2)
    async with httpx.AsyncClient(timeout=180, limits=limits) as client:

        async def one():
            async with sem:
                t0 = time.perf_counter()
                r = await client.post(
                    f"http://127.0.0.1:{port}/plan", json=body(rng.choice(pts), rng.choice(pts))
                )
                lat.append((time.perf_counter() - t0) * 1e3)
                codes.append(r.status_code)

        t0 = time.perf_counter()
        await asyncio.gather(*(one() for _ in range(total)))
        wall = time.perf_counter() - t0
    return {
        "concurrency": conc,
        "requests": total,
        "p50_ms": round(pct(lat, 50), 1),
        "p95_ms": round(pct(lat, 95), 1),
        "throughput_rps": round(total / wall, 2),
        "errors": sum(1 for c in codes if c != 200),
    }


async def longest_route(port, pts):
    best = None
    async with httpx.AsyncClient(timeout=60) as client:
        for _ in range(30):
            r = await client.post(
                f"http://127.0.0.1:{port}/plan", json=body(rng.choice(pts), rng.choice(pts))
            )
            d = r.json()
            if best is None or d["distance_m"] > best["distance_m"]:
                best = d
    return best


async def ws_load(port, route_id, n, pid, window=6.0):
    url = f"ws://127.0.0.1:{port}/ws/drive?route_id={route_id}"
    socks = [await connect(url) for _ in range(n)]
    _, cpu0 = proc_stats(pid)
    t_start = time.perf_counter()
    await asyncio.gather(*(s.send('{"cmd":"start"}') for s in socks))
    stop = time.perf_counter() + window
    stamps = [[] for _ in range(n)]

    async def reader(i, s):
        try:
            while True:
                rem = stop - time.perf_counter()
                if rem <= 0:
                    return
                await asyncio.wait_for(s.recv(), rem)
                stamps[i].append(time.perf_counter())
        except (TimeoutError, ConnectionClosed):
            return

    await asyncio.gather(*(reader(i, s) for i, s in enumerate(socks)))
    rss1, cpu1 = proc_stats(pid)
    wall = time.perf_counter() - t_start
    gaps, rates = [], []
    for st in stamps:
        st = [t for t in st if t > t_start + 1.5]
        if len(st) > 2:
            rates.append((len(st) - 1) / (st[-1] - st[0]))
            gaps.extend(b - a for a, b in itertools.pairwise(st))
    await asyncio.gather(*(s.close() for s in socks), return_exceptions=True)
    return {
        "drives": n,
        "frame_rate_hz": round(stats.mean(rates), 2) if rates else None,
        "gap_p50_ms": round(pct(gaps, 50) * 1e3, 1),
        "gap_p95_ms": round(pct(gaps, 95) * 1e3, 1),
        "gap_p99_ms": round(pct(gaps, 99) * 1e3, 1),
        "server_cpu_cores": round((cpu1 - cpu0) / wall, 2),
        "server_rss_mb": round(rss1, 1),
    }


async def race_test(port, pts, n=10):
    """With the cap at 2, start n drives at once and count how many are admitted."""
    best = await longest_route(port, pts)
    url = f"ws://127.0.0.1:{port}/ws/drive?route_id={best['route_id']}"
    socks = [await connect(url) for _ in range(n)]
    await asyncio.sleep(0.3)
    await asyncio.gather(*(s.send('{"cmd":"start"}') for s in socks))
    driving = rejected = 0
    for s in socks:
        try:
            driving += json.loads(await asyncio.wait_for(s.recv(), 5)).get("state") == "driving"
        except ConnectionClosed:
            rejected += 1
    late = await connect(url)
    try:
        await asyncio.wait_for(late.recv(), 3)
        late_result = "accepted"
    except ConnectionClosed as exc:
        late_result = "closed " + str(exc.rcvd.code if exc.rcvd else None)
    except TimeoutError:
        late_result = "timeout"
    for s in [*socks, late]:
        await s.close()
    return {
        "cap": 2,
        "simultaneous_starts": n,
        "admitted": driving,
        "rejected": rejected,
        "later_connection": late_result,
    }


async def main() -> None:
    RESULTS.mkdir(exist_ok=True)
    proc, _ = start_server(8791, AV_MAP_PATH=ART, AV_SIM_REALTIME=1, AV_MAX_CONCURRENT_DRIVES=2)
    try:
        show("cap_race", await race_test(8791, sample_points(8791)))
    finally:
        proc.terminate()

    proc, boot = start_server(
        8792, AV_MAP_PATH=ART, AV_SIM_REALTIME=1, AV_MAX_CONCURRENT_DRIVES=1000000
    )
    pid = real_pid(proc.pid)
    show("startup_s", round(boot, 2))
    show("rss_after_start_mb", round(proc_stats(pid)[0], 1))
    try:
        pts = sample_points(8792)
        show("http", [await http_load(8792, c, 64, pts) for c in (1, 4, 16, 64)])
        show("rss_after_plans_mb", round(proc_stats(pid)[0], 1))
        route_id = (await longest_route(8792, pts))["route_id"]
        show("ws", [await ws_load(8792, route_id, n, pid) for n in (1, 50, 200, 500)])
    finally:
        proc.terminate()
    (RESULTS / "load_test.json").write_text(json.dumps(RES, indent=1))
    print("LOADTEST_OK")


if __name__ == "__main__":
    asyncio.run(main())
