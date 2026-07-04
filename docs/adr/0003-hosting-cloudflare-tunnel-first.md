# ADR-0003: Cloudflare Tunnel from the dev machine first; Fly.io as the migration target

**Status:** Accepted · 2026-07-05

## Context
The demo needs a public HTTPS/WSS endpoint at ~$0. The dev laptop already runs
24/7 (hosts a live trading stack), so "laptop sleeps → dead link" does not
apply. Recruiters must never hit a cold-start stall.

## Decision
Primary hosting: the demo container runs on the laptop's Docker alongside a
`cloudflared` sidecar (docker-compose in `infra/`), exposing it via Cloudflare
Tunnel — free HTTPS + custom domain, no port-forwarding, origin IP hidden.

Migration path is a hard requirement: the app is **one Docker image, fully
12-factor** (all config via env vars, no host assumptions, map artifact baked
in at build). Moving to Fly.io is `fly launch` with the committed `fly.toml`
stub + DNS repoint; nothing else changes.

## Consequences
+ $0, always-on, live today; laptop resources are ample (demo needs ~100 MB RAM).
+ Same image → hosting is swappable without code changes.
− Shares fate with the laptop (reboots/outages take the demo down) — acceptable
  for a portfolio; revisit via Fly.io if it becomes a problem.
− Keep the demo container resource-capped (compose `mem_limit`/`cpus`) so it can
  never starve the trading stack.
