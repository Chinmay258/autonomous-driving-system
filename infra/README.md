# Infrastructure

Hosting decision: ADR-0003 — Cloudflare Tunnel from the 24/7 dev machine first,
one 12-factor Docker image, Fly.io as the drop-in migration target.

## Run it

```bash
docker compose -f infra/docker-compose.yml up -d --build
docker logs av_tunnel | grep trycloudflare   # your public https URL
```

- `../Dockerfile` — python:3.11-slim + uv frozen sync, Barcelona map baked in,
  non-root user, healthcheck. Config via env (`AV_MAP_PATH`, `AV_SIM_REALTIME`,
  `AV_MAX_CONCURRENT_DRIVES`).
- `docker-compose.yml` — demo (384 MB / 0.5 CPU) + `cloudflared` (128 MB /
  0.25 CPU), both hard-capped so they can never contend with anything else on
  the host. Port 8018 bound to 127.0.0.1 for local smoke checks only.

## Quick tunnel vs. permanent URL
The default compose uses a **quick tunnel**: free, no account, but the
`*.trycloudflare.com` URL changes whenever the tunnel restarts. For a stable
URL on your own domain: Cloudflare Zero Trust → Tunnels → create tunnel → copy
the token into `infra/.env` as `TUNNEL_TOKEN=...`, then switch the two
commented lines in `docker-compose.yml`.

## Fly.io migration (when desired)
`fly launch --copy-config --no-deploy && fly deploy` with the committed
`../fly.toml`, then repoint DNS. Same image, same env vars — zero code changes.
