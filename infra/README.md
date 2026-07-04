# Infrastructure

Hosting decision: ADR-0003 — Cloudflare Tunnel from the 24/7 dev machine first,
one 12-factor Docker image, Fly.io as the drop-in migration target.

## Layout (populated in P7)
- `docker-compose.yml` — demo container + `cloudflared` sidecar, resource-capped
  (`mem_limit: 256m`, `cpus: 0.5`) so it can never contend with the trading stack.
- `fly.toml` — committed stub; migration = `fly launch` + DNS repoint.
- `Dockerfile` — multi-stage: build frontend → bake map artifact → slim runtime,
  non-root user.

## Cloudflare Tunnel (one-time setup, P7)
1. Free Cloudflare account; add your domain (or use a free `*.trycloudflare.com` quick tunnel for dev shares).
2. `cloudflared tunnel create av-demo` → credentials JSON (kept out of git; mounted via compose secret).
3. Route DNS: `cloudflared tunnel route dns av-demo demo.<yourdomain>`.
4. `docker compose up -d` — tunnel connects outbound; no inbound ports opened.

## Fly.io migration (when desired)
`fly launch --copy-config` with the stub, `fly secrets set` for the tile key,
repoint DNS. Same image, same env vars — zero code changes (ADR-0003).
