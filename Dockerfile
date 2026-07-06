# Single 12-factor image (ADR-0003): all config via env vars, map baked in.
FROM python:3.11-slim
COPY --from=ghcr.io/astral-sh/uv:0.11.26 /uv /uvx /usr/local/bin/

WORKDIR /app
COPY pyproject.toml uv.lock ./
COPY packages ./packages
COPY apps/webdemo/backend ./apps/webdemo/backend
RUN uv sync --frozen --no-dev --all-packages && rm -rf /root/.cache

COPY maps/barcelona_eixample.osm ./maps/barcelona_eixample.osm

ENV AV_MAP_PATH=/app/maps/barcelona_eixample.osm \
    AV_SIM_REALTIME=1 \
    AV_MAX_CONCURRENT_DRIVES=20 \
    PYTHONUNBUFFERED=1

RUN useradd -m demo && chown -R demo /app
USER demo
EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --retries=3 CMD ["python", "-c", \
    "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://127.0.0.1:8000/healthz', timeout=3).status==200 else 1)"]

CMD ["uv", "run", "--frozen", "--no-sync", "uvicorn", "webdemo_backend.app:create_app", \
     "--factory", "--host", "0.0.0.0", "--port", "8000"]
