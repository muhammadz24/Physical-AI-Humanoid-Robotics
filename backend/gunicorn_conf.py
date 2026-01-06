# backend/gunicorn_conf.py
# Production configuration for Gunicorn server

import multiprocessing
import os

# Server socket
bind = "0.0.0.0:8000"
backlog = 2048

# Worker processes
workers = int(os.environ.get("WORKERS", 1))  # For Vercel serverless functions
worker_class = "uvicorn.workers.UvicornWorker"
worker_connections = 1000
timeout = 300  # 5 minutes for longer AI processing
keepalive = 60
max_requests = 1000
max_requests_jitter = 100

# Restart workers after this many requests, to help prevent memory leaks
preload_app = True

# Logging
accesslog = "-"
errorlog = "-"
loglevel = "info"

# Process naming
proc_name = "physical-ai-backend"

# Security
limit_request_line = 4094
limit_request_fields = 100
limit_request_field_size = 8190