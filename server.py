"""
server.py — Local dev server for Lucas Pin Portfolio
Serves the static HTML/CSS files at http://localhost:8000

Usage:
    python server.py
    python server.py --port 5500
"""

import os
import argparse
import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse

# ==============================
# CONFIG
# ==============================
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
PORTFOLIO_DIR = os.path.join(BASE_DIR, "portfolio")
DEFAULT_PORT  = 8000

# ==============================
# APP
# ==============================
app = FastAPI(title="Lucas Pin — Portfolio Dev Server")


# ==============================
# ROOT REDIRECT
# ==============================
@app.get("/")
def root():
    """Redirect root to home page."""
    return RedirectResponse(url="/home.html")


# ==============================
# STATIC FILES
# ==============================
if os.path.isdir(PORTFOLIO_DIR):
    app.mount("/", StaticFiles(directory=PORTFOLIO_DIR, html=True), name="static")
    print(f"[Server] Serving portfolio from: {PORTFOLIO_DIR}")
else:
    print(f"[Server] ERROR — portfolio/ directory not found at: {PORTFOLIO_DIR}")


# ==============================
# ENTRY POINT
# ==============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Portfolio local dev server")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Port to listen on")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to bind to")
    args = parser.parse_args()

    print(f"\n  Portfolio running at -> http://{args.host}:{args.port}/home.html\n")
    uvicorn.run("server:app", host=args.host, port=args.port, reload=True)
