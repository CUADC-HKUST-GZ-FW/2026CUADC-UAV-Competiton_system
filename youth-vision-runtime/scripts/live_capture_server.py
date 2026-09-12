#!/usr/bin/env python3
import argparse
import json
import os
import re
import shutil
import threading
import time
import urllib.parse
import zipfile
from datetime import datetime
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class LiveCaptureHandler(SimpleHTTPRequestHandler):
    collect_lock = threading.Lock()
    web_root: Path
    capture_root: Path
    recon_root: Path | None = None

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        if path != "/api/collect":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            with self.collect_lock:
                result = self._collect_current_crops()
            self._send_json(HTTPStatus.OK, result)
        except FileNotFoundError as exc:
            self._send_json(HTTPStatus.CONFLICT, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self.log_error("collection failed: %s", exc)
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/recon":
            self._send_json(HTTPStatus.OK, self._read_recon_snapshot())
            return
        if path.startswith("/recon/"):
            self._serve_recon_file(path)
            return
        if path.startswith("/downloads/") and path.endswith(".zip"):
            self._serve_download(path)
            return
        super().do_GET()

    def _read_recon_snapshot(self):
        if self.recon_root is None:
            return {"ok": True, "enabled": False, "status": None, "targets": []}
        root = self.recon_root
        status = None
        status_path = root / "status.json"
        if status_path.is_file():
            try:
                status = self._read_manifest(status_path)
            except (OSError, json.JSONDecodeError):
                status = {"status": "updating", "coordinate_valid": False}
        targets = []
        if root.is_dir():
            for result_path in sorted(root.glob("target_*/result.json")):
                try:
                    result = self._read_manifest(result_path)
                except (OSError, json.JSONDecodeError):
                    continue
                target_id = result_path.parent.name
                result["frame_url"] = f"/recon/{target_id}/frame.jpg"
                result["crop_url"] = f"/recon/{target_id}/crop_128.jpg"
                targets.append(result)
        return {"ok": True, "enabled": True, "status": status, "targets": targets}

    def _serve_recon_file(self, request_path):
        if self.recon_root is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        relative = urllib.parse.unquote(request_path.removeprefix("/recon/")).lstrip("/")
        root = self.recon_root.resolve()
        target = (root / relative).resolve()
        if not target.is_relative_to(root) or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = "image/jpeg" if target.suffix.lower() in {".jpg", ".jpeg"} else "application/octet-stream"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(target.stat().st_size))
        self.end_headers()
        with target.open("rb") as source:
            shutil.copyfileobj(source, self.wfile)

    def _collect_current_crops(self):
        latest_dir = self.web_root / "latest_crops"
        manifest_path = latest_dir / "manifest.json"
        if not manifest_path.exists():
            raise FileNotFoundError("尚未生成分类输入裁剪")

        self.capture_root.mkdir(parents=True, exist_ok=True)
        downloads = self.web_root / "downloads"
        downloads.mkdir(parents=True, exist_ok=True)

        session_id = datetime.now().strftime("capture_%Y%m%d_%H%M%S_%f")[:-3]
        final_dir = self.capture_root / session_id
        temp_dir = self.capture_root / f".{session_id}.tmp"

        snapshot = None
        for _ in range(5):
            if temp_dir.exists():
                shutil.rmtree(temp_dir)
            temp_dir.mkdir(parents=True)

            before = self._read_manifest(manifest_path)
            crops = before.get("crops", [])
            if not crops:
                shutil.rmtree(temp_dir)
                raise FileNotFoundError("当前画面没有可采集的标靶")

            saved = []
            for item in crops:
                source_name = Path(str(item.get("src", ""))).name
                source = latest_dir / source_name
                if not source.is_file():
                    raise FileNotFoundError(f"裁剪尚未写完: {source_name}")
                rank = int(item.get("rank", len(saved))) + 1
                label = re.sub(r"[^0-9A-Za-z_-]+", "_", str(item.get("class_label", "unknown")))
                probability = float(item.get("class_prob", 0.0))
                target_name = f"target_{rank:02d}_cls_{label}_p_{probability:.4f}.jpg"
                shutil.copy2(source, temp_dir / target_name)
                saved_item = dict(item)
                saved_item["saved_file"] = target_name
                saved.append(saved_item)

            after = self._read_manifest(manifest_path)
            if before.get("frame") == after.get("frame"):
                snapshot = dict(before)
                snapshot["crops"] = saved
                break
            time.sleep(0.02)

        if snapshot is None:
            shutil.rmtree(temp_dir, ignore_errors=True)
            raise RuntimeError("实时帧更新过快，请再次点击采集")

        snapshot["captured_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
        snapshot["session"] = session_id
        (temp_dir / "manifest.json").write_text(
            json.dumps(snapshot, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        overlay = self.web_root / "latest.jpg"
        if overlay.is_file():
            shutil.copy2(overlay, temp_dir / "context_overlay.jpg")

        temp_dir.rename(final_dir)
        archive = downloads / f"{session_id}.zip"
        with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as output:
            for file in sorted(final_dir.iterdir()):
                output.write(file, arcname=f"{session_id}/{file.name}")

        self._trim_downloads(downloads)
        return {
            "ok": True,
            "session": session_id,
            "count": len(snapshot["crops"]),
            "frame": snapshot.get("frame"),
            "download_url": f"/downloads/{archive.name}",
            "saved_on_jetson": str(final_dir),
        }

    @staticmethod
    def _read_manifest(path):
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def _trim_downloads(directory, keep=50):
        archives = sorted(directory.glob("capture_*.zip"), key=lambda item: item.stat().st_mtime, reverse=True)
        for old in archives[keep:]:
            old.unlink(missing_ok=True)

    def _serve_download(self, request_path):
        filename = Path(urllib.parse.unquote(request_path)).name
        target = (self.web_root / "downloads" / filename).resolve()
        downloads = (self.web_root / "downloads").resolve()
        if target.parent != downloads or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        size = target.stat().st_size
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/zip")
        self.send_header("Content-Disposition", f'attachment; filename="{target.name}"')
        self.send_header("Content-Length", str(size))
        self.end_headers()
        with target.open("rb") as source:
            shutil.copyfileobj(source, self.wfile)

    def _send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    parser = argparse.ArgumentParser(description="Youth Vision live page and crop collection server")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--web-root", type=Path, required=True)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--recon-root", type=Path)
    args = parser.parse_args()

    web_root = args.web_root.resolve()
    web_root.mkdir(parents=True, exist_ok=True)
    handler = lambda *handler_args, **handler_kwargs: LiveCaptureHandler(
        *handler_args, directory=str(web_root), **handler_kwargs
    )
    LiveCaptureHandler.web_root = web_root
    LiveCaptureHandler.capture_root = args.capture_root.resolve()
    LiveCaptureHandler.recon_root = args.recon_root.resolve() if args.recon_root else None

    server = ThreadingHTTPServer((args.bind, args.port), handler)
    print(f"serving {web_root} on {args.bind}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
