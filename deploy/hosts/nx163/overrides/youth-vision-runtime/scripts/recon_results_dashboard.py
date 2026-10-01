#!/usr/bin/env python3
"""Serve a fast, read-only dashboard for de-duplicated recon results."""

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
import urllib.parse

from show_latest_recon_results import load_summary


HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>侦察识别结果</title>
  <style>
    :root { font-family: "Segoe UI", "Microsoft YaHei", sans-serif; color: #172026; background: #eef1f3; }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #eef1f3; }
    header { position: sticky; top: 0; z-index: 2; display: grid; gap: 8px; padding: 14px 20px; color: #f7f9fa; background: #20272c; border-bottom: 3px solid #d8aa31; }
    .topline { display: flex; align-items: baseline; justify-content: space-between; gap: 18px; }
    h1 { margin: 0; font-size: 19px; letter-spacing: 0; }
    #clock { color: #b7c0c6; font: 12px Consolas, monospace; }
    .summary { display: flex; flex-wrap: wrap; gap: 12px 24px; color: #c9d0d5; font-size: 12px; }
    .summary strong { margin-left: 6px; color: #fff; }
    #health[data-level="ok"] { color: #65d6a6; }
    #health[data-level="warn"] { color: #ffd067; }
    #health[data-level="error"] { color: #ff8585; }
    main { width: min(1500px, 100%); margin: 0 auto; padding: 16px; }
    #notice { display: none; margin-bottom: 12px; padding: 10px 12px; border-left: 4px solid #d8aa31; background: #fff8df; color: #604b16; font-size: 13px; }
    #targets { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
    .target { min-width: 0; display: grid; grid-template-columns: 160px minmax(0, 1fr); gap: 14px; padding: 12px; border: 1px solid #ccd2d6; border-radius: 6px; background: #fff; box-shadow: 0 1px 2px rgba(0,0,0,.06); }
    .target.valid { border-left: 5px solid #238260; }
    .target.pending { border-left: 5px solid #d09b20; }
    .crop { display: block; width: 160px; height: 160px; object-fit: contain; border: 1px solid #aeb7bd; background: #101315; }
    .details { min-width: 0; display: grid; align-content: start; gap: 8px; }
    .result-line { display: flex; justify-content: space-between; align-items: baseline; gap: 12px; padding-bottom: 7px; border-bottom: 1px solid #e3e6e8; }
    .label { min-width: 0; overflow-wrap: anywhere; font-size: 25px; font-weight: 750; color: #11181c; }
    .confidence { white-space: nowrap; color: #117557; font: 700 17px Consolas, monospace; }
    .coordinate { display: grid; grid-template-columns: 42px minmax(0, 1fr); gap: 4px 8px; font: 14px/1.4 Consolas, monospace; }
    .coordinate b { color: #707b82; font-weight: 500; }
    .coordinate span { overflow-wrap: anywhere; color: #172026; }
    .quality { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 5px 14px; color: #59646b; font-size: 12px; }
    .quality strong { color: #263138; }
    .duplicate { padding-top: 7px; border-top: 1px solid #e3e6e8; color: #765717; font-size: 12px; overflow-wrap: anywhere; }
    .empty { grid-column: 1 / -1; padding: 70px 20px; text-align: center; border: 1px solid #ccd2d6; background: #fff; color: #68737a; }
    @media (max-width: 1040px) { #targets { grid-template-columns: 1fr; } }
    @media (max-width: 600px) {
      header { padding: 12px; }
      main { padding: 10px; }
      .target { grid-template-columns: 112px minmax(0, 1fr); gap: 10px; padding: 9px; }
      .crop { width: 112px; height: 112px; }
      .label { font-size: 20px; }
      .confidence { font-size: 14px; }
      .coordinate { font-size: 12px; }
      .quality { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <div class="topline"><h1>侦察识别结果</h1><span id="clock">--</span></div>
    <div class="summary">
      <span>任务<strong id="session">--</strong></span>
      <span>状态<strong id="health" data-level="warn">等待</strong></span>
      <span>原始结果<strong id="raw-count">0</strong></span>
      <span>去重标靶<strong id="target-count">0</strong></span>
      <span>去重半径<strong id="radius">--</strong></span>
    </div>
  </header>
  <main>
    <div id="notice"></div>
    <section id="targets"><div class="empty">正在读取识别结果</div></section>
  </main>
  <script>
    const session = document.getElementById('session');
    const health = document.getElementById('health');
    const rawCount = document.getElementById('raw-count');
    const targetCount = document.getElementById('target-count');
    const radius = document.getElementById('radius');
    const notice = document.getElementById('notice');
    const targets = document.getElementById('targets');
    const clock = document.getElementById('clock');

    const esc = value => String(value ?? '').replace(/[&<>'"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;',"'":'&#39;','"':'&quot;'}[c]));
    const number = (value, digits) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : '--';

    function render(data) {
      session.textContent = data.session_name || '--';
      rawCount.textContent = data.raw_result_count ?? 0;
      targetCount.textContent = data.deduplicated_count ?? 0;
      radius.textContent = `${number(data.dedup_radius_m, 1)} m`;
      const state = data.pipeline_status || {};
      health.textContent = state.status || 'unknown';
      health.dataset.level = data.targets?.some(t => t.valid) ? 'ok' : (state.status?.includes('waiting') ? 'warn' : 'error');
      clock.textContent = new Date().toLocaleString('zh-CN', { hour12: false });
      if (data.fallback_to_previous_session) {
        notice.style.display = 'block';
        notice.textContent = `当前任务尚无坐标结果，正在显示最近有结果的任务：${data.session_name}`;
      } else if (state.detail) {
        notice.style.display = 'block';
        notice.textContent = state.detail;
      } else {
        notice.style.display = 'none';
      }
      targets.replaceChildren();
      if (!data.targets?.length) {
        const empty = document.createElement('div');
        empty.className = 'empty';
        empty.textContent = '尚未产生可显示的标靶坐标结果';
        targets.append(empty);
        return;
      }
      data.targets.forEach((item, index) => targets.append(buildTarget(item, index)));
    }

    function buildTarget(item, index) {
      const article = document.createElement('article');
      article.className = `target ${item.valid ? 'valid' : 'pending'}`;
      const image = document.createElement('img');
      image.className = 'crop';
      image.alt = `标靶 ${index + 1} - ${item.label || 'unknown'}`;
      image.src = item.crop_url ? `${item.crop_url}&t=${Date.now()}` : '';
      const duplicate = [];
      if (item.merged_count > 1) duplicate.push(`疑似重复已合并 ${item.merged_count} 条：${item.merged_target_ids.join(', ')}`);
      if (item.alternative_labels?.length) duplicate.push(`标签冲突：曾识别为 ${item.alternative_labels.join(', ')}`);
      const details = document.createElement('div');
      details.className = 'details';
      details.innerHTML = `
        <div class="result-line"><span class="label">${esc(item.label || '--')}</span><span class="confidence">${number(Number(item.confidence) * 100, 2)}%</span></div>
        <div class="coordinate">
          <b>LAT</b><span>${number(item.latitude, 8)}</span>
          <b>LON</b><span>${number(item.longitude, 8)}</span>
          <b>MSL</b><span>${number(item.altitude_msl_m, 2)} m</span>
        </div>
        <div class="quality">
          <span>状态 <strong>${esc(item.status)} / ${item.valid ? '有效' : '待确认'}</strong></span>
          <span>观测 <strong>${item.observation_count} 帧</strong></span>
          <span>定位 <strong>${item.rtk_fixed ? 'RTK Fixed' : `GPS fix ${item.gps_fix_type}`}</strong></span>
          <span>R95 <strong>${number(item.horizontal_radius_95_m, 2)} m</strong></span>
          <span>组内离散 <strong>${number(item.cluster_spread_m, 2)} m</strong></span>
          <span>姿态 <strong>${esc(item.attitude_source)}</strong></span>
        </div>
        ${duplicate.length ? `<div class="duplicate">${duplicate.map(esc).join('<br>')}</div>` : ''}`;
      article.append(image, details);
      return article;
    }

    async function refresh() {
      try {
        const response = await fetch(`/api/results?t=${Date.now()}`, { cache: 'no-store' });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        render(await response.json());
      } catch (error) {
        health.textContent = '读取失败';
        health.dataset.level = 'error';
        notice.style.display = 'block';
        notice.textContent = error.message;
      }
    }
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>'''


class DashboardHandler(BaseHTTPRequestHandler):
    root = Path('.')
    dedup_radius_m = 8.0

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        super().end_headers()

    def do_GET(self):
        request = urllib.parse.urlparse(self.path)
        if request.path in {'/', '/index.html'}:
            self._send_bytes(HTTPStatus.OK, 'text/html; charset=utf-8', HTML.encode('utf-8'))
            return
        if request.path == '/api/results':
            self._send_results()
            return
        if request.path == '/asset':
            self._send_asset(urllib.parse.parse_qs(request.query).get('path', [''])[0])
            return
        if request.path == '/health':
            self._send_json({'ok': True})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def _send_results(self):
        summary = load_summary(self.root, None, False, self.dedup_radius_m)
        for target in summary['targets']:
            crop_path = target.pop('representative_crop_path', '')
            target.pop('representative_frame_path', None)
            target['crop_url'] = '/asset?path=' + urllib.parse.quote(crop_path, safe='') if crop_path else ''
        self._send_json(summary)

    def _send_asset(self, requested_path):
        try:
            root = self.root.resolve()
            target = Path(urllib.parse.unquote(requested_path)).resolve()
            if not target.is_relative_to(root) or not target.is_file():
                raise FileNotFoundError
            if target.suffix.lower() not in {'.jpg', '.jpeg', '.png'}:
                raise FileNotFoundError
        except (OSError, FileNotFoundError):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = 'image/png' if target.suffix.lower() == '.png' else 'image/jpeg'
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(target.stat().st_size))
        self.end_headers()
        with target.open('rb') as source:
            shutil.copyfileobj(source, self.wfile)

    def _send_json(self, value):
        payload = json.dumps(value, ensure_ascii=False).encode('utf-8')
        self._send_bytes(HTTPStatus.OK, 'application/json; charset=utf-8', payload)

    def _send_bytes(self, status, content_type, payload):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format_text, *args):
        return


def main():
    parser = argparse.ArgumentParser(description='Serve the de-duplicated recon results dashboard.')
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8092)
    parser.add_argument('--root', type=Path, default=Path('/home/nx163/youth-vision-runtime/recon_results'))
    parser.add_argument('--dedup-radius-m', type=float, default=8.0)
    args = parser.parse_args()
    DashboardHandler.root = args.root
    DashboardHandler.dedup_radius_m = args.dedup_radius_m
    server = ThreadingHTTPServer((args.bind, args.port), DashboardHandler)
    print(f'recon results dashboard listening on http://{args.bind}:{args.port}', flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
