"""Shared MJPEG preview server.

A browser-viewable stream at ``http://<robot>:<port>/`` needs no ROS on the
viewing device, which is what makes the preview usable from a phone or a laptop
in a teaching laboratory. Frames are encoded once by the caller and fanned out
to every connected client, so the cost does not grow with the number of viewers.
"""

# References (full entries: docs/references.md at the repository root):
#   [R30] OpenCV 4.x: JPEG frames for the MJPEG stream
# More details: README.md of this package (web preview); Python http.server documentation

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

INDEX_HTML = b"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport"
content="width=device-width,initial-scale=1"><title>Learn2Race preview</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center}
img{width:100%;max-width:640px;height:auto;image-rendering:pixelated}
#st{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;max-width:640px;margin:8px auto;padding:0 8px}
.c{background:#1d1d1d;border-radius:6px;padding:6px 10px;min-width:120px;text-align:left}
.k{color:#888;font-size:12px}.v{font-size:20px;font-weight:600}
#led{display:inline-block;width:22px;height:22px;border-radius:50%;border:1px solid #555;vertical-align:middle;margin-right:6px}
</style></head><body>
<p>Learn2Race line follower</p>
<div id="st">
<div class="c"><div class="k">Status LED</div><div class="v"><span id="led"></span><span id="ledname">-</span></div></div>
<div class="c"><div class="k">Race state</div><div class="v" id="state">-</div></div>
<div class="c"><div class="k" id="rtk">Race time</div><div class="v" id="rt">-</div></div>
<div class="c gp"><div class="k">Current lap</div><div class="v" id="lt">-</div></div>
<div class="c gp"><div class="k">Laps counted (referee)</div><div class="v" id="laps">-</div></div>
<div class="c gp"><div class="k">Lap times</div><div class="v" id="lts">-</div></div>
<div class="c"><div class="k">Speed</div><div class="v" id="sp">-</div></div>
<div class="c"><div class="k">Peak speed</div><div class="v" id="spp">-</div></div>
<div class="c"><div class="k">Average speed</div><div class="v" id="spa">-</div></div>
<div class="c"><div class="k">Distance</div><div class="v" id="dist">-</div></div>
<div class="c"><div class="k">Finish marker seen</div><div class="v" id="finish">-</div></div>
<div class="c dr"><div class="k">Dragster phase</div><div class="v" id="dph">-</div></div>
<div class="c dr"><div class="k">Finish line</div><div class="v" id="dfin">-</div></div>
<div class="c dr"><div class="k">Braking zone (stop line)</div><div class="v" id="dbz">-</div></div>
<div class="c dr"><div class="k">At rest</div><div class="v" id="drest">-</div></div>
<div class="c gp"><div class="k">Splits seen</div><div class="v" id="forks">-</div></div>
<div class="c"><div class="k">Last mark</div><div class="v" id="last">-</div></div>
</div>
<img src="/stream.mjpg">
<script>
function upd(){fetch('/status.json',{cache:'no-store'}).then(r=>r.json()).then(s=>{
const l=s.led||{};const c=l.rgb||[0,0,0];
document.getElementById('led').style.background='rgb('+c.map(v=>Math.round(v*255)).join(',')+')';
document.getElementById('ledname').textContent=l.name||'-';
document.getElementById('state').textContent=s.state||'-';
const E=id=>document.getElementById(id);const sec=t=>(t==null?'-':t.toFixed(2)+' s');
const drag=s.mode==='dragster';
document.querySelectorAll('.gp').forEach(e=>e.style.display=drag?'none':'');
document.querySelectorAll('.dr').forEach(e=>e.style.display=drag?'':'none');
const d=s.dragster||{};const m=v=>(v==null?'-':v.toFixed(2)+' m');
E('dph').textContent=d.phase||'-';
E('dfin').textContent=d.finish_m==null?'not yet':m(d.finish_m)+(d.finish_time!=null?', '+d.finish_time.toFixed(2)+' s':'');
E('dbz').textContent=d.finish_m==null?'-':(d.stop_line_m!=null?'stop line at '+m(d.stop_line_m):'in the zone');
E('drest').textContent=d.rest_m==null?'-':m(d.past_finish_m)+' past finish, braked '+m(d.braked_m);
E('rtk').textContent=drag?'Run time':'Race time';
E('rt').textContent=sec(s.race_time);E('lt').textContent=sec(s.lap_time);
E('lts').textContent=(s.lap_times&&s.lap_times.length)?s.lap_times.map(t=>t.toFixed(1)).join(' / '):'-';
E('sp').textContent=(s.speed!=null?s.speed.toFixed(2):'-')+' m/s';
E('spp').textContent=(s.speed_peak!=null?s.speed_peak.toFixed(2):'-')+' m/s';
E('spa').textContent=(s.speed_avg!=null?s.speed_avg.toFixed(2):'-')+' m/s';
E('dist').textContent=(s.distance!=null?s.distance.toFixed(2):'-')+' m';
document.getElementById('laps').textContent=s.laps;
document.getElementById('finish').textContent=s.finish_marks+(s.finish_checker?' ('+s.finish_checker+' checker)':'');
document.getElementById('forks').textContent=s.fork_marks;
document.getElementById('last').textContent=s.last_mark||'-';
}).catch(()=>{});}
setInterval(upd,250);upd();
</script></body></html>"""


class FrameBuffer:
    """The most recent encoded frame, shared with every HTTP client."""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.clients = 0
        # Small JSON-able status shown beside the stream (/status.json).
        self.status: dict = {}
        self.status_lock = threading.Lock()

    def status_json(self) -> bytes:
        with self.status_lock:
            return json.dumps(self.status).encode()

    def publish(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()


def _handler(buffer: FrameBuffer):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, *_args):
            pass

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(INDEX_HTML)
                return
            if self.path == '/status.json':
                body = buffer.status_json()
                self.send_response(200)
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != '/stream.mjpg':
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            with buffer.condition:
                buffer.clients += 1
            try:
                while True:
                    with buffer.condition:
                        buffer.condition.wait(timeout=2.0)
                        frame = buffer.jpeg
                    if frame is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with buffer.condition:
                    buffer.clients -= 1

    return Handler


def start_server(buffer: FrameBuffer, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(('0.0.0.0', port), _handler(buffer))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
