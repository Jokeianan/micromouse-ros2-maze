"""Lightweight browser view of actual A* records and live ROS control state."""
import gzip
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import uuid
from urllib.parse import parse_qs,urlsplit


class StepVisualization:
    def __init__(self,grid,plan,asset,output,port=8765,host='0.0.0.0'):
        self.lock=threading.Lock(); self.running=False; self.state={}; self.trail=[]
        self.session=uuid.uuid4().hex
        self.version=0; self.closed=False
        payload=dict(session=self.session,width=grid.width,height=grid.height,resolution=grid.res,origin=grid.origin,
                     radius=grid.radius,margin=grid.margin,boxes=grid.boxes,
                     occupied=grid.occupied.astype('int8').ravel().tolist(),
                     blocked=grid.blocked.astype('int8').ravel().tolist(),
                     domain=grid.domain.astype('int8').ravel().tolist(),
                     distance=grid.distance.round(4).ravel().tolist(),
                     plan={k:v for k,v in plan.items() if k!='search_trace'},trace=plan.get('search_trace',[]))
        raw=json.dumps(payload,separators=(',',':')).encode()
        self.snapshot=gzip.compress(raw,compresslevel=5)
        self.page=Path(asset).read_bytes()
        Path(output,'visualization_snapshot.json.gz').write_bytes(self.snapshot)
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args): pass
            def reply(self,body,content_type='application/json; charset=utf-8',status=200,compressed=False):
                self.send_response(status); self.send_header('Content-Type',content_type)
                self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
                if compressed: self.send_header('Content-Encoding','gzip')
                self.end_headers(); self.wfile.write(body)
            def do_GET(self):
                request=urlsplit(self.path)
                if request.path in ['/','/index.html']:
                    self.reply(owner.page,'text/html; charset=utf-8')
                elif request.path=='/api/snapshot':
                    self.reply(owner.snapshot,compressed=True)
                elif request.path=='/api/live':
                    try: since=max(0,int(parse_qs(request.query).get('since',['0'])[0]))
                    except ValueError: since=0
                    with owner.lock:
                        result=dict(session=owner.session,state=dict(owner.state),running=owner.running,
                                    trail=owner.trail[since:since+2000],next=min(len(owner.trail),since+2000),
                                    total=len(owner.trail),version=owner.version)
                    self.reply(json.dumps(result,separators=(',',':')).encode())
                else: self.reply(b'{}',status=404)
            def do_POST(self):
                if urlsplit(self.path).path!='/api/control': self.reply(b'{}',status=404); return
                # Same-origin JSON requests only; this dashboard controls a local
                # simulation and does not expose arbitrary code or file access.
                origin=self.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host'):
                    self.reply(b'{"error":"origin"}',status=403); return
                if not self.headers.get('Content-Type','').startswith('application/json'):
                    self.reply(b'{"error":"json required"}',status=415); return
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<1024: raise ValueError('Invalid request size')
                    action=json.loads(self.rfile.read(size)).get('action')
                    if action not in ['start','pause','resume']: raise ValueError('Unknown action')
                except (ValueError,TypeError,json.JSONDecodeError):
                    self.reply(b'{"error":"invalid action"}',status=400); return
                with owner.lock:
                    done=owner.state.get('state') in ['completed','aborted']
                    if not done: owner.running=action!='pause'
                    owner.version+=1
                    result=dict(ok=not done,running=owner.running)
                self.reply(json.dumps(result).encode())
        self.server=ThreadingHTTPServer((host,int(port)),Handler)
        self.server.daemon_threads=True
        self.port=self.server.server_address[1]
        self.thread=threading.Thread(target=self.server.serve_forever,name='astar-visualization',daemon=True)
        self.thread.start()

    def is_running(self):
        with self.lock: return self.running

    def update(self,data):
        with self.lock:
            self.state=dict(data)
            if all(k in data for k in ['x','y']):
                point=[round(data['x'],4),round(data['y'],4)]
                if not self.trail or math.dist(self.trail[-1],point)>=.025: self.trail.append(point)
            if data.get('state') in ['completed','aborted']: self.running=False

    def close(self):
        if not self.closed:
            self.closed=True; self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=2)
