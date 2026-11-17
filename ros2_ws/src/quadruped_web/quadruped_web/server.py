# SPDX-License-Identifier: MIT
import argparse,copy,json,mimetypes,os,re,signal,socket,sys,threading,time,traceback
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import urlparse,unquote
import yaml,rclpy
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from .maps import MapCatalog
from .bridge import Bridge
from .runtime import Runtime

class WebServer(ThreadingHTTPServer):
 daemon_threads=True
 allow_reuse_address=True
 def __init__(self,address,bridge,catalog,runtime,webroot):
  self.bridge,self.catalog,self.runtime,self.webroot=bridge,catalog,runtime,Path(webroot).resolve();self.switch_lock=threading.Lock();super().__init__(address,Handler)

class Handler(BaseHTTPRequestHandler):
 protocol_version='HTTP/1.1'
 def log_message(self,*args):pass
 def send(self,status,body,kind='application/json; charset=utf-8',cache=False):
  if not isinstance(body,bytes):body=json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
  self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','public, max-age=3600' if cache else 'no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Referrer-Policy','same-origin');self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'");self.end_headers()
  try:self.wfile.write(body)
  except (BrokenPipeError,ConnectionResetError):pass
 def do_GET(self):
  try:
   path=urlparse(self.path).path;b=self.server.bridge
   if path=='/api/state':
    result=b.snapshot();result['processes']=self.server.runtime.status();return self.send(200,result)
   if path=='/api/map':return self.send(200,b.map_data)
   if path=='/api/maps':return self.send(200,dict(maps=self.server.catalog.listing(),active=b.map_data['id']))
   if path.startswith('/api/'):return self.send(404,dict(error='Unknown API'))
   f=(self.server.webroot/unquote(path).lstrip('/')).resolve() if path!='/' else self.server.webroot/'index.html'
   if not f.is_relative_to(self.server.webroot) or not f.is_file():return self.send(404,dict(error='Not found'))
   mime=mimetypes.guess_type(str(f))[0] or 'application/octet-stream';return self.send(200,f.read_bytes(),mime,cache='/vendor/' in path)
  except Exception as e:self.send(500,dict(error=str(e)))
 def do_POST(self):
  try:
   # Loopback binding plus same-origin JSON/custom-header requests; no remote ROS exposure.
   host=self.headers.get('Host','');origin=self.headers.get('Origin','')
   if origin!='http://'+host:raise PermissionError('Same-origin request required')
   if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('JSON request required')
   owner=self.headers.get('X-Client-ID','')
   if not re.fullmatch(r'[0-9a-f]{32}',owner):raise ValueError('Client ID required')
   size=int(self.headers.get('Content-Length','0'))
   if size<0 or size>65536:raise ValueError('Request body too large')
   body=json.loads(self.rfile.read(size));path=urlparse(self.path).path;b=self.server.bridge
   if path=='/api/stop':result=b.rpc(lambda:b.stop())
   elif path=='/api/control':result=b.rpc(lambda:b.acquire(body['mode'],owner))
   elif path=='/api/heartbeat':result=b.rpc(lambda:(b.router.ping(owner),dict(alive=True))[1])
   elif path=='/api/manual':result=b.rpc(lambda:(b.router.command(owner,body['velocity']),dict(accepted=True))[1])
   elif path=='/api/initialpose':result=b.rpc(lambda:b.initial(body['pose'],owner))
   elif path=='/api/navigation':
    if body.get('localization_confirmed') is not True:raise ValueError('Confirm observed localization before navigation')
    result=b.rpc(lambda:b.submit(body['poses'],body['kind'],owner))
   elif path=='/api/cancel':result=b.rpc(lambda:b.stop('cancel requested'))
   elif path=='/api/map/select':result=self.select_map(body['id'])
   else:return self.send(404,dict(error='Unknown API'))
   self.send(200,dict(ok=True,result=result))
  except PermissionError as e:self.send(403,dict(ok=False,error=str(e)))
  except (ValueError,KeyError,TypeError) as e:self.send(409,dict(ok=False,error=str(e)))
  except Exception as e:self.send(500,dict(ok=False,error=str(e)))
 def select_map(self,map_id):
  if not self.server.switch_lock.acquire(blocking=False):raise ValueError('A map switch is already running')
  b=self.server.bridge
  try:
   def begin():
    if b.active() or b.router.mode!='idle':raise ValueError('Stop the current task/control before switching maps')
    b.map_switching=True;b.stop('map switching')
   data=self.server.catalog.load(map_id);b.rpc(begin)
   self.server.runtime.stop()
   def replace():
    b.map_data=data;b.loc={};b.pose=None;b.paths={};b.cloud=[];b.seen={};b.states={};b.task={'phase':'idle'};b.event('map_selected',dict(id=map_id,revision=data['revision']))
   b.rpc(replace);self.server.runtime.start(self.server.catalog.data_root/'maps'/map_id);b.rpc(lambda:setattr(b,'map_switching',False));return dict(map_id=map_id,requires_initialpose=True)
  finally:
   b.rpc(lambda:setattr(b,'map_switching',False));self.server.switch_lock.release()

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--data-root',type=Path,default=Path('/data0/rl/quadruped_nav'));p.add_argument('--map-id',default='d1_full_room_web_20261117_01');p.add_argument('--mode',choices=['sim','real'],default='sim');p.add_argument('--with-lio',action='store_true');p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=8766);a=p.parse_args()
 if a.mode=='real':p.error('First release is simulation-tested only; enable real control after verified SDK limits and robot-side timeout integration')
 if not 1024<=a.port<=65535:p.error('Invalid port')
 try:
  with socket.socket() as probe:
   probe.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);probe.bind(('127.0.0.1',a.port))
 except OSError:p.error('Web port is already in use; reuse the running session or choose another --port')
 a.output.mkdir(parents=True,exist_ok=False);share=Path(get_package_share_directory('quadruped_web'));cfg=yaml.safe_load((share/'config/web.yaml').read_text());exporter=Path(get_package_prefix('quadruped_web'))/'lib/quadruped_web/octomap_web_export';catalog=MapCatalog(a.root,a.data_root,exporter);data=catalog.load(a.map_id);runtime=Runtime(a.root,a.output/'runtime',a.mode,a.with_lio);events=(a.output/'events.jsonl').open('w',buffering=1)
 rclpy.init(signal_handler_options=SignalHandlerOptions.NO);bridge=Bridge(cfg,data,a.mode,events);executor=SingleThreadedExecutor();executor.add_node(bridge);ros_thread=threading.Thread(target=executor.spin,daemon=True);ros_thread.start();webroot=a.root/'ros2_ws/src/quadruped_web/web';server=WebServer(('127.0.0.1',a.port),bridge,catalog,runtime,webroot if webroot.is_dir() else share/'web')
 closing=threading.Event()
 def shutdown(sig,frame):
  if closing.is_set():return
  closing.set();threading.Thread(target=server.shutdown,daemon=True).start()
 signal.signal(signal.SIGINT,shutdown);signal.signal(signal.SIGTERM,shutdown)
 try:
  runtime.start(a.data_root/'maps'/a.map_id)
  (a.output/'session.json').write_text(json.dumps(dict(map_id=a.map_id,environment=a.mode,port=a.port,pid=os.getpid(),scope='real map data + existing navigation/localization; camera and SDK actions not integrated'),indent=2)+'\n')
  print('QUADRUPED_WEB_READY http://127.0.0.1:'+str(a.port),flush=True);server.serve_forever(poll_interval=.2)
 finally:
  try:bridge.rpc(lambda:bridge.stop('server shutdown'))
  except Exception:traceback.print_exc()
  finally:runtime.stop()
  bridge.stopping=True;time.sleep(.15);executor.shutdown(timeout_sec=3);ros_thread.join(timeout=3);bridge.destroy_node();rclpy.shutdown();events.close();server.server_close()
