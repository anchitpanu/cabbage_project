"use client";

import React, { useEffect, useRef, useState, useCallback } from "react";
import * as ROSLIB from "roslib";
import {
  Camera, Gauge, PlugZap, Satellite, Signal,
  MapPin, Leaf, Ruler, Activity, Navigation
} from "lucide-react";

type RosLike = ROSLIB.Ros | null;

const TOPICS = {
  CAMERA1 : "/camera1/image_detected",
  CAMERA2 : "/camera2/image_detected",
  DIST_AB : "/quin/debug/encoder",
  DIST_C  : "/quin/debug/encoder_c",
  DIST_DE : "/quin/debug/encoder_de",
  ODOM    : "/odom",
  YOLO    : "/yolo/detections",
};

interface Cabbage { id: string; size: number; keep: boolean; score: number; }
interface GraphPoint { t: number; x: number; y: number; v: number; }

export default function Page() {
  const [wsUrl, setWsUrl]       = useState("ws://10.129.196.237:9090");
  const [mjpeg1, setMjpeg1]     = useState(`http://10.129.196.237:8081/stream?topic=${TOPICS.CAMERA1}`);
  const [mjpeg2, setMjpeg2]     = useState(`http://10.129.196.237:8081/stream?topic=${TOPICS.CAMERA2}`);
  const rosRef                  = useRef<RosLike>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [autoReconnect, setAutoReconnect] = useState(true);
  const [distAB, setDistAB]     = useState<number | null>(null);
  const [distC,  setDistC]      = useState<number | null>(null);
  const [distDE, setDistDE]     = useState<number | null>(null);
  const [distABAt, setDistABAt] = useState<number | null>(null);
  const [distCAt,  setDistCAt]  = useState<number | null>(null);
  const [distDEAt, setDistDEAt] = useState<number | null>(null);
  const [posX,   setPosX]       = useState<number | null>(null);
  const [posY,   setPosY]       = useState<number | null>(null);
  const [vel,    setVel]        = useState<number | null>(null);
  const [heading, setHeading]   = useState<number | null>(null);
  const [odomAt, setOdomAt]     = useState<number | null>(null);
  const [cabbages, setCabbages] = useState<Cabbage[]>([]);
  const cabCounter              = useRef(0);
  const [graph, setGraph]       = useState<GraphPoint[]>([]);
  const t0Ref                   = useRef<number | null>(null);
  const canvasRef               = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    return () => { try { rosRef.current?.close(); } catch {} };
  }, []);

  const connect = useCallback(() => {
    try { rosRef.current?.close(); } catch {}
    const ros = new ROSLIB.Ros({ url: wsUrl });
    rosRef.current = ros;
    ros.on("connection", () => { setIsConnected(true); subscribeAll(ros); });
    ros.on("close", () => { setIsConnected(false); if (autoReconnect) setTimeout(() => connect(), 1500); });
    ros.on("error", () => setIsConnected(false));
  }, [wsUrl, autoReconnect]);

  const disconnect = () => { setAutoReconnect(false); try { rosRef.current?.close(); } catch {} };

  const subsRef = useRef<ROSLIB.Topic[]>([]);
  const subscribeAll = (ros: ROSLIB.Ros) => {
    subsRef.current.forEach(s => s.unsubscribe());
    subsRef.current = [];
    const sub = (name: string, type: string, cb: (m: any) => void, throttle = 100) => {
      const t = new ROSLIB.Topic({ ros, name, messageType: type, throttle_rate: throttle, queue_size: 1 });
      t.subscribe(cb); subsRef.current.push(t); return t;
    };
    sub(TOPICS.DIST_AB, "std_msgs/msg/Float32", (m) => { const cm = Number(m?.data); if (Number.isFinite(cm)) { setDistAB(cm); setDistABAt(Date.now()); } });
    sub(TOPICS.DIST_C,  "std_msgs/msg/Float32", (m) => { const cm = Number(m?.data); if (Number.isFinite(cm)) { setDistC(cm);  setDistCAt(Date.now());  } });
    sub(TOPICS.DIST_DE, "std_msgs/msg/Float32", (m) => { const cm = Number(m?.data); if (Number.isFinite(cm)) { setDistDE(cm); setDistDEAt(Date.now()); } });
    sub(TOPICS.ODOM, "nav_msgs/msg/Odometry", (m) => {
      const px = m.pose.pose.position.x, py = m.pose.pose.position.y;
      const q  = m.pose.pose.orientation;
      const yaw = Math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
      const vx = m.twist.twist.linear.x, vy = m.twist.twist.linear.y;
      const spd = Math.sqrt(vx*vx+vy*vy);
      setPosX(px); setPosY(py); setVel(spd);
      setHeading(Math.round(yaw*180/Math.PI)); setOdomAt(Date.now());
      const now = Date.now()/1000;
      if (!t0Ref.current) t0Ref.current = now;
      const t = parseFloat((now-t0Ref.current).toFixed(2));
      setGraph(prev => { const next = [...prev, {t,x:px,y:py,v:spd}]; return next.length>300?next.slice(-300):next; });
    }, 50);
    sub(TOPICS.YOLO, "vision_msgs/msg/Detection2DArray", (m) => {
      const newCabs: Cabbage[] = [];
      for (const det of (m.detections||[])) {
        for (const res of (det.results||[])) {
          if ((res.hypothesis?.score??0)<0.5) continue;
          if (!res.hypothesis?.class_id?.toLowerCase().includes("cabbage")) continue;
          cabCounter.current+=1;
          const diam = ((det.bbox?.size_x??0)+(det.bbox?.size_y??0))/2*0.05;
          newCabs.push({ id:`C-${String(cabCounter.current).padStart(3,"0")}`, size:parseFloat(diam.toFixed(1)), keep:diam>=20.0, score:parseFloat((res.hypothesis.score).toFixed(2)) });
        }
      }
      if (newCabs.length>0) setCabbages(prev=>[...prev,...newCabs].slice(-20));
    });
  };

  useEffect(() => {
    const canvas = canvasRef.current; if (!canvas) return;
    const DPR = window.devicePixelRatio||1;
    const W = canvas.offsetWidth, H = canvas.offsetHeight;
    canvas.width=W*DPR; canvas.height=H*DPR;
    const ctx = canvas.getContext("2d")!; ctx.scale(DPR,DPR);
    const M={top:24,right:16,bottom:44,left:52};
    const CW=W-M.left-M.right, CH=H-M.top-M.bottom;
    ctx.clearRect(0,0,W,H); ctx.fillStyle="#f8fafc"; ctx.fillRect(0,0,W,H);
    ctx.fillStyle="#f1f5f9"; ctx.fillRect(M.left,M.top,CW,CH);
    const pts=graph;
    const yMax=pts.length>0?Math.max(...pts.flatMap(p=>[p.x,p.y,p.v]),3.5):3.5;
    const tMax=pts.length>0?Math.max(pts[pts.length-1].t,10):60;
    const toX=(t:number)=>M.left+(t/tMax)*CW;
    const toY=(v:number)=>M.top+CH-(v/yMax)*CH;
    ctx.strokeStyle="#e2e8f0"; ctx.lineWidth=1;
    const yStep=yMax<=4?0.5:1;
    for(let v=0;v<=yMax;v+=yStep){ctx.beginPath();ctx.moveTo(M.left,toY(v));ctx.lineTo(M.left+CW,toY(v));ctx.stroke();}
    const tStep=tMax<=30?5:10;
    for(let t=0;t<=tMax;t+=tStep){ctx.beginPath();ctx.moveTo(toX(t),M.top);ctx.lineTo(toX(t),M.top+CH);ctx.stroke();}
    ctx.strokeStyle="#ef4444"; ctx.setLineDash([6,4]); ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(M.left,toY(3)); ctx.lineTo(M.left+CW,toY(3)); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle="#ef4444"; ctx.font="10px monospace"; ctx.textAlign="left";
    ctx.fillText("REF = 3.0 m",M.left+4,toY(3)-5);
    ctx.strokeStyle="#94a3b8"; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(M.left,M.top); ctx.lineTo(M.left,M.top+CH); ctx.lineTo(M.left+CW,M.top+CH); ctx.stroke();
    ctx.fillStyle="#64748b"; ctx.font="10px monospace"; ctx.textAlign="right";
    for(let v=0;v<=yMax;v+=yStep) ctx.fillText(v.toFixed(1),M.left-5,toY(v)+4);
    ctx.textAlign="center";
    for(let t=0;t<=tMax;t+=tStep) ctx.fillText(t+"s",toX(t),M.top+CH+16);
    ctx.save(); ctx.translate(13,M.top+CH/2); ctx.rotate(-Math.PI/2);
    ctx.fillStyle="#475569"; ctx.font="11px sans-serif"; ctx.textAlign="center";
    ctx.fillText("Position / Velocity",0,0); ctx.restore();
    ctx.fillStyle="#475569"; ctx.font="11px sans-serif"; ctx.textAlign="center";
    ctx.fillText("Time (seconds)",M.left+CW/2,H-6);
    if(pts.length<2) return;
    const drawArea=(buf:number[],color:string)=>{
      ctx.beginPath(); ctx.moveTo(toX(pts[0].t),toY(0));
      buf.forEach((v,i)=>ctx.lineTo(toX(pts[i].t),toY(v)));
      ctx.lineTo(toX(pts[pts.length-1].t),toY(0)); ctx.closePath();
      ctx.fillStyle=color; ctx.fill();
    };
    drawArea(pts.map(p=>p.v),"rgba(251,191,36,0.15)");
    drawArea(pts.map(p=>p.y),"rgba(34,197,94,0.12)");
    drawArea(pts.map(p=>p.x),"rgba(59,130,246,0.10)");
    const drawLine=(buf:number[],color:string,lw:number)=>{
      if(buf.length<2) return;
      ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=lw;
      ctx.lineJoin="round"; ctx.lineCap="round";
      buf.forEach((v,i)=>i===0?ctx.moveTo(toX(pts[i].t),toY(v)):ctx.lineTo(toX(pts[i].t),toY(v)));
      ctx.stroke();
    };
    drawLine(pts.map(p=>p.v),"#f59e0b",1.5);
    drawLine(pts.map(p=>p.y),"#22c55e",2);
    drawLine(pts.map(p=>p.x),"#3b82f6",2.5);
    const last=pts[pts.length-1];
    const lx=toX(last.t),ly=toY(last.x);
    ctx.beginPath(); ctx.arc(lx,ly,5,0,Math.PI*2); ctx.fillStyle="#3b82f6"; ctx.fill();
    ctx.strokeStyle="rgba(59,130,246,0.3)"; ctx.lineWidth=3;
    ctx.beginPath(); ctx.arc(lx,ly,9,0,Math.PI*2); ctx.stroke();
    ctx.strokeStyle="rgba(59,130,246,0.15)"; ctx.lineWidth=1; ctx.setLineDash([4,3]);
    ctx.beginPath(); ctx.moveTo(lx,M.top); ctx.lineTo(lx,M.top+CH); ctx.stroke();
    ctx.setLineDash([]);
  }, [graph]);

  const fmt=(n:number|null,d=3)=>n==null?"-":n.toFixed(d);
  const keepCount=cabbages.filter(c=>c.keep).length;
  const rejectCount=cabbages.filter(c=>!c.keep).length;
  const keepRate=cabbages.length>0?Math.round(keepCount/cabbages.length*100):0;
  const FIELD=5;
  const mapX=posX!=null?Math.min(93,Math.max(7,((posX+FIELD)/(FIELD*2))*100)):50;
  const mapY=posY!=null?Math.min(93,Math.max(7,(1-(posY+FIELD)/(FIELD*2))*100)):50;

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100 p-4 sm:p-6 text-slate-900">
      <div className="mx-auto max-w-7xl space-y-4">

        <header className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <PlugZap className={`h-6 w-6 ${isConnected?"text-green-600":"text-slate-400"}`}/>
            <div>
              <h1 className="text-2xl font-semibold">AgriBot Monitor</h1>
              <p className="text-xs text-slate-500">Cabbage Field Monitoring — Raspberry Pi 5</p>
            </div>
          </div>
          <div className="flex items-center gap-3">
            <Signal className={`h-5 w-5 ${isConnected?"text-green-600":"text-red-500"}`}/>
            <span className={`text-sm font-medium ${isConnected?"text-green-600":"text-red-500"}`}>
              {isConnected?"Connected":"Disconnected"}
            </span>
          </div>
        </header>

        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Satellite className="h-4 w-4"/><span>Connection</span>
          </div>
          <div className="grid gap-3 md:grid-cols-4">
            <LabeledInput label="Rosbridge WebSocket URL" value={wsUrl} onChange={setWsUrl} placeholder="ws://192.168.x.x:9090"/>
            <LabeledInput label="Camera A MJPEG URL" value={mjpeg1} onChange={setMjpeg1} placeholder="http://pi:8081/stream?topic=..."/>
            <LabeledInput label="Camera B MJPEG URL" value={mjpeg2} onChange={setMjpeg2} placeholder="http://pi:8081/stream?topic=..."/>
            <div className="flex items-end gap-2">
              <button onClick={connect} disabled={isConnected} className="flex-1 rounded-xl bg-slate-900 px-4 py-2 text-sm text-white hover:bg-slate-800 disabled:cursor-not-allowed disabled:opacity-50">Connect</button>
              <button onClick={disconnect} className="flex-1 rounded-xl border px-4 py-2 text-sm hover:bg-slate-50">Disconnect</button>
            </div>
          </div>
          <div className="mt-3 flex items-center justify-between">
            <label className="flex items-center gap-2 text-sm">
              <input type="checkbox" checked={autoReconnect} onChange={e=>setAutoReconnect(e.target.checked)}/>
              Auto reconnect
            </label>
            <p className="text-xs text-slate-400">Topics: <code>{TOPICS.DIST_AB}</code> · <code>{TOPICS.ODOM}</code> · <code>{TOPICS.YOLO}</code></p>
          </div>
        </section>

        <div className="grid gap-4 md:grid-cols-2">
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Camera className="h-4 w-4"/><span>Camera A — Front View</span>
              <span className={`ml-auto text-xs font-normal px-2 py-0.5 rounded-full ${isConnected?"bg-green-100 text-green-700":"bg-slate-100 text-slate-500"}`}>
                {isConnected?"● LIVE":"● WAITING"}
              </span>
            </div>
            <div className="aspect-video w-full overflow-hidden rounded-xl border bg-black">
              {isConnected
                ? <img src={mjpeg1} alt="Camera A" className="h-full w-full object-contain"/>
                : <div className="flex h-full items-center justify-center text-slate-600 text-sm">No signal — connect first</div>}
            </div>
            <p className="mt-2 text-xs text-slate-400">MJPEG · <code>{TOPICS.CAMERA1}</code></p>
          </section>
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-2 flex items-center gap-2 text-base font-semibold">
              <Camera className="h-4 w-4"/><span>Camera B — Top-Down</span>
              <span className={`ml-auto text-xs font-normal px-2 py-0.5 rounded-full ${isConnected?"bg-green-100 text-green-700":"bg-slate-100 text-slate-500"}`}>
                {isConnected?"● LIVE":"● WAITING"}
              </span>
            </div>
            <div className="aspect-video w-full overflow-hidden rounded-xl border bg-black">
              {isConnected
                ? <img src={mjpeg2} alt="Camera B" className="h-full w-full object-contain"/>
                : <div className="flex h-full items-center justify-center text-slate-600 text-sm">No signal — connect first</div>}
            </div>
            <p className="mt-2 text-xs text-slate-400">MJPEG · <code>{TOPICS.CAMERA2}</code></p>
          </section>
        </div>

        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Ruler className="h-4 w-4"/><span>AprilTag Distances</span>
          </div>
          <div className="grid gap-4 md:grid-cols-3">
            <KPI icon={<Ruler className="h-4 w-4"/>} title="Planting Distance [AB]" value={fmt(distAB!=null?distAB/100:null,3)} unit="m" sub={distAB!=null?`${distAB.toFixed(1)} cm`:"-"} updatedAt={distABAt} note={TOPICS.DIST_AB}/>
            <KPI icon={<Ruler className="h-4 w-4"/>} title="Spacing Gap [C]"        value={fmt(distC!=null?distC/100:null,3)}   unit="m" sub={distC!=null?`${distC.toFixed(1)} cm`:"-"}   updatedAt={distCAt}  note={TOPICS.DIST_C}/>
            <KPI icon={<Ruler className="h-4 w-4"/>} title="Cabbage Interval [DE]"  value={fmt(distDE!=null?distDE/100:null,3)} unit="m" sub={distDE!=null?`${distDE.toFixed(1)} cm`:"-"} updatedAt={distDEAt} note={TOPICS.DIST_DE}/>
          </div>
        </section>

        <div className="grid gap-4 md:grid-cols-2">
          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-3 flex items-center gap-2 text-base font-semibold">
              <Navigation className="h-4 w-4"/><span>Robot Position</span>
            </div>
            <div className="grid grid-cols-2 gap-3 mb-3">
              <KPI icon={<MapPin className="h-4 w-4"/>}    title="Position X" value={fmt(posX,3)}    unit="m"   updatedAt={odomAt} note="/odom"/>
              <KPI icon={<MapPin className="h-4 w-4"/>}    title="Position Y" value={fmt(posY,3)}    unit="m"   updatedAt={odomAt} note="/odom"/>
              <KPI icon={<Gauge className="h-4 w-4"/>}     title="Velocity"   value={fmt(vel,3)}     unit="m/s" updatedAt={odomAt} note="/odom twist"/>
              <KPI icon={<Navigation className="h-4 w-4"/>} title="Heading"   value={fmt(heading,1)} unit="°"   updatedAt={odomAt} note="yaw"/>
            </div>
            <div>
              <div className="flex justify-between text-xs text-slate-500 mb-1"><span>Velocity</span><span>{fmt(vel,3)} m/s</span></div>
              <div className="h-2 rounded-full bg-slate-100 overflow-hidden">
                <div className="h-full rounded-full bg-blue-500 transition-all duration-500" style={{width:vel!=null?Math.min(100,vel*100)+"%":"0%"}}/>
              </div>
            </div>
          </section>

          <section className="rounded-2xl border bg-white p-4 shadow-sm">
            <div className="mb-3 flex items-center gap-2 text-base font-semibold">
              <MapPin className="h-4 w-4"/><span>Field Map</span>
              <span className="ml-auto text-xs font-normal text-slate-400">±5 m</span>
            </div>
            <div className="relative h-52 rounded-xl border bg-slate-50 overflow-hidden"
              style={{backgroundImage:"linear-gradient(#e2e8f0 1px,transparent 1px),linear-gradient(90deg,#e2e8f0 1px,transparent 1px)",backgroundSize:"20px 20px"}}>
              {[["A","20%","30%"],["B","45%","22%"],["C","33%","68%"],["D","65%","40%"],["E","82%","62%"]].map(([l,x,y])=>(
                <div key={l} style={{position:"absolute",left:x,top:y,transform:"translate(-50%,-50%)"}} className="flex flex-col items-center gap-0.5">
                  <div className="w-3 h-3 rounded-sm border-2 border-blue-500 bg-blue-100"/>
                  <span className="text-[8px] text-blue-600 font-mono font-bold">{l}</span>
                </div>
              ))}
              <div style={{position:"absolute",left:`${mapX}%`,top:`${mapY}%`,transform:"translate(-50%,-50%)",transition:"left 0.5s ease,top 0.5s ease"}}
                className="w-4 h-4 rounded-full bg-green-500 shadow-lg border-2 border-white ring-2 ring-green-300"/>
              <div className="absolute bottom-2 right-2 bg-white rounded-lg border p-2 flex flex-col gap-1 text-[10px] text-slate-500">
                <div className="flex items-center gap-1.5"><div className="w-3 h-3 rounded-sm border-2 border-blue-500 bg-blue-100"/><span>AprilTag</span></div>
                <div className="flex items-center gap-1.5"><div className="w-3 h-3 rounded-full bg-green-500"/><span>Robot</span></div>
              </div>
            </div>
            <p className="mt-2 text-xs text-slate-400">Position from <code>/odom</code> · Field ±5 m</p>
          </section>
        </div>

        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Leaf className="h-4 w-4"/><span>Cabbage Detection</span>
            <span className="ml-auto text-sm font-normal text-slate-500">{cabbages.length} detected</span>
          </div>
          <div className="grid grid-cols-3 gap-3 mb-4">
            <div className="rounded-xl bg-green-50 border border-green-200 p-3 text-center"><div className="text-3xl font-bold text-green-600">{keepCount}</div><div className="text-xs text-green-600 mt-0.5">KEEP</div></div>
            <div className="rounded-xl bg-red-50 border border-red-200 p-3 text-center"><div className="text-3xl font-bold text-red-500">{rejectCount}</div><div className="text-xs text-red-500 mt-0.5">REJECT</div></div>
            <div className="rounded-xl bg-amber-50 border border-amber-200 p-3 text-center"><div className="text-3xl font-bold text-amber-600">{keepRate}%</div><div className="text-xs text-amber-600 mt-0.5">KEEP RATE</div></div>
          </div>
          {cabbages.length===0
            ? <div className="rounded-xl border border-dashed p-8 text-center text-sm text-slate-400">Waiting for YOLO detections on <code>{TOPICS.YOLO}</code>...</div>
            : <div className="grid gap-2 max-h-64 overflow-y-auto pr-1">
                {[...cabbages].reverse().map(c=>(
                  <div key={c.id} className={`flex items-center gap-3 rounded-xl border p-3 ${c.keep?"border-green-200 bg-green-50":"border-red-200 bg-red-50"}`}>
                    <span className="text-xl">🥬</span>
                    <div className="flex-1">
                      <div className="flex items-center gap-2">
                        <span className="font-mono text-xs text-slate-500">{c.id}</span>
                        <span className="text-sm font-semibold">{c.size.toFixed(1)} cm</span>
                        <span className="text-xs text-slate-400">conf: {c.score}</span>
                      </div>
                      <div className="mt-1 h-1.5 rounded-full bg-slate-200 overflow-hidden">
                        <div className={`h-full rounded-full transition-all ${c.keep?"bg-green-500":"bg-red-400"}`} style={{width:Math.min(100,(c.size/40)*100)+"%"}}/>
                      </div>
                    </div>
                    <span className={`rounded-full px-3 py-1 text-xs font-bold ${c.keep?"bg-green-200 text-green-800":"bg-red-200 text-red-800"}`}>{c.keep?"KEEP":"REJECT"}</span>
                  </div>
                ))}
              </div>
          }
        </section>

        <section className="rounded-2xl border bg-white p-4 shadow-sm">
          <div className="mb-3 flex items-center gap-2 text-base font-semibold">
            <Activity className="h-4 w-4"/><span>Robot Movement Graph — Position vs. Time</span>
            <span className="ml-auto text-xs font-normal text-slate-400">T+{graph.length>0?graph[graph.length-1].t.toFixed(0):0}s · {graph.length} pts</span>
          </div>
          <canvas ref={canvasRef} style={{width:"100%",height:"260px",display:"block"}}/>
          <div className="mt-3 flex flex-wrap gap-4 text-xs text-slate-500">
            <div className="flex items-center gap-1.5"><div className="w-5 h-0.5 rounded bg-blue-500"/><span>Position X (m)</span></div>
            <div className="flex items-center gap-1.5"><div className="w-5 h-0.5 rounded bg-green-500"/><span>Position Y (m)</span></div>
            <div className="flex items-center gap-1.5"><div className="w-5 h-0.5 rounded bg-amber-500"/><span>Velocity (m/s)</span></div>
            <div className="flex items-center gap-1.5"><div className="w-5 border-t-2 border-dashed border-red-400"/><span>Reference 3.0 m</span></div>
          </div>
          <div className="mt-3 flex flex-wrap gap-4 rounded-xl bg-slate-50 border p-3 text-xs text-slate-500">
            <span>Sample rate: 20 Hz</span>
            <span>Window: 300 samples</span>
            <span>Max X: <b className="text-blue-600">{graph.length>0?Math.max(...graph.map(p=>p.x)).toFixed(3):"--"} m</b></span>
            <span>Max Y: <b className="text-green-600">{graph.length>0?Math.max(...graph.map(p=>p.y)).toFixed(3):"--"} m</b></span>
            <span>Avg V: <b className="text-amber-600">{graph.length>0?(graph.reduce((a,p)=>a+p.v,0)/graph.length).toFixed(3):"--"} m/s</b></span>
          </div>
        </section>

      </div>
    </div>
  );
}

function LabeledInput({ label, value, onChange, placeholder }: {
  label: string; value: string; onChange: (v:string)=>void; placeholder?: string;
}) {
  return (
    <div className="flex flex-col gap-1">
      <label className="text-xs text-slate-500">{label}</label>
      <input value={value} onChange={e=>onChange(e.target.value)} placeholder={placeholder}
        className="rounded-xl border px-3 py-2 text-sm outline-none focus:ring-2 focus:ring-blue-200 placeholder-slate-400"/>
    </div>
  );
}

function KPI({ icon, title, value, unit, sub, updatedAt, note }: {
  icon: React.ReactNode; title: string; value: string|number;
  unit?: string; sub?: string; updatedAt: number|null; note?: string;
}) {
  return (
    <section className="rounded-xl border bg-slate-50 p-3">
      <div className="mb-1 flex items-center gap-1.5 text-sm font-semibold text-slate-600">
        {icon}<span>{title}</span>
      </div>
      <div className="flex items-end justify-between gap-2">
        <div className="text-3xl font-semibold tracking-tight">
          {value} {unit&&<span className="text-sm font-normal text-slate-400">{unit}</span>}
        </div>
        <div className="text-right">
          {sub&&<div className="text-xs text-slate-400">{sub}</div>}
          <div className="text-xs text-slate-400">{updatedAt?new Date(updatedAt).toLocaleTimeString():"-"}</div>
        </div>
      </div>
      {note&&<div className="mt-1 text-xs text-slate-400"><code>{note}</code></div>}
    </section>
  );
}