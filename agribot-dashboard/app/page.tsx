"use client";

import React, { useEffect, useRef, useState, useCallback } from "react";
import ROSLIB from "roslib";
import {
  Camera, Gauge, PlugZap, Satellite, Signal, Leaf,
  Ruler, Activity, Navigation, Cpu, Zap, CheckCircle2,
  XCircle, Clock, Bot, Sprout, ScanLine
} from "lucide-react";

// ─── Topic Registry ───────────────────────────────────────────────────────────
const TOPICS = {
  FRONT_CAMERA    : "/camera1/image_detected",
  TOP_CAMERA      : "/camera2/image_detected",
  MISSION_PARAMS  : "/mission_params",
  DISTANCE        : "/quin/distance_inside_planter",
  MOVE_COMMAND    : "/quin/move_command",
  CMD_MOVE        : "/quin/cmd_move",
  MOVE_DONE       : "/quin/move_done",
  MISSION_STATUS  : "/quin/mission_status",
  MISSION_RESTART : "/quin/mission_restart",
  DEBUG_ENCODER   : "/quin/debug/encoder",
  DEBUG_MOTOR     : "/quin/debug/motor",
  PLANT_CMD       : "/plant_cmd",
  PLANT_DONE      : "/plant_done",
  ACTUATOR_ACK    : "/microros/actuator_ack",
  ENTRY_START     : "/entry_start",
  ENTRY_DONE      : "/entry_done",
  DETECT_TRIGGER  : "/quin/detect_trigger",
  DETECT_RESULT   : "/quin/detect_result",
  CABBAGE_LOG     : "/quin/cabbage_log",
  ROBOT_READY     : "/quin/robot_ready",
  TRIGGER_RESET   : "/quin/trigger_reset",
};

// ─── Types ────────────────────────────────────────────────────────────────────
interface GraphPoint   { t: number; dist: number; cmdVel: number; }
interface CabbageEntry { index: number; size_cm: number; harvestable: number; status: string; }

type MissionPhase =
  | "IDLE" | "ENTERING" | "RESETTING" | "MOVING"
  | "PLANTING" | "DETECTING" | "DONE" | "ABORTED";

function inferPhase(flags: {
  entryActive: boolean; robotReady: boolean; plantBusy: boolean;
  detectActive: boolean; missionStatus: boolean | null; isMoving: boolean;
}): MissionPhase {
  if (flags.missionStatus === true)  return "DONE";
  if (flags.missionStatus === false) return "ABORTED";
  if (flags.entryActive)             return "ENTERING";
  if (!flags.robotReady)             return "RESETTING";
  if (flags.plantBusy)               return "PLANTING";
  if (flags.detectActive)            return "DETECTING";
  if (flags.isMoving)                return "MOVING";
  return "IDLE";
}

const PHASE_STYLE: Record<MissionPhase, { label: string; color: string; bg: string; border: string }> = {
  IDLE:      { label: "IDLE",      color: "text-gray-400",   bg: "bg-gray-800",   border: "border-gray-700" },
  ENTERING:  { label: "ENTERING",  color: "text-cyan-400",   bg: "bg-cyan-950",   border: "border-cyan-800" },
  RESETTING: { label: "RESETTING", color: "text-yellow-400", bg: "bg-yellow-950", border: "border-yellow-800" },
  MOVING:    { label: "MOVING",    color: "text-blue-400",   bg: "bg-blue-950",   border: "border-blue-800" },
  PLANTING:  { label: "PLANTING",  color: "text-orange-400", bg: "bg-orange-950", border: "border-orange-800" },
  DETECTING: { label: "DETECTING", color: "text-purple-400", bg: "bg-purple-950", border: "border-purple-800" },
  DONE:      { label: "COMPLETE",  color: "text-green-400",  bg: "bg-green-950",  border: "border-green-800" },
  ABORTED:   { label: "ABORTED",   color: "text-red-400",    bg: "bg-red-950",    border: "border-red-800" },
};

// ─────────────────────────────────────────────────────────────────────────────
export default function Page() {

  // Connection
  const [wsUrl,  setWsUrl]  = useState("ws://10.129.196.237:9090");
  const [mjpeg1, setMjpeg1] = useState("http://10.129.196.237:8081/stream?topic=/camera1/image_detected");
  const [mjpeg2, setMjpeg2] = useState("http://10.129.196.237:8081/stream?topic=/camera2/image_detected");
  const rosRef              = useRef<ROSLIB.Ros | null>(null);
  const [isConnected,   setIsConnected]   = useState(false);
  const [autoReconnect, setAutoReconnect] = useState(true);

  // Mission params
  const [AB, setAB] = useState<number | null>(null);
  const [C,  setC]  = useState<number | null>(null);
  const [DE, setDE] = useState<number | null>(null);
  const [paramsAt, setParamsAt] = useState<number | null>(null);

  // Movement
  const [distance,   setDistance]   = useState<number | null>(null);
  const [distanceAt, setDistanceAt] = useState<number | null>(null);
  const [cmdVelX,    setCmdVelX]    = useState<number | null>(null);
  const [targetDist, setTargetDist] = useState<number | null>(null);
  const [moveDone,   setMoveDone]   = useState<boolean | null>(null);
  const [moveDoneAt, setMoveDoneAt] = useState<number | null>(null);
  const [isMoving,   setIsMoving]   = useState(false);

  // Mission status
  const [missionStatus, setMissionStatus] = useState<boolean | null>(null);
  const [missionAt,     setMissionAt]     = useState<number | null>(null);
  const [robotReady,    setRobotReady]    = useState(false);

  // Entry
  const [entryActive, setEntryActive] = useState(false);
  const [entryDone,   setEntryDone]   = useState<boolean | null>(null);
  const [entryAt,     setEntryAt]     = useState<number | null>(null);

  // Detection
  const [detectActive, setDetectActive] = useState(false);
  const [detectResult, setDetectResult] = useState<string | null>(null);
  const [detectAt,     setDetectAt]     = useState<number | null>(null);

  // Cabbage log
  const [cabbageLog, setCabbageLog] = useState<CabbageEntry[]>([]);

  // Planting
  const [plantBusy,   setPlantBusy]   = useState(false);
  const [plantDone,   setPlantDone]   = useState<boolean | null>(null);
  const [actuatorAck, setActuatorAck] = useState<boolean | null>(null);
  const [plantAt,     setPlantAt]     = useState<number | null>(null);

  // Debug
  const [encTicks,  setEncTicks]  = useState<{ l: number; r: number } | null>(null);
  const [motorDuty, setMotorDuty] = useState<{ l: number; r: number } | null>(null);

  // Graph
  const [graph,   setGraph]   = useState<GraphPoint[]>([]);
  const t0Ref     = useRef<number | null>(null);
  const cmdVelRef = useRef<number>(0);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const subsRef   = useRef<ROSLIB.Topic[]>([]);

  const phase = inferPhase({ entryActive, robotReady, plantBusy, detectActive, missionStatus, isMoving });
  const ps    = PHASE_STYLE[phase];

  useEffect(() => { return () => { try { rosRef.current?.close(); } catch {} }; }, []);

  // ── Connect ────────────────────────────────────────────────────────────────
  const connect = useCallback(() => {
    try { rosRef.current?.close(); } catch {}
    const ros = new ROSLIB.Ros({ url: wsUrl });
    rosRef.current = ros;
    ros.on("connection", () => { setIsConnected(true); subscribeAll(ros); });
    ros.on("close",      () => { setIsConnected(false); if (autoReconnect) setTimeout(connect, 1500); });
    ros.on("error",      () => setIsConnected(false));
  }, [wsUrl, autoReconnect]);

  const disconnect = () => { setAutoReconnect(false); try { rosRef.current?.close(); } catch {} };

  const publish = (name: string, type: string, msg: object) => {
    if (!rosRef.current || !isConnected) return;
    const t = new ROSLIB.Topic({ ros: rosRef.current, name, messageType: type });
    t.publish(new ROSLIB.Message(msg));
  };

  // ── Subscribe all ──────────────────────────────────────────────────────────
  const subscribeAll = (ros: ROSLIB.Ros) => {
    subsRef.current.forEach(s => s.unsubscribe());
    subsRef.current = [];
    const sub = (name: string, type: string, cb: (m: any) => void, throttle = 100) => {
      const t = new ROSLIB.Topic({ ros, name, messageType: type, throttle_rate: throttle, queue_size: 1 });
      t.subscribe(cb); subsRef.current.push(t);
    };

    sub(TOPICS.MISSION_PARAMS, "std_msgs/msg/Int32MultiArray", (m) => {
      if (m.data?.length >= 3) { setAB(m.data[0]); setC(m.data[1]); setDE(m.data[2]); setParamsAt(Date.now()); }
    });

    sub(TOPICS.DISTANCE, "std_msgs/msg/Float32", (m) => {
      const v = Number(m?.data); if (!Number.isFinite(v)) return;
      setDistance(v); setDistanceAt(Date.now());
      const now = Date.now() / 1000;
      if (!t0Ref.current) t0Ref.current = now;
      const t = parseFloat((now - t0Ref.current).toFixed(2));
      setGraph(prev => { const next = [...prev, { t, dist: v, cmdVel: cmdVelRef.current }]; return next.length > 300 ? next.slice(-300) : next; });
    }, 50);

    sub(TOPICS.MOVE_COMMAND, "geometry_msgs/msg/Twist", (m) => {
      const vx = Number(m?.linear?.x ?? 0); cmdVelRef.current = vx; setCmdVelX(vx); setIsMoving(Math.abs(vx) > 0.001);
    }, 50);

    sub(TOPICS.CMD_MOVE,       "std_msgs/msg/Float32", (m) => { const v = Number(m?.data); if (Number.isFinite(v)) { setTargetDist(v); setMoveDone(null); } });
    sub(TOPICS.MOVE_DONE,      "std_msgs/msg/Bool",    (m) => { setMoveDone(m?.data); setMoveDoneAt(Date.now()); setIsMoving(false); });
    sub(TOPICS.MISSION_STATUS, "std_msgs/msg/Bool",    (m) => { setMissionStatus(m?.data); setMissionAt(Date.now()); });
    sub(TOPICS.ROBOT_READY,    "std_msgs/msg/Bool",    (m) => { if (m?.data) setRobotReady(true); });

    sub(TOPICS.ENTRY_START, "std_msgs/msg/Bool", (m) => { if (m?.data) { setEntryActive(true); setEntryDone(null); } });
    sub(TOPICS.ENTRY_DONE,  "std_msgs/msg/Bool", (m) => { setEntryDone(m?.data); setEntryActive(false); setEntryAt(Date.now()); });

    sub(TOPICS.DETECT_TRIGGER, "std_msgs/msg/Bool",   (m) => { if (m?.data) { setDetectActive(true); setDetectResult(null); } });
    sub(TOPICS.DETECT_RESULT,  "std_msgs/msg/String", (m) => { setDetectResult(m?.data ?? null); setDetectActive(false); setDetectAt(Date.now()); });
    sub(TOPICS.CABBAGE_LOG,    "std_msgs/msg/String", (m) => { try { setCabbageLog(JSON.parse(m?.data ?? "[]")); } catch {} }, 200);

    sub(TOPICS.PLANT_CMD,    "std_msgs/msg/Bool", (m) => { if (m?.data) setPlantBusy(true); });
    sub(TOPICS.PLANT_DONE,   "std_msgs/msg/Bool", (m) => { setPlantDone(m?.data); setPlantBusy(false); setPlantAt(Date.now()); });
    sub(TOPICS.ACTUATOR_ACK, "std_msgs/msg/Bool", (m) => { setActuatorAck(m?.data); setPlantAt(Date.now()); });

    sub(TOPICS.DEBUG_ENCODER, "geometry_msgs/msg/Twist", (m) => {
      setEncTicks({ l: Number(m?.linear?.x ?? 0) + Number(m?.linear?.z ?? 0), r: Number(m?.linear?.y ?? 0) + Number(m?.angular?.z ?? 0) });
    }, 200);
    sub(TOPICS.DEBUG_MOTOR, "geometry_msgs/msg/Twist", (m) => {
      setMotorDuty({ l: Number(m?.linear?.y ?? 0), r: Number(m?.linear?.z ?? 0) });
    }, 200);
  };

  // ── Graph renderer ─────────────────────────────────────────────────────────
  useEffect(() => {
    const canvas = canvasRef.current; if (!canvas) return;
    const DPR = window.devicePixelRatio || 1;
    const W = canvas.offsetWidth, H = canvas.offsetHeight;
    canvas.width = W * DPR; canvas.height = H * DPR;
    const ctx = canvas.getContext("2d")!; ctx.scale(DPR, DPR);
    const M = { top: 24, right: 16, bottom: 44, left: 56 };
    const CW = W - M.left - M.right, CH = H - M.top - M.bottom;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0f172a"; ctx.fillRect(0, 0, W, H);
    ctx.fillStyle = "#1e293b"; ctx.fillRect(M.left, M.top, CW, CH);
    const pts = graph;
    const dMax = pts.length > 0 ? Math.max(...pts.map(p => p.dist), 1.5) : 1.5;
    const tMax = pts.length > 0 ? Math.max(pts[pts.length - 1].t, 10) : 60;
    const toX  = (t: number) => M.left + (t / tMax) * CW;
    const toYD = (v: number) => M.top + CH - (v / dMax) * CH;
    const toYV = (v: number) => M.top + CH - (v / 0.3) * CH;
    ctx.strokeStyle = "#334155"; ctx.lineWidth = 1;
    for (let v = 0; v <= dMax; v += 0.5) { ctx.beginPath(); ctx.moveTo(M.left, toYD(v)); ctx.lineTo(M.left + CW, toYD(v)); ctx.stroke(); }
    const tStep = tMax <= 30 ? 5 : 10;
    for (let t = 0; t <= tMax; t += tStep) { ctx.beginPath(); ctx.moveTo(toX(t), M.top); ctx.lineTo(toX(t), M.top + CH); ctx.stroke(); }
    if (targetDist != null) {
      ctx.strokeStyle = "#ef4444"; ctx.setLineDash([6, 4]); ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(M.left, toYD(targetDist)); ctx.lineTo(M.left + CW, toYD(targetDist)); ctx.stroke();
      ctx.setLineDash([]); ctx.fillStyle = "#ef4444"; ctx.font = "10px monospace"; ctx.textAlign = "left";
      ctx.fillText("TARGET=" + targetDist.toFixed(3) + "m", M.left + 4, toYD(targetDist) - 5);
    }
    ctx.strokeStyle = "#475569"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(M.left, M.top); ctx.lineTo(M.left, M.top + CH); ctx.lineTo(M.left + CW, M.top + CH); ctx.stroke();
    ctx.fillStyle = "#64748b"; ctx.font = "10px monospace"; ctx.textAlign = "right";
    for (let v = 0; v <= dMax; v += 0.5) ctx.fillText(v.toFixed(1), M.left - 5, toYD(v) + 4);
    ctx.textAlign = "center";
    for (let t = 0; t <= tMax; t += tStep) ctx.fillText(t + "s", toX(t), M.top + CH + 16);
    ctx.save(); ctx.translate(13, M.top + CH / 2); ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = "#94a3b8"; ctx.font = "11px sans-serif"; ctx.textAlign = "center";
    ctx.fillText("Distance (m)", 0, 0); ctx.restore();
    ctx.fillStyle = "#94a3b8"; ctx.font = "11px sans-serif"; ctx.textAlign = "center";
    ctx.fillText("Time (seconds)", M.left + CW / 2, H - 6);
    if (pts.length < 2) return;
    ctx.beginPath(); ctx.moveTo(toX(pts[0].t), toYD(0));
    pts.forEach(p => ctx.lineTo(toX(p.t), toYD(p.dist)));
    ctx.lineTo(toX(pts[pts.length - 1].t), toYD(0)); ctx.closePath();
    ctx.fillStyle = "rgba(59,130,246,0.15)"; ctx.fill();
    ctx.beginPath(); ctx.moveTo(toX(pts[0].t), toYV(0));
    pts.forEach(p => ctx.lineTo(toX(p.t), toYV(p.cmdVel)));
    ctx.lineTo(toX(pts[pts.length - 1].t), toYV(0)); ctx.closePath();
    ctx.fillStyle = "rgba(34,197,94,0.12)"; ctx.fill();
    ctx.beginPath(); ctx.strokeStyle = "#3b82f6"; ctx.lineWidth = 2.5; ctx.lineJoin = "round"; ctx.lineCap = "round";
    pts.forEach((p, i) => i === 0 ? ctx.moveTo(toX(p.t), toYD(p.dist)) : ctx.lineTo(toX(p.t), toYD(p.dist))); ctx.stroke();
    ctx.beginPath(); ctx.strokeStyle = "#22c55e"; ctx.lineWidth = 2; ctx.lineJoin = "round"; ctx.lineCap = "round";
    pts.forEach((p, i) => i === 0 ? ctx.moveTo(toX(p.t), toYV(p.cmdVel)) : ctx.lineTo(toX(p.t), toYV(p.cmdVel))); ctx.stroke();
    const last = pts[pts.length - 1]; const lx = toX(last.t), ly = toYD(last.dist);
    ctx.beginPath(); ctx.arc(lx, ly, 5, 0, Math.PI * 2); ctx.fillStyle = "#3b82f6"; ctx.fill();
    ctx.strokeStyle = "rgba(59,130,246,0.4)"; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(lx, ly, 9, 0, Math.PI * 2); ctx.stroke();
  }, [graph, targetDist]);

  // ── Helpers ────────────────────────────────────────────────────────────────
  const fmt   = (n: number | null, d = 3) => n == null ? "—" : n.toFixed(d);
  const fmtT  = (ts: number | null) => ts ? new Date(ts).toLocaleTimeString() : "—";
  const progress = (distance != null && targetDist != null && targetDist > 0) ? Math.min(100, (distance / targetDist) * 100) : 0;
  const harvCount     = cabbageLog.filter(e => e.harvestable === 1).length;
  const notReadyCount = cabbageLog.filter(e => e.harvestable === 0).length;
  const noDetectCount = cabbageLog.filter(e => e.harvestable === -1).length;

  // ── Render ─────────────────────────────────────────────────────────────────
  return (
    <div className="min-h-screen bg-slate-950 p-4 sm:p-6 text-slate-100">
      <div className="mx-auto max-w-7xl space-y-4">

        {/* Header */}
        <header className="flex items-center justify-between py-2">
          <div className="flex items-center gap-3">
            <Bot className={`h-7 w-7 ${isConnected ? "text-green-400" : "text-slate-600"}`} />
            <div>
              <h1 className="text-2xl font-bold text-white tracking-tight">AgriBot Monitor</h1>
              <p className="text-xs text-slate-500">Cabbage Field Robot — Full System Dashboard</p>
            </div>
          </div>
          <div className={`flex items-center gap-2 rounded-full px-3 py-1 text-sm font-medium border
            ${isConnected ? "bg-green-950 border-green-800 text-green-400" : "bg-red-950 border-red-800 text-red-400"}`}>
            <Signal className="h-4 w-4" />{isConnected ? "Connected" : "Disconnected"}
          </div>
        </header>

        {/* Connection */}
        <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
          <SectionTitle icon={<Satellite className="h-4 w-4 text-blue-400" />} title="Connection" />
          <div className="grid gap-3 md:grid-cols-4">
            <DI label="Rosbridge WebSocket URL" value={wsUrl}   onChange={setWsUrl} />
            <DI label="Camera Front MJPEG"       value={mjpeg1} onChange={setMjpeg1} />
            <DI label="Camera Top MJPEG"         value={mjpeg2} onChange={setMjpeg2} />
            <div className="flex items-end gap-2">
              <button onClick={connect} disabled={isConnected}
                className="flex-1 rounded-xl bg-blue-600 px-4 py-2 text-sm text-white font-medium hover:bg-blue-500 disabled:opacity-40 transition-colors">
                Connect
              </button>
              <button onClick={disconnect}
                className="flex-1 rounded-xl border border-slate-700 px-4 py-2 text-sm text-slate-400 hover:bg-slate-800 transition-colors">
                Disconnect
              </button>
            </div>
          </div>
          <div className="mt-3 flex items-center justify-between">
            <label className="flex items-center gap-2 text-sm text-slate-400 cursor-pointer">
              <input type="checkbox" checked={autoReconnect} onChange={e => setAutoReconnect(e.target.checked)} className="accent-blue-500" />
              Auto reconnect
            </label>
            <button
              onClick={() => publish(TOPICS.MISSION_RESTART, "std_msgs/msg/Bool", { data: true })}
              disabled={!isConnected}
              className="rounded-xl bg-yellow-700 hover:bg-yellow-600 disabled:opacity-40 px-4 py-1.5 text-sm font-medium text-white transition-colors">
              ↺ Restart Mission
            </button>
          </div>
        </section>

        {/* Mission Phase Banner */}
        <section className={`rounded-2xl border ${ps.border} ${ps.bg} p-4`}>
          <div className="flex items-center justify-between">
            <div>
              <p className="text-xs text-slate-500 mb-1">Current Mission Phase</p>
              <p className={`text-4xl font-bold tracking-widest ${ps.color}`}>{ps.label}</p>
            </div>
            <div className="grid grid-cols-3 gap-2 text-xs text-slate-500">
              <PhaseFlag label="ENTRY"   active={entryActive}            done={entryDone === true}    color="cyan" />
              <PhaseFlag label="ROBOT"   active={!robotReady}            done={robotReady}             color="yellow" />
              <PhaseFlag label="PLANT"   active={plantBusy}              done={plantDone === true}     color="orange" />
              <PhaseFlag label="DETECT"  active={detectActive}           done={detectResult != null}   color="purple" />
              <PhaseFlag label="MISSION" active={false}                  done={missionStatus === true} color="green" />
              <PhaseFlag label="ABORTED" active={missionStatus === false} done={false}                 color="red" />
            </div>
          </div>
        </section>

        {/* Cameras */}
        <div className="grid gap-4 md:grid-cols-2">
          <CameraPanel title="Camera — Front (Entry)"  topic={TOPICS.FRONT_CAMERA} src={mjpeg1} connected={isConnected} color="blue" />
          <CameraPanel title="Camera — Top (Cabbage)"  topic={TOPICS.TOP_CAMERA}   src={mjpeg2} connected={isConnected} color="green" />
        </div>

        {/* Mission Params */}
        <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
          <SectionTitle icon={<Ruler className="h-4 w-4 text-yellow-400" />} title="AprilTag Mission Parameters"
            extra={<span className="text-xs text-slate-600">{paramsAt ? "Last: " + fmtT(paramsAt) : "Waiting for AprilTag..."}</span>} />
          <div className="grid gap-4 md:grid-cols-3">
            <DK icon={<Ruler className="h-4 w-4" />} title="Planting Distance [AB]" value={fmt(AB != null ? AB / 100 : null, 3)} unit="m" sub={AB != null ? AB + " cm" : "—"} color="blue"   note={TOPICS.MISSION_PARAMS + " [0]"} />
            <DK icon={<Ruler className="h-4 w-4" />} title="Spacing Gap [C]"        value={fmt(C  != null ? C  / 100 : null, 3)} unit="m" sub={C  != null ? C  + " cm" : "—"} color="yellow" note={TOPICS.MISSION_PARAMS + " [1]"} />
            <DK icon={<Ruler className="h-4 w-4" />} title="Cabbage Interval [DE]"  value={fmt(DE != null ? DE / 100 : null, 3)} unit="m" sub={DE != null ? DE + " cm" : "—"} color="green"  note={TOPICS.MISSION_PARAMS + " [2]"} />
          </div>
        </section>

        {/* Movement + Mission Status */}
        <div className="grid gap-4 md:grid-cols-2">

          {/* Movement */}
          <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
            <SectionTitle icon={<Gauge className="h-4 w-4 text-blue-400" />} title="Distance Inside Planter" />
            <div className="grid grid-cols-2 gap-3 mb-4">
              <DK icon={<Gauge className="h-4 w-4" />}      title="Current" value={fmt(distance, 3)}   unit="m" sub={fmtT(distanceAt)} color="blue"   note={TOPICS.DISTANCE} />
              <DK icon={<Navigation className="h-4 w-4" />} title="Target"  value={fmt(targetDist, 3)} unit="m" sub="from mission"      color="purple" note={TOPICS.CMD_MOVE} />
            </div>
            <div className="mb-3">
              <div className="flex justify-between text-xs text-slate-500 mb-2">
                <span>Progress to target</span>
                <span className="text-slate-300 font-medium">{progress.toFixed(1)}%</span>
              </div>
              <div className="h-3 rounded-full bg-slate-800 overflow-hidden">
                <div className={`h-full rounded-full transition-all duration-500 ${moveDone === true ? "bg-green-500" : "bg-blue-500"}`} style={{ width: progress + "%" }} />
              </div>
              <div className="flex justify-between text-xs text-slate-600 mt-1">
                <span>0 m</span>
                <span className={moveDone === true ? "text-green-400 font-semibold" : moveDone === false ? "text-red-400 font-semibold" : "text-slate-500"}>
                  {moveDone === true ? "✓ REACHED" : moveDone === false ? "✗ FAILED" : "Moving..."}
                </span>
                <span>{fmt(targetDist, 3)} m</span>
              </div>
            </div>
            <div>
              <div className="flex justify-between text-xs text-slate-500 mb-2">
                <span>Commanded velocity</span>
                <span className="text-green-400 font-medium">{fmt(cmdVelX, 3)} m/s</span>
              </div>
              <div className="h-2 rounded-full bg-slate-800 overflow-hidden">
                <div className="h-full rounded-full bg-green-500 transition-all duration-300"
                  style={{ width: cmdVelX != null ? Math.min(100, Math.abs(cmdVelX) / 0.3 * 100) + "%" : "0%" }} />
              </div>
            </div>
          </section>

          {/* Mission + Plant */}
          <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
            <SectionTitle icon={<Leaf className="h-4 w-4 text-green-400" />} title="Mission & Planting Status" />
            <StatusRow label="Entry Phase"  topic={TOPICS.ENTRY_DONE}      active={entryActive}  success={entryDone === true}     failed={entryDone === false}   activeText="ENTERING BOX..." successText="✓ ENTERED"  failText="✗ FAILED"  idleText="— WAITING"   time={entryAt} />
            <StatusRow label="Robot Ready"  topic={TOPICS.ROBOT_READY}     active={!robotReady && phase === "RESETTING"} success={robotReady} failed={false} activeText="RESETTING..."    successText="✓ READY"    failText=""          idleText="— NOT READY" time={null} />
            <StatusRow label="Mission"      topic={TOPICS.MISSION_STATUS}  active={phase !== "IDLE" && phase !== "DONE" && phase !== "ABORTED"} success={missionStatus === true} failed={missionStatus === false} activeText="RUNNING..." successText="✓ COMPLETE" failText="✗ ABORTED" idleText="— IDLE" time={missionAt} />
            <StatusRow label="Planting"     topic={TOPICS.PLANT_DONE}      active={plantBusy}    success={plantDone === true}     failed={plantDone === false}   activeText="⟳ PLANTING..." successText="✓ PLANTED" failText="✗ FAILED"  idleText="— IDLE"      time={plantAt} />
            <div className="rounded-xl border border-slate-700 bg-slate-800 p-3">
              <div className="text-xs text-slate-500 mb-1">Actuator ACK · <code className="text-slate-600">{TOPICS.ACTUATOR_ACK}</code></div>
              <div className={`text-xl font-bold ${actuatorAck === true ? "text-green-400" : actuatorAck === false ? "text-red-400" : "text-slate-600"}`}>
                {actuatorAck === true ? "✓ ACK" : actuatorAck === false ? "✗ NACK" : "—"}
              </div>
            </div>
          </section>
        </div>

        {/* Cabbage Detection */}
        <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
          <SectionTitle icon={<ScanLine className="h-4 w-4 text-purple-400" />} title="Cabbage Detection"
            extra={
              <span className={`text-xs px-2 py-0.5 rounded-full border
                ${detectActive ? "bg-purple-950 border-purple-800 text-purple-400" : "bg-slate-800 border-slate-700 text-slate-500"}`}>
                {detectActive ? "● SCANNING" : detectResult ? "● RESULT RECEIVED" : "● IDLE"}
              </span>
            } />
          <div className="grid gap-3 md:grid-cols-3 mb-4">
            <DK icon={<ScanLine className="h-4 w-4" />}    title="Last Size"      value={detectResult ? detectResult.split(",")[0] : "—"} unit="cm" sub={fmtT(detectAt)} color="purple" note={TOPICS.DETECT_RESULT} />
            <DK icon={<CheckCircle2 className="h-4 w-4" />} title="Harvestable"   value={detectResult ? (detectResult.split(",")[1] === "1" ? "YES ✓" : detectResult.split(",")[1] === "0" ? "NOT YET" : "N/D") : "—"} unit="" sub="" color={detectResult?.split(",")[1] === "1" ? "green" : "yellow"} note="" />
            <DK icon={<Sprout className="h-4 w-4" />}       title="Total Scanned" value={String(cabbageLog.length)} unit="" sub={`✓ ${harvCount} ready`} color="blue" note={TOPICS.CABBAGE_LOG} />
          </div>

          {cabbageLog.length > 0 ? (
            <div className="rounded-xl border border-slate-700 overflow-hidden">
              <div className="bg-slate-800 px-4 py-2 flex items-center justify-between">
                <span className="text-xs font-semibold text-slate-400">Scan Log</span>
                <div className="flex gap-3 text-xs">
                  <span className="text-green-400">✓ {harvCount} ready</span>
                  <span className="text-orange-400">⏳ {notReadyCount} not ready</span>
                  <span className="text-slate-500">— {noDetectCount} not detected</span>
                </div>
              </div>
              <div className="max-h-48 overflow-y-auto">
                <table className="w-full text-sm">
                  <thead className="sticky top-0 bg-slate-800 text-xs text-slate-500">
                    <tr>
                      <th className="px-4 py-2 text-left">#</th>
                      <th className="px-4 py-2 text-left">Size (cm)</th>
                      <th className="px-4 py-2 text-left">Status</th>
                    </tr>
                  </thead>
                  <tbody>
                    {cabbageLog.map((e, i) => (
                      <tr key={i} className="border-t border-slate-800">
                        <td className="px-4 py-2 text-slate-500">{e.index}</td>
                        <td className="px-4 py-2 font-mono text-slate-200">{e.size_cm.toFixed(1)}</td>
                        <td className={`px-4 py-2 font-medium ${e.harvestable === 1 ? "text-green-400" : e.harvestable === 0 ? "text-orange-400" : "text-slate-500"}`}>
                          {e.status}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </div>
          ) : (
            <div className="rounded-xl border border-slate-700 bg-slate-800 p-6 text-center text-slate-600 text-sm">
              No cabbage data yet — waiting for mission to reach cabbage zone...
            </div>
          )}
        </section>

        {/* ESP32 Debug */}
        <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
          <SectionTitle icon={<Cpu className="h-4 w-4 text-purple-400" />} title="ESP32 Debug — PID & Encoder" />
          <div className="grid gap-3 md:grid-cols-4">
            <DK icon={<Zap className="h-4 w-4" />}   title="Left Encoder Δ"   value={fmt(encTicks?.l ?? null, 0)}  unit="ticks" color="blue"   note="M1+M3" />
            <DK icon={<Zap className="h-4 w-4" />}   title="Right Encoder Δ"  value={fmt(encTicks?.r ?? null, 0)}  unit="ticks" color="green"  note="M2+M4" />
            <DK icon={<Gauge className="h-4 w-4" />} title="Left Motor Duty"  value={fmt(motorDuty?.l ?? null, 3)} unit=""      color="yellow" note="PID" />
            <DK icon={<Gauge className="h-4 w-4" />} title="Right Motor Duty" value={fmt(motorDuty?.r ?? null, 3)} unit=""      color="purple" note="PID" />
          </div>
          {motorDuty && (
            <div className="mt-3 grid gap-2 md:grid-cols-2">
              <div>
                <div className="flex justify-between text-xs text-slate-500 mb-1"><span>Left Duty</span><span className="text-blue-400">{fmt(motorDuty.l, 3)}</span></div>
                <div className="h-2 rounded-full bg-slate-800 overflow-hidden"><div className="h-full rounded-full bg-blue-500 transition-all" style={{ width: Math.min(100, motorDuty.l * 100) + "%" }} /></div>
              </div>
              <div>
                <div className="flex justify-between text-xs text-slate-500 mb-1"><span>Right Duty</span><span className="text-green-400">{fmt(motorDuty.r, 3)}</span></div>
                <div className="h-2 rounded-full bg-slate-800 overflow-hidden"><div className="h-full rounded-full bg-green-500 transition-all" style={{ width: Math.min(100, motorDuty.r * 100) + "%" }} /></div>
              </div>
            </div>
          )}
        </section>

        {/* Graph */}
        <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
          <SectionTitle icon={<Activity className="h-4 w-4 text-blue-400" />} title="Movement Graph — Distance vs Time"
            extra={<span className="text-xs text-slate-600">T+{graph.length > 0 ? graph[graph.length - 1].t.toFixed(0) : 0}s · {graph.length} pts</span>} />
          <canvas ref={canvasRef} style={{ width: "100%", height: "260px", display: "block", borderRadius: "12px" }} />
          <div className="mt-3 flex flex-wrap gap-4 text-xs text-slate-500">
            <div className="flex items-center gap-1.5"><div className="w-5 h-0.5 rounded bg-blue-500" /><span>Actual Distance (m)</span></div>
            <div className="flex items-center gap-1.5"><div className="w-5 h-0.5 rounded bg-green-500" /><span>Commanded Velocity (m/s)</span></div>
            <div className="flex items-center gap-1.5"><div className="w-5 border-t-2 border-dashed border-red-500" /><span>Target Distance</span></div>
          </div>
          <div className="mt-3 flex flex-wrap gap-4 rounded-xl bg-slate-800 border border-slate-700 p-3 text-xs text-slate-500">
            <span>Max dist: <b className="text-blue-400">{graph.length > 0 ? Math.max(...graph.map(p => p.dist)).toFixed(3) : "—"} m</b></span>
            <span>Max vel:  <b className="text-green-400">{graph.length > 0 ? Math.max(...graph.map(p => p.cmdVel)).toFixed(3) : "—"} m/s</b></span>
          </div>
        </section>

      </div>
    </div>
  );
}

// ─── Sub-components ───────────────────────────────────────────────────────────

function SectionTitle({ icon, title, extra }: { icon: React.ReactNode; title: string; extra?: React.ReactNode }) {
  return (
    <div className="mb-3 flex items-center gap-2 text-base font-semibold text-slate-200">
      {icon}<span>{title}</span>{extra && <span className="ml-auto">{extra}</span>}
    </div>
  );
}

function CameraPanel({ title, topic, src, connected, color }: { title: string; topic: string; src: string; connected: boolean; color: string }) {
  const iconColor = color === "blue" ? "text-blue-400" : "text-green-400";
  return (
    <section className="rounded-2xl border border-slate-800 bg-slate-900 p-4">
      <div className="mb-2 flex items-center gap-2 text-base font-semibold text-slate-200">
        <Camera className={`h-4 w-4 ${iconColor}`} /><span>{title}</span>
        <span className={`ml-auto text-xs px-2 py-0.5 rounded-full border ${connected ? "bg-green-950 border-green-800 text-green-400" : "bg-slate-800 border-slate-700 text-slate-500"}`}>
          {connected ? "● LIVE" : "● WAITING"}
        </span>
      </div>
      <div className="aspect-video w-full overflow-hidden rounded-xl border border-slate-800 bg-black">
        {connected ? <img src={src} alt={title} className="h-full w-full object-contain" />
          : <div className="flex h-full items-center justify-center text-slate-600 text-sm">Connect first</div>}
      </div>
      <p className="mt-2 text-xs text-slate-700"><code className="text-slate-600">{topic}</code></p>
    </section>
  );
}

function PhaseFlag({ label, active, done, color }: { label: string; active: boolean; done: boolean; color: string }) {
  const colorMap: Record<string, string> = {
    cyan: "text-cyan-400", yellow: "text-yellow-400", orange: "text-orange-400",
    purple: "text-purple-400", green: "text-green-400", red: "text-red-400",
  };
  const c = colorMap[color] ?? "text-slate-400";
  return (
    <div className={`flex items-center gap-1 px-2 py-1 rounded-lg border text-xs font-medium
      ${done ? "border-green-800 bg-green-950 text-green-400"
        : active ? `border-slate-600 bg-slate-800 ${c}`
        : "border-slate-800 bg-slate-900 text-slate-600"}`}>
      {done ? <CheckCircle2 className="h-3 w-3" /> : active ? <Clock className="h-3 w-3" /> : <XCircle className="h-3 w-3 opacity-30" />}
      {label}
    </div>
  );
}

function StatusRow({ label, topic, active, success, failed, activeText, successText, failText, idleText, time }:
  { label: string; topic: string; active: boolean; success: boolean; failed: boolean; activeText: string; successText: string; failText: string; idleText: string; time: number | null }) {
  const fmtT = (ts: number | null) => ts ? new Date(ts).toLocaleTimeString() : "—";
  return (
    <div className={`rounded-xl border p-3 mb-2
      ${success ? "border-green-800 bg-green-950"
        : failed  ? "border-red-800 bg-red-950"
        : active  ? "border-yellow-800 bg-yellow-950"
        : "border-slate-700 bg-slate-800"}`}>
      <div className="text-xs text-slate-500 mb-1">{label} · <code>{topic}</code></div>
      <div className={`text-lg font-bold ${success ? "text-green-400" : failed ? "text-red-400" : active ? "text-yellow-400" : "text-slate-600"}`}>
        {success ? successText : failed ? failText : active ? activeText : idleText}
      </div>
      {time && <div className="text-xs text-slate-600 mt-0.5">{fmtT(time)}</div>}
    </div>
  );
}

function DI({ label, value, onChange }: { label: string; value: string; onChange: (v: string) => void }) {
  return (
    <div className="flex flex-col gap-1">
      <label className="text-xs text-slate-500">{label}</label>
      <input value={value} onChange={e => onChange(e.target.value)}
        className="rounded-xl border border-slate-700 bg-slate-800 px-3 py-2 text-sm text-slate-200 outline-none focus:border-blue-500" />
    </div>
  );
}

const CM: Record<string, { v: string; bg: string; b: string }> = {
  blue:   { v: "text-blue-400",   bg: "bg-blue-950",   b: "border-blue-900" },
  green:  { v: "text-green-400",  bg: "bg-green-950",  b: "border-green-900" },
  yellow: { v: "text-yellow-400", bg: "bg-yellow-950", b: "border-yellow-900" },
  purple: { v: "text-purple-400", bg: "bg-purple-950", b: "border-purple-900" },
  orange: { v: "text-orange-400", bg: "bg-orange-950", b: "border-orange-900" },
};

function DK({ icon, title, value, unit, sub, note, color = "blue" }:
  { icon: React.ReactNode; title: string; value: string | number; unit?: string; sub?: string; note?: string; color?: string }) {
  const c = CM[color] ?? CM.blue;
  return (
    <div className={`rounded-xl border ${c.b} ${c.bg} p-3`}>
      <div className={`mb-1 flex items-center gap-1.5 text-sm font-semibold ${c.v}`}>{icon}<span className="text-slate-400">{title}</span></div>
      <div className="flex items-end justify-between gap-2">
        <div className={`text-3xl font-semibold tracking-tight ${c.v}`}>
          {value}{unit && <span className="text-sm font-normal text-slate-600 ml-1">{unit}</span>}
        </div>
        {sub && <div className="text-xs text-slate-600">{sub}</div>}
      </div>
      {note && <div className="mt-1 text-xs text-slate-700"><code>{note}</code></div>}
    </div>
  );
}