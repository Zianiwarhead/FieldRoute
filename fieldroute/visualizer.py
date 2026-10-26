"""
Matplotlib publication rendering and HTML interactive telemetry dashboard export
with real-time state-of-charge / fuel tracking, teardrop headland turns, and obstacle clearance.
"""

from __future__ import annotations
import json
from typing import List, Tuple, Dict, Any, Callable
from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from .core import Tractor

DockFn = Callable[[float], Tuple[float, float]]


def plot(
    field: Polygon,
    obstacles: List[Polygon],
    res: Any,
    dock: DockFn,
    metrics: Dict[str, Any],
    path: str = "fieldroute.png",
) -> None:
    """Render high-resolution agricultural mission map with telemetry HUD overlay."""
    fig, ax = plt.subplots(figsize=(11, 8.5), dpi=150)
    fig.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("#f1f5f9")

    # 1. Field boundary & obstacles
    ax.fill(*field.exterior.xy, color="#dcfce7", ec="#15803d", lw=2.0, label="Field Boundary", zorder=1)
    for i, o in enumerate(obstacles):
        ax.fill(*o.exterior.xy, color="#bfdbfe", ec="#1d4ed8", lw=1.8, hatch="//",
                label="Obstacle (Pond/Trees)" if i == 0 else None, zorder=2)

    # 2. Tractor swaths, teardrop turns, and obstacle-avoiding transits
    for seg in res["segments"]:
        a, b = seg["p_start"], seg["p_end"]
        seg_type = seg.get("seg_type", "")
        if seg["is_working"]:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#0284c7", lw=1.5, ls="-", zorder=3)
        elif seg_type == "TURN_HEADLAND":
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#06b6d4", lw=1.2, ls="-", alpha=0.9, zorder=4)
        elif seg_type == "TRANSIT_DOCK":
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#ef4444", lw=1.4, ls="--", alpha=0.9, zorder=4)
        else:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#94a3b8", lw=1.0, ls="--", alpha=0.75, zorder=4)

    # 3. Dock path
    pts = [dock(s) for s in range(0, int(res["time"]), 20)]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#16a34a", ls=":", lw=2.2, label="Dock Path", zorder=5)
    start_dock = dock(0)
    ax.scatter([start_dock[0]], [start_dock[1]], color="#16a34a", marker="s", s=90, edgecolors="black",
               lw=1.5, label="Dock Station", zorder=6)

    # 4. Refueling stops with badges
    for ref in res["refuels"]:
        q = ref["pos"]
        ax.scatter([q[0]], [q[1]], color="#dc2626", marker="*", s=220, edgecolors="#7f1d1d",
                   lw=1.2, zorder=7)
        ax.annotate(f"Refuel #{ref['index']}\n({ref['time_h']:.1f}h)",
                    xy=(q[0], q[1]), xytext=(q[0] + 12, q[1] + 8),
                    fontsize=8.5, fontweight="bold", color="#991b1b",
                    bbox=dict(boxstyle="round,pad=0.25", fc="#fee2e2", ec="#ef4444", lw=1),
                    zorder=8)

    ax.set_aspect("equal", "datalim")
    ax.grid(True, linestyle=":", alpha=0.6, color="#cbd5e1")
    ax.set_xlabel("Eastings (metres)", fontsize=10, fontweight="bold", color="#334155")
    ax.set_ylabel("Northings (metres)", fontsize=10, fontweight="bold", color="#334155")

    # HUD Banner
    title_text = (
        f"FieldRoute Coverage & Autonomous Refuel Simulation\n"
        f"Coverage: {metrics['coverage_pct']:.1f}% ({metrics['covered_area_ha']:.2f} ha) | "
        f"Overlap: {metrics['overlap_pct']:.1f}% | "
        f"Distance: {metrics['total_distance_km']:.1f} km | "
        f"Fuel: {metrics['fuel_used_l']:.1f} L | "
        f"Refuels: {metrics['refuels_count']} | "
        f"Duration: {metrics['mission_time_h']:.1f} h"
    )
    ax.set_title(title_text, fontsize=11, fontweight="bold", color="#0f172a", pad=12)

    # Custom Legend
    legend_elements = [
        mpatches.Patch(facecolor="#dcfce7", edgecolor="#15803d", label="Field Polygon"),
        mpatches.Patch(facecolor="#bfdbfe", edgecolor="#1d4ed8", hatch="//", label="Obstacle (Pond)"),
        plt.Line2D([0], [0], color="#0284c7", lw=2, label="Working Row"),
        plt.Line2D([0], [0], color="#06b6d4", lw=1.5, label="Teardrop Headland Turn"),
        plt.Line2D([0], [0], color="#ef4444", lw=1.5, ls="--", label="Low-Fuel Return Route"),
        plt.Line2D([0], [0], color="#94a3b8", lw=1.5, ls="--", label="Transit Route"),
        plt.Line2D([0], [0], color="#16a34a", marker="s", lw=0, label="Dock Station"),
        plt.Line2D([0], [0], color="#dc2626", marker="*", lw=0, markersize=12, label="Refuel Event"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", framealpha=0.92, facecolor="#ffffff",
              edgecolor="#cbd5e1", fontsize=8.5)

    plt.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"High-res mission map saved to: {path}")


def export_interactive_html(
    field: Polygon,
    obstacles: List[Polygon],
    res: Any,
    dock: DockFn,
    metrics: Dict[str, Any],
    tr: Tractor,
    path: str = "fieldroute_dashboard.html",
) -> None:
    """Export self-contained interactive web simulation with playback controls & live telemetry."""
    field_coords = list(field.exterior.coords)
    obs_coords = [list(o.exterior.coords) for o in obstacles]

    segments_data = []
    for s in res["segments"]:
        segments_data.append({
            "p0": s["p_start"],
            "p1": s["p_end"],
            "t0": s["t_start"],
            "t1": s["t_end"],
            "f0": s["fuel_start"],
            "f1": s["fuel_end"],
            "type": s["seg_type"],
            "label": s["status_label"],
            "working": s["is_working"],
            "warn": s["is_warning"],
        })

    refuels_data = [
        {"pos": r["pos"], "index": r["index"], "time_h": round(r["time_h"], 2)}
        for r in res["refuels"]
    ]
    dock_sample = [{"t": s, "pos": dock(s)} for s in range(0, int(res["time"]) + 30, 20)]

    payload = {
        "field": field_coords,
        "obstacles": obs_coords,
        "segments": segments_data,
        "refuels": refuels_data,
        "dock_samples": dock_sample,
        "metrics": metrics,
        "tank_capacity": tr.tank,
        "tractor_speed": tr.speed,
        "refuel_time": tr.refuel_time,
        "implement_width": tr.width,
        "min_turn_radius": tr.min_turn_radius,
    }

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>FieldRoute | Autonomous Tractor Path Planning & Telemetry</title>
<style>
  :root {{
    --bg-dark: #090d16;
    --card-bg: #131b2e;
    --border: #24324f;
    --accent: #22c55e;
    --accent-blue: #38bdf8;
    --text-main: #f8fafc;
    --text-muted: #94a3b8;
    --warn: #ef4444;
    --warn-bg: rgba(239, 68, 68, 0.15);
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }}
  body {{ background: var(--bg-dark); color: var(--text-main); display: flex; height: 100vh; overflow: hidden; }}
  #sidebar {{ width: 390px; background: var(--card-bg); border-right: 1px solid var(--border); display: flex; flex-direction: column; padding: 18px; overflow-y: auto; gap: 14px; }}
  #main {{ flex: 1; display: flex; flex-direction: column; position: relative; }}
  header {{ display: flex; align-items: center; justify-content: space-between; padding-bottom: 12px; border-bottom: 1px solid var(--border); }}
  header h1 {{ font-size: 20px; font-weight: 700; color: #fff; display: flex; align-items: center; gap: 8px; }}
  .tag {{ font-size: 11px; font-weight: 700; padding: 3px 8px; border-radius: 999px; background: rgba(34, 197, 94, 0.2); color: #4ade80; border: 1px solid #22c55e; }}
  
  .gauge-box {{ background: rgba(9, 13, 22, 0.7); border: 1px solid var(--border); border-radius: 8px; padding: 14px; }}
  .gauge-label-row {{ display: flex; justify-content: space-between; font-size: 12px; margin-bottom: 8px; font-weight: 700; }}
  .gauge-bar {{ height: 16px; background: #1e293b; border-radius: 8px; overflow: hidden; position: relative; border: 1px solid #334155; }}
  .gauge-fill {{ height: 100%; width: 100%; background: #22c55e; transition: width 0.08s linear, background-color 0.2s ease; }}
  
  #alert-banner {{ display: none; margin-top: 10px; padding: 10px 12px; border-radius: 6px; font-size: 12px; font-weight: 600; line-height: 1.4; animation: pulseAlert 1.2s infinite; }}
  #alert-banner.warning {{ display: block; background: rgba(239, 68, 68, 0.2); border: 1px solid #ef4444; color: #fca5a5; }}
  #alert-banner.refuel {{ display: block; background: rgba(34, 197, 94, 0.2); border: 1px solid #22c55e; color: #86efac; }}

  @keyframes pulseAlert {{
    0%, 100% {{ opacity: 1; }}
    50% {{ opacity: 0.65; }}
  }}

  .status-badge {{ display: inline-flex; align-items: center; gap: 6px; font-weight: 700; font-size: 12px; padding: 6px 12px; border-radius: 6px; margin-top: 10px; background: #0284c7; color: white; width: 100%; justify-content: center; }}
  
  .kpi-grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
  .kpi-card {{ background: rgba(9, 13, 22, 0.6); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; }}
  .kpi-label {{ font-size: 11px; text-transform: uppercase; letter-spacing: 0.05em; color: var(--text-muted); margin-bottom: 3px; }}
  .kpi-val {{ font-size: 18px; font-weight: 700; color: #fff; }}
  .kpi-sub {{ font-size: 11px; color: var(--text-muted); margin-top: 2px; }}

  .info-card {{ background: rgba(9, 13, 22, 0.6); border: 1px solid var(--border); border-radius: 8px; padding: 12px; font-size: 12px; line-height: 1.5; color: #cbd5e1; }}
  .info-card strong {{ color: #fff; }}

  #canvas-container {{ flex: 1; position: relative; background: #070a12; cursor: grab; }}
  canvas {{ width: 100%; height: 100%; display: block; }}
  #playback-bar {{ background: var(--card-bg); border-top: 1px solid var(--border); padding: 12px 20px; display: flex; align-items: center; gap: 14px; z-index: 10; }}
  button {{ background: #2563eb; color: white; border: none; padding: 8px 16px; border-radius: 6px; font-weight: 600; cursor: pointer; transition: background 0.15s; font-size: 13px; }}
  button:hover {{ background: #1d4ed8; }}
  .speed-btn {{ background: #1e293b; padding: 6px 12px; border: 1px solid var(--border); }}
  .speed-btn.active {{ background: #0284c7; border-color: #38bdf8; }}
  #slider {{ flex: 1; accent-color: #38bdf8; cursor: pointer; }}
  .time-display {{ font-family: monospace; font-size: 14px; font-weight: 700; min-width: 90px; text-align: center; }}
  
  .map-legend {{ position: absolute; top: 16px; right: 16px; background: rgba(19, 27, 46, 0.94); border: 1px solid var(--border); border-radius: 8px; padding: 12px 16px; font-size: 12px; display: flex; flex-direction: column; gap: 8px; pointer-events: none; }}
  .legend-item {{ display: flex; align-items: center; gap: 8px; }}
  .dot {{ width: 12px; height: 12px; border-radius: 3px; }}
</style>
</head>
<body>

<div id="sidebar">
  <header>
    <h1>FieldRoute</h1>
    <span class="tag">AUTONOMOUS</span>
  </header>

  <div class="gauge-box">
    <div class="gauge-label-row">
      <span>TRACTOR TANK (STATE OF CHARGE)</span>
      <span id="fuel-text">{tr.tank:.1f} / {tr.tank:.1f} L (100%)</span>
    </div>
    <div class="gauge-bar">
      <div id="fuel-fill" class="gauge-fill" style="width: 100%;"></div>
    </div>
    
    <div id="alert-banner"></div>
    <div id="status-badge" class="status-badge">READY TO START</div>
  </div>

  <div class="kpi-grid">
    <div class="kpi-card">
      <div class="kpi-label">Field Coverage</div>
      <div class="kpi-val">{metrics['coverage_pct']:.1f}%</div>
      <div class="kpi-sub">{metrics['covered_area_ha']:.2f} ha worked</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Implement Overlap</div>
      <div class="kpi-val">{metrics['overlap_pct']:.2f}%</div>
      <div class="kpi-sub">{metrics['overlap_area_m2']:,.0f} m² safe seam</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Total Distance</div>
      <div class="kpi-val">{metrics['total_distance_km']:.1f} km</div>
      <div class="kpi-sub">{metrics['work_distance_km']:.1f} km working</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Route Efficiency</div>
      <div class="kpi-val">{metrics['efficiency_pct']:.1f}%</div>
      <div class="kpi-sub">{metrics['travel_distance_km']:.1f} km deadhead</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Fuel Burned</div>
      <div class="kpi-val">{metrics['fuel_used_l']:.1f} L</div>
      <div class="kpi-sub">{metrics['fuel_per_ha']:.1f} L / ha</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Refuel Stops</div>
      <div class="kpi-val">{metrics['refuels_count']}</div>
      <div class="kpi-sub">5m automated swap</div>
    </div>
  </div>

  <div class="info-card">
    <strong>Autonomous Safety Features Active:</strong><br>
    • <strong>Obstacle Avoidance:</strong> Transit lines automatically skirt around the pond perimeter.<br>
    • <strong>Teardrop Turns (Ω):</strong> Smooth curvature turns at row ends avoid tight pivot tire scrub.<br>
    • <strong>Reserve Threshold:</strong> Detours to base station when remaining tank &lt; return distance × 1.15.
  </div>
</div>

<div id="main">
  <div id="canvas-container">
    <canvas id="fieldCanvas"></canvas>
    <div class="map-legend">
      <div class="legend-item"><div class="dot" style="background: #15803d; border: 1px solid #4ade80;"></div> Field Boundary ({metrics['field_area_ha']:.1f} ha)</div>
      <div class="legend-item"><div class="dot" style="background: #1e3a8a; border: 1px solid #38bdf8;"></div> Obstacle (Pond - Protected)</div>
      <div class="legend-item"><div class="dot" style="background: #0284c7;"></div> Working Pass ({tr.width:.1f}m swath)</div>
      <div class="legend-item"><div class="dot" style="background: #06b6d4;"></div> Teardrop Headland Turn</div>
      <div class="legend-item"><div class="dot" style="background: #ef4444; border: 1px dashed #f87171;"></div> Low-Fuel Return Route</div>
      <div class="legend-item"><div class="dot" style="background: #94a3b8; border: 1px dashed #cbd5e1;"></div> Safe Transit Route</div>
      <div class="legend-item"><div class="dot" style="background: #22c55e;"></div> Base Station / Dock</div>
      <div class="legend-item"><div class="dot" style="background: #dc2626; border-radius: 50%;"></div> Refuel Event Badge</div>
    </div>
  </div>

  <div id="playback-bar">
    <button id="playBtn">Play</button>
    <button id="resetBtn" style="background: #334155;">Reset</button>
    <button class="speed-btn" data-speed="1">1x</button>
    <button class="speed-btn active" data-speed="5">5x</button>
    <button class="speed-btn" data-speed="20">20x</button>
    <button class="speed-btn" data-speed="60">60x</button>
    <input type="range" id="slider" min="0" max="{int(res['time'])}" value="0">
    <span class="time-display" id="timeDisplay">00:00:00</span>
  </div>
</div>

<script>
const DATA = {json.dumps(payload)};

const canvas = document.getElementById('fieldCanvas');
const ctx = canvas.getContext('2d');
const container = document.getElementById('canvas-container');

let width, height;
function resize() {{
  width = container.clientWidth;
  height = container.clientHeight;
  canvas.width = width * window.devicePixelRatio;
  canvas.height = height * window.devicePixelRatio;
  ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
  draw();
}}
window.addEventListener('resize', resize);

const totalMissionTime = DATA.metrics.mission_time_s;
const segments = DATA.segments;

// Bounding box for auto-scale
let allX = DATA.field.map(p => p[0]).concat(DATA.dock_samples.map(d => d.pos[0]));
let allY = DATA.field.map(p => p[1]).concat(DATA.dock_samples.map(d => d.pos[1]));
let minX = Math.min(...allX) - 40, maxX = Math.max(...allX) + 40;
let minY = Math.min(...allY) - 40, maxY = Math.max(...allY) + 40;

function toScreen(p) {{
  let scale = Math.min((width - 90) / (maxX - minX), (height - 90) / (maxY - minY));
  let cx = (minX + maxX) / 2;
  let cy = (minY + maxY) / 2;
  return [
    width / 2 + (p[0] - cx) * scale,
    height / 2 - (p[1] - cy) * scale
  ];
}}

let currT = 0;
let isPlaying = false;
let speed = 5;
let lastAnimTime = null;

function getStateAt(t) {{
  if (!segments || segments.length === 0) {{
    return {{ pos: [0, 0], fuel: DATA.tank_capacity, status: "READY", segIndex: 0, isWarn: false, isRefuel: false }};
  }}

  // Binary search or linear scan for current segment
  let low = 0, high = segments.length - 1;
  let idx = 0;
  while (low <= high) {{
    let mid = Math.floor((low + high) / 2);
    if (t < segments[mid].t0) {{
      high = mid - 1;
    }} else if (t > segments[mid].t1) {{
      low = mid + 1;
      idx = mid;
    }} else {{
      idx = mid;
      break;
    }}
  }}
  idx = Math.max(0, Math.min(segments.length - 1, idx));
  let s = segments[idx];

  let dur = Math.max(0.0001, s.t1 - s.t0);
  let ratio = Math.max(0.0, Math.min(1.0, (t - s.t0) / dur));

  let pos = [
    s.p0[0] + (s.p1[0] - s.p0[0]) * ratio,
    s.p0[1] + (s.p1[1] - s.p0[1]) * ratio
  ];

  let fuel = s.f0 + (s.f1 - s.f0) * ratio;
  let isRefuel = (s.type === 'REFUEL');
  let isWarn = s.warn || (fuel <= DATA.tank_capacity * 0.20);

  return {{
    pos: pos,
    fuel: Math.max(0.0, Math.min(DATA.tank_capacity, fuel)),
    status: s.label,
    segIndex: idx,
    isWarn: isWarn,
    isRefuel: isRefuel,
    segType: s.type
  }};
}}

function formatTime(s) {{
  let h = Math.floor(s / 3600);
  let m = Math.floor((s % 3600) / 60);
  let sec = Math.floor(s % 60);
  return `${{String(h).padStart(2,'0')}}:${{String(m).padStart(2,'0')}}:${{String(sec).padStart(2,'0')}}`;
}}

function draw() {{
  ctx.clearRect(0, 0, width, height);

  // 1. Field Polygon
  ctx.beginPath();
  let sp0 = toScreen(DATA.field[0]);
  ctx.moveTo(sp0[0], sp0[1]);
  for (let i = 1; i < DATA.field.length; i++) {{
    let sp = toScreen(DATA.field[i]);
    ctx.lineTo(sp[0], sp[1]);
  }}
  ctx.closePath();
  ctx.fillStyle = '#064e3b';
  ctx.fill();
  ctx.strokeStyle = '#22c55e';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  // 2. Obstacles (Pond)
  for (let obs of DATA.obstacles) {{
    ctx.beginPath();
    let op0 = toScreen(obs[0]);
    ctx.moveTo(op0[0], op0[1]);
    for (let j = 1; j < obs.length; j++) {{
      let op = toScreen(obs[j]);
      ctx.lineTo(op[0], op[1]);
    }}
    ctx.closePath();
    ctx.fillStyle = '#1e3a8a';
    ctx.fill();
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    ctx.stroke();
  }}

  // 3. Render Trail Segments up to currT
  let st = getStateAt(currT);

  for (let i = 0; i < segments.length; i++) {{
    let s = segments[i];
    let a = toScreen(s.p0);
    let b = toScreen(s.p1);

    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);

    let isPast = (i <= st.segIndex);

    if (s.working) {{
      ctx.strokeStyle = isPast ? '#0284c7' : 'rgba(2, 132, 199, 0.18)';
      ctx.lineWidth = isPast ? 2.6 : 1.2;
      ctx.setLineDash([]);
    }} else if (s.type === 'TURN_HEADLAND') {{
      ctx.strokeStyle = isPast ? '#06b6d4' : 'rgba(6, 182, 212, 0.2)';
      ctx.lineWidth = isPast ? 2.0 : 1.0;
      ctx.setLineDash([]);
    }} else if (s.type === 'TRANSIT_DOCK') {{
      ctx.strokeStyle = isPast ? '#ef4444' : 'rgba(239, 68, 68, 0.2)';
      ctx.lineWidth = isPast ? 2.0 : 1.0;
      ctx.setLineDash([4, 4]);
    }} else {{
      ctx.strokeStyle = isPast ? '#94a3b8' : 'rgba(148, 163, 184, 0.15)';
      ctx.lineWidth = isPast ? 1.4 : 0.8;
      ctx.setLineDash([3, 3]);
    }}
    ctx.stroke();
  }}
  ctx.setLineDash([]);

  // 4. Base Station Dock
  let dockPt = toScreen(DATA.dock_samples[0].pos);
  ctx.fillStyle = '#22c55e';
  ctx.fillRect(dockPt[0] - 8, dockPt[1] - 8, 16, 16);
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.8;
  ctx.strokeRect(dockPt[0] - 8, dockPt[1] - 8, 16, 16);

  // 5. Refuel Stops Badges
  for (let r of DATA.refuels) {{
    let rp = toScreen(r.pos);
    ctx.fillStyle = '#dc2626';
    ctx.beginPath();
    ctx.arc(rp[0], rp[1], 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('R' + r.index, rp[0] + 10, rp[1] + 4);
  }}

  // 6. Tractor Vehicle Indicator
  let tp = toScreen(st.pos);

  // Pulsing Low-Fuel Beacon Halo
  if (st.isWarn) {{
    let pulseR = 12 + (Date.now() % 1000) / 1000 * 14;
    ctx.beginPath();
    ctx.arc(tp[0], tp[1], pulseR, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(239, 68, 68, 0.3)';
    ctx.fill();
  }} else if (st.isRefuel) {{
    let pulseG = 12 + (Date.now() % 1000) / 1000 * 14;
    ctx.beginPath();
    ctx.arc(tp[0], tp[1], pulseG, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(34, 197, 94, 0.35)';
    ctx.fill();
  }}

  ctx.shadowColor = st.isWarn ? '#ef4444' : '#38bdf8';
  ctx.shadowBlur = 10;
  ctx.fillStyle = st.isWarn ? '#ef4444' : '#f59e0b';
  ctx.beginPath();
  ctx.arc(tp[0], tp[1], 7.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.shadowBlur = 0;

  // 7. Update UI HUD
  document.getElementById('slider').value = currT;
  document.getElementById('timeDisplay').innerText = formatTime(currT);

  let fuelPct = Math.max(0, Math.min(100, (st.fuel / DATA.tank_capacity) * 100));
  let fuelText = document.getElementById('fuel-text');
  fuelText.innerText = `${{st.fuel.toFixed(1)}} / ${{DATA.tank_capacity.toFixed(1)}} L (${{fuelPct.toFixed(0)}}%)`;

  let fuelFill = document.getElementById('fuel-fill');
  fuelFill.style.width = fuelPct + '%';

  // Dynamic Fuel Fill Color
  if (fuelPct > 45) {{
    fuelFill.style.background = '#22c55e';
  }} else if (fuelPct > 20) {{
    fuelFill.style.background = '#eab308';
  }} else {{
    fuelFill.style.background = '#ef4444';
  }}

  let alertBanner = document.getElementById('alert-banner');
  if (st.isRefuel) {{
    alertBanner.className = 'refuel';
    alertBanner.innerHTML = `⚡ <strong>REFUELLING DOCK ACTIVE:</strong> Automated 5-minute swap cycle in progress (+${{(DATA.tank_capacity - st.fuel).toFixed(1)}} L).`;
  }} else if (st.isWarn) {{
    alertBanner.className = 'warning';
    alertBanner.innerHTML = `⚠️ <strong>LOW FUEL WARNING (${{fuelPct.toFixed(0)}}%):</strong> Remaining energy below safe row reserve. Autonomous detour to dock engaged.`;
  }} else {{
    alertBanner.className = '';
  }}

  let badge = document.getElementById('status-badge');
  badge.innerText = st.status;
  if (st.isRefuel) badge.style.background = '#15803d';
  else if (st.isWarn) badge.style.background = '#dc2626';
  else if (st.segType === 'WORK') badge.style.background = '#0284c7';
  else if (st.segType === 'TURN_HEADLAND') badge.style.background = '#0891b2';
  else badge.style.background = '#475569';
}}

function step(timestamp) {{
  if (!lastAnimTime) lastAnimTime = timestamp;
  let dt = (timestamp - lastAnimTime) / 1000;
  lastAnimTime = timestamp;

  if (isPlaying) {{
    currT += dt * speed;
    if (currT >= totalMissionTime) {{
      currT = totalMissionTime;
      isPlaying = false;
      document.getElementById('playBtn').innerText = 'Play';
    }}
    draw();
  }} else if (getStateAt(currT).isWarn || getStateAt(currT).isRefuel) {{
    // Keep pulsing beacon animation active when paused on alert
    draw();
  }}
  requestAnimationFrame(step);
}}

document.getElementById('playBtn').addEventListener('click', () => {{
  isPlaying = !isPlaying;
  document.getElementById('playBtn').innerText = isPlaying ? 'Pause' : 'Play';
  lastAnimTime = null;
}});

document.getElementById('resetBtn').addEventListener('click', () => {{
  currT = 0;
  isPlaying = false;
  document.getElementById('playBtn').innerText = 'Play';
  draw();
}});

document.getElementById('slider').addEventListener('input', (e) => {{
  currT = parseFloat(e.target.value);
  draw();
}});

document.querySelectorAll('.speed-btn').forEach(btn => {{
  btn.addEventListener('click', () => {{
    document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    speed = parseFloat(btn.dataset.speed);
  }});
}});

resize();
requestAnimationFrame(step);
</script>
</body>
</html>"""

    with open(path, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"Interactive telemetry dashboard saved to: {path}")
