"""Offline artifact rendering. Ground truth never feeds back into the agent."""
from __future__ import annotations

import html
import json
from pathlib import Path


def render_svg(result: dict) -> str:
    truth, known = result["offline_ground_truth"]["rows"], result["final_known_map"]
    width, height, scale = len(truth[0]), len(truth), 24
    panel_width, panel_height = max(width * scale, 360), height * scale
    gap, top = 48, 62
    canvas_width, canvas_height = panel_width * 2 + gap + 40, panel_height + 120
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_width}" height="{canvas_height}" viewBox="0 0 {canvas_width} {canvas_height}">',
             '<rect width="100%" height="100%" fill="#f3f6fa"/>',
             '<style>text{font-family:Arial,sans-serif;fill:#1e293b} .grid{stroke:#cbd5e1;stroke-width:.5}</style>']
    colors = {"#": "#334155", ".": "#ffffff", "?": "#a5b4c4", "S": "#ffffff", "T": "#ffffff", " ": "#ffffff"}
    for index, rows in enumerate((known, truth)):
        ox = 20 + index * (panel_width + gap)
        label = "Agent observations (unknown = gray)" if index == 0 else "OFFLINE ground truth (evaluation only)"
        parts.append(f'<text x="{ox}" y="24" font-size="14" font-weight="bold">{label}</text>')
        for y, row in enumerate(rows):
            for x, value in enumerate(row):
                parts.append(f'<rect class="grid" x="{ox+x*scale}" y="{top+y*scale}" width="{scale}" height="{scale}" fill="{colors[value]}"/>')
        path = " ".join(f"{ox+(p[0]+.5)*scale},{top+(p[1]+.5)*scale}" for p in result["trajectory"])
        parts.append(f'<polyline points="{path}" fill="none" stroke="#2563eb" stroke-width="3" stroke-linejoin="round" opacity=".65"/>')
        targets = result["offline_ground_truth"]["targets"] if index else result["target_annotations"]
        markers = [(result["start"], "S", "#059669", "start")]
        markers += [(t["grid_xy"], "T", "#f97316", t["id"]) for t in targets]
        for p, label, color, item_id in markers:
            if p is not None:
                cx, cy = ox+(p[0]+.5)*scale, top+(p[1]+.5)*scale
                parts.append(f'<g><title>{html.escape(item_id)}: {p}</title><circle cx="{cx}" cy="{cy}" r="9" fill="{color}"/><text x="{cx}" y="{cy+4}" text-anchor="middle" style="fill:white" font-size="12">{label}</text></g>')
    label = html.escape(f"{result['metrics']['status']} | moves: {result['metrics']['steps']} | reachable free coverage: {result['metrics']['reachable_free_coverage']:.1%}")
    parts.append(f'<text x="20" y="{canvas_height-24}" font-size="14">{label}</text></svg>')
    return "".join(parts)


def write_artifacts(result: dict, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    json_path, svg_path, html_path = output / "run.json", output / "overview.svg", output / "report.html"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    maze_path = output / "maze.txt"
    maze_path.write_text("\n".join(result["offline_ground_truth"]["rows"]) + "\n", encoding="utf-8")
    svg = render_svg(result)
    svg_path.write_text(svg, encoding="utf-8")
    # JSON values cannot terminate this script element, including custom names.
    payload = json.dumps(result, ensure_ascii=False).replace("<", "\\u003c")
    rows = "".join(f"<tr><th>{html.escape(k)}</th><td>{html.escape(str(v))}</td></tr>" for k, v in result["metrics"].items())
    targets_table = "".join(f'<tr><td>{html.escape(t["id"])}</td><td>{t["grid_xy"]}</td><td>{t["first_detection_step"]}</td></tr>' for t in result["target_annotations"])
    if not targets_table:
        targets_table = '<tr><td colspan="3">未观测到目标；不据此推断隔离区域中不存在目标。</td></tr>'
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LIMO 未知迷宫：任务逻辑仿真</title><style>
body{margin:0;background:#f1f5f9;color:#1e293b;font:16px/1.6 system-ui,sans-serif}main{max-width:1180px;margin:32px auto;padding:0 24px}h1{font-size:28px}section{background:white;border-radius:12px;padding:20px;margin:20px 0;overflow:auto}canvas{max-width:100%;height:auto;border:1px solid #cbd5e1}table{border-collapse:collapse}th,td{text-align:left;border-bottom:1px solid #e2e8f0;padding:6px 16px 6px 0}th{font-weight:500}input{width:65%}button{padding:6px 16px}small{color:#475569}.overview svg{max-width:100%;height:auto}</style>
<main><h1>LIMO 未知迷宫：任务逻辑仿真</h1><p>二维理想格网实验：目标位置和数量对代理未知，完整自主探索、标注所有已发现目标、返航，不拿取。已知精确位姿、格网边界、静态障碍；局部有限量程与遮挡观测；只在已观测空闲格上规划。这里没有执行 SLAM、Gazebo、相机识别、动力学或实车验证。</p>
<section><h2>按步回放代理观测</h2><p>灰色：未知；深色：墙；浅蓝：已进行目标搜索的空闲区域；绿色 S：起点；蓝点：当前位置；橙色 T：已检测目标。</p><button id="play">播放</button> <input id="slider" type="range" min="0" value="0"><output id="step"></output><p id="state"></p><canvas id="map"></canvas><p><small>回放只显示当时获得的观测。下方真值图仅用于事后评估。</small></p></section>
<section class="overview"><h2>最终地图与离线真值对照</h2>__SVG__</section>
<section><h2>已发现目标标注</h2><p>格网坐标 [x,y]，单位为格，左上角为原点；目标 ID 仅在观测到时获得，不向代理提供目标总数。</p><table><tr><th>观测 ID</th><th>地图坐标</th><th>首次发现步数</th></tr>__TARGETS__</table></section>
<section><h2>指标</h2><p>search_complete_reachable_region 表示起点连通区域内的地图探索与理想目标搜索均已完成；它不证明隔离区域中没有目标。reachable_free_coverage 是离线几何覆盖率，searched_reachable_free_coverage 是离线目标搜索覆盖率。超时返航记录为搜索未完成，不算完整任务成功。</p><table>__ROWS__</table></section>
<p><a href="run.json">完整 JSON 轨迹与观测</a> · <a href="overview.svg">导出 SVG</a></p></main>
<script type="application/json" id="data">__PAYLOAD__</script><script>
const r=JSON.parse(document.querySelector('#data').textContent),frames=r.frames,slider=document.querySelector('#slider');
const canvas=document.querySelector('#map'),ctx=canvas.getContext('2d'),w=r.final_known_map[0].length,h=r.final_known_map.length,s=26;
canvas.width=w*s;canvas.height=h*s;slider.max=frames.length-1;let timer=null;
function draw(index){const grid=Array.from({length:h},()=>Array(w).fill('?')),searched=new Set();let detected=[];
for(let i=0;i<=index;i++){for(const [x,y,v] of frames[i].new_cells)grid[y][x]=v;for(const p of frames[i].new_searched_cells)searched.add(p.join(','));detected=frames[i].detected_targets;}
for(let y=0;y<h;y++)for(let x=0;x<w;x++){ctx.fillStyle=grid[y][x]=='#'?'#334155':grid[y][x]=='?'?'#a5b4c4':searched.has(x+','+y)?'#dbeafe':'#fff';ctx.fillRect(x*s,y*s,s,s);ctx.strokeStyle='#cbd5e1';ctx.strokeRect(x*s,y*s,s,s);}
ctx.strokeStyle='#2563eb';ctx.lineWidth=2;ctx.beginPath();for(let i=0;i<=index;i++){const [x,y]=frames[i].position;i?ctx.lineTo((x+.5)*s,(y+.5)*s):ctx.moveTo((x+.5)*s,(y+.5)*s)}ctx.stroke();
function marker(p,color,label){ctx.fillStyle=color;ctx.beginPath();ctx.arc((p[0]+.5)*s,(p[1]+.5)*s,9,0,Math.PI*2);ctx.fill();ctx.fillStyle='white';ctx.textAlign='center';ctx.font='12px sans-serif';ctx.fillText(label,(p[0]+.5)*s,(p[1]+.5)*s+4)}
marker(r.start,'#059669','S');for(const t of detected)marker(t.grid_xy,'#f97316','T');marker(frames[index].position,'#2563eb','R');
document.querySelector('#step').textContent=` ${index} / ${frames.length-1}`;document.querySelector('#state').textContent=`阶段：${frames[index].phase}；已知格数：${frames[index].known_cells}；已发现目标数：${detected.length}（总数未知）；返航原因：${frames[index].return_reason||'—'}`;}
slider.oninput=()=>draw(+slider.value);document.querySelector('#play').onclick=()=>{if(timer){clearInterval(timer);timer=null;document.querySelector('#play').textContent='播放';return}if(+slider.value===frames.length-1)slider.value=0;document.querySelector('#play').textContent='暂停';timer=setInterval(()=>{slider.value=Math.min(+slider.value+1,frames.length-1);draw(+slider.value);if(+slider.value===frames.length-1){clearInterval(timer);timer=null;document.querySelector('#play').textContent='播放'}},120)};draw(0);
</script></html>'''
    html_path.write_text(page.replace("__SVG__", svg).replace("__ROWS__", rows).replace("__TARGETS__", targets_table).replace("__PAYLOAD__", payload), encoding="utf-8")
    return {"json": str(json_path.resolve()), "svg": str(svg_path.resolve()), "html": str(html_path.resolve()),
            "maze": str(maze_path.resolve())}
