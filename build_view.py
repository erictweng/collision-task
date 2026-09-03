"""Build out/view.html: a self-contained review view. No external assets."""
import json, pathlib

DATA = json.load(open("out/view_data.json"))
HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Motion screening review</title>
<style>
:root{--bg:#faf9f7;--panel:#fff;--ink:#1c1b19;--dim:#6b6862;--line:#e3e0da;
--ok:#2f7d4f;--bad:#b3261e;--warn:#a4682a;--accent:#2f5d8a;color-scheme:light}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#16151a;--panel:#1e1d23;
--ink:#eceaf0;--dim:#9b97a3;--line:#33313b;--ok:#68c48d;--bad:#f2837a;--warn:#e0a463;--accent:#7fb0dd}}
:root[data-theme=dark]{--bg:#16151a;--panel:#1e1d23;--ink:#eceaf0;--dim:#9b97a3;--line:#33313b;
--ok:#68c48d;--bad:#f2837a;--warn:#e0a463;--accent:#7fb0dd}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{padding:18px 22px;border-bottom:1px solid var(--line)}
h1{margin:0 0 4px;font-size:17px;letter-spacing:-.01em}
.sub{color:var(--dim);font-size:13px}
.stats{display:flex;gap:22px;margin-top:12px;flex-wrap:wrap}
.stat b{display:block;font-size:19px;font-variant-numeric:tabular-nums}
.stat span{color:var(--dim);font-size:11.5px;text-transform:uppercase;letter-spacing:.06em}
main{display:grid;grid-template-columns:326px 1fr;gap:0;min-height:calc(100vh - 118px)}
#list{border-right:1px solid var(--line);overflow:auto;max-height:calc(100vh - 118px)}
#filters{display:flex;flex-wrap:wrap;gap:6px;padding:12px}
button.f{border:1px solid var(--line);background:var(--panel);color:var(--ink);
border-radius:999px;padding:4px 11px;font-size:12px;cursor:pointer}
button.f[aria-pressed=true]{background:var(--accent);border-color:var(--accent);color:#fff}
.row{padding:9px 12px;border-top:1px solid var(--line);cursor:pointer;display:flex;
justify-content:space-between;gap:8px;align-items:baseline}
.row:hover{background:var(--bg)}
.row[aria-selected=true]{background:var(--bg);box-shadow:inset 3px 0 0 var(--accent)}
.mid{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
.tag{color:var(--dim);font-size:11px}
.pill{font-size:10.5px;padding:1px 7px;border-radius:999px;border:1px solid currentColor;white-space:nowrap}
.fa{color:var(--bad)} .fr{color:var(--warn)} .agree{color:var(--dim)}
#detail{padding:18px 22px;overflow:auto;max-height:calc(100vh - 118px)}
.views{display:flex;gap:18px;flex-wrap:wrap}
figure{margin:0}figcaption{color:var(--dim);font-size:11.5px;margin-bottom:5px}
svg{background:var(--panel);border:1px solid var(--line);border-radius:8px;max-width:100%}
table{border-collapse:collapse;margin-top:16px;font-size:12.5px}
th,td{text-align:left;padding:4px 12px 4px 0;border-bottom:1px solid var(--line)}
th{color:var(--dim);font-weight:500;font-size:11px;text-transform:uppercase;letter-spacing:.05em}
td.num{font-variant-numeric:tabular-nums;text-align:right;padding-right:16px}
.reason{font-family:ui-monospace,Menlo,monospace;font-size:12px;color:var(--bad)}
.note{color:var(--dim);max-width:62ch;margin-top:14px;font-size:12.5px}
.empty{color:var(--dim);padding:40px 0}
</style></head><body>
<header>
<h1>Motion screening review</h1>
<div class="sub">Each row is one candidate motion. <b>Verdict</b> is what the screener decided from the
scene <i>estimate</i>; <b>outcome</b> is what happened when it ran in the true scene.
Disagreements are the point.</div>
<div class="stats" id="stats"></div>
</header>
<main>
<div id="list"><div id="filters"></div><div id="rows"></div></div>
<div id="detail"><div class="empty">Select a motion.</div></div>
</main>
<script id="payload" type="application/json">__DATA__</script>
<script>
const D=JSON.parse(document.getElementById('payload').textContent);
const M=D.motions, L=D.layouts;
const kind=m=> m.allow&&m.unsafe?'fa' : (!m.allow&&!m.unsafe?'fr' : 'agree');
const KL={fa:'false accept',fr:'false reject',agree:'agreed'};
const counts={fa:0,fr:0,agree:0};M.forEach(m=>counts[kind(m)]++);
document.getElementById('stats').innerHTML=[
 ['motions',M.length],['false accepts',counts.fa],['false rejects',counts.fr],
 ['agreed',counts.agree],['margin',D.config.margin_mm.toFixed(0)+' mm'],
 ['poses',D.config.n_poses]
].map(([k,v])=>`<div class="stat"><b>${v}</b><span>${k}</span></div>`).join('');

let filter='fa', sel=null;
const FILTERS=[['fa','False accepts'],['fr','False rejects'],['agree','Agreed'],['all','All']];
document.getElementById('filters').innerHTML=FILTERS.map(([k,l])=>
 `<button class="f" data-k="${k}" aria-pressed="${k===filter}">${l}</button>`).join('');
document.getElementById('filters').onclick=e=>{
 const b=e.target.closest('button');if(!b)return;filter=b.dataset.k;
 [...document.querySelectorAll('button.f')].forEach(x=>x.setAttribute('aria-pressed',x.dataset.k===filter));
 renderList();};

function renderList(){
 const rows=M.filter(m=>filter==='all'||kind(m)===filter);
 document.getElementById('rows').innerHTML=rows.length?rows.map(m=>`
  <div class="row" data-id="${m.mid}" aria-selected="${sel===m.mid}">
   <div><div class="mid">${m.mid}</div><div class="tag">${m.tag} &middot; ${m.layout}</div></div>
   <div class="pill ${kind(m)}">${KL[kind(m)]}</div></div>`).join('')
  :'<div class="empty" style="padding:24px 12px">Nothing in this category.</div>';
 document.querySelectorAll('.row').forEach(r=>r.onclick=()=>{sel=r.dataset.id;renderList();renderDetail();});
}

function proj(pts,ax,ay,box){
 const xs=pts.map(p=>p[ax]),ys=pts.map(p=>p[ay]);
 return {x0:Math.min(...xs),x1:Math.max(...xs),y0:Math.min(...ys),y1:Math.max(...ys)};
}
function svgView(m,ax,ay,w,h,label,flipY){
 const lay=L[m.layout];
 let pts=[];lay.parts.forEach(p=>{pts.push([p.lo[0],p.lo[1],p.lo[2]],[p.hi[0],p.hi[1],p.hi[2]]);});
 m.path.forEach(p=>pts.push(p));
 const b=proj(pts,ax,ay),pad=0.05;
 const X0=b.x0-pad,X1=b.x1+pad,Y0=b.y0-pad,Y1=b.y1+pad;
 const sx=v=>(v-X0)/(X1-X0)*(w-24)+12;
 const sy=v=>flipY?(h-12)-(v-Y0)/(Y1-Y0)*(h-24):(v-Y0)/(Y1-Y0)*(h-24)+12;
 let out=`<figure><figcaption>${label}</figcaption><svg viewBox="0 0 ${w} ${h}" width="${w}" height="${h}">`;
 lay.parts.forEach(p=>{
  const isT=p.owner===m.target,isD=p.owner===m.dest,isBlame=p.owner===m.blamed;
  const col=isT?'var(--ok)':isD?'var(--accent)':isBlame?'var(--bad)':'var(--dim)';
  const x=sx(Math.min(p.lo[ax],p.hi[ax])),X=sx(Math.max(p.lo[ax],p.hi[ax]));
  const y1=sy(p.lo[ay]),y2=sy(p.hi[ay]);
  out+=`<rect x="${x}" y="${Math.min(y1,y2)}" width="${Math.max(X-x,1.5)}" height="${Math.max(Math.abs(y2-y1),1.5)}"
   fill="${col}" fill-opacity="${isBlame?0.30:0.14}" stroke="${col}" stroke-width="1"/>`;});
 const d=m.path.map((p,i)=>`${i?'L':'M'}${sx(p[ax]).toFixed(1)},${sy(p[ay]).toFixed(1)}`).join('');
 out+=`<path d="${d}" fill="none" stroke="var(--ink)" stroke-width="1.6" stroke-opacity=".75"/>`;
 const s=m.path[0],e=m.path[m.path.length-1];
 out+=`<circle cx="${sx(s[ax])}" cy="${sy(s[ay])}" r="3.2" fill="var(--ink)"/>`;
 out+=`<circle cx="${sx(e[ax])}" cy="${sy(e[ay])}" r="3.2" fill="none" stroke="var(--ink)" stroke-width="1.6"/>`;
 return out+'</svg></figure>';
}

function renderDetail(){
 const m=M.find(x=>x.mid===sel);if(!m)return;
 const g=Object.entries(m.gaps).sort((a,b)=>a[1]-b[1]);
 const role=o=>o===m.target?'target':o===m.dest?'destination':o==='table'?'structure':'obstacle';
 document.getElementById('detail').innerHTML=`
  <div style="display:flex;gap:10px;align-items:baseline;flex-wrap:wrap">
   <div class="mid" style="font-size:15px">${m.mid}</div>
   <div class="pill ${kind(m)}">${KL[kind(m)]}</div>
   <div class="tag">${m.tag} &middot; grasp ${m.target} &rarr; deliver to ${m.dest}</div></div>
  <div class="note" style="margin-top:8px">
   Screener said <b>${m.allow?'permit':'reject'}</b>. Executed, it was
   <b>${m.unsafe?'unsafe':'safe'}</b>${m.unsafe?': <span class="reason">'+m.reasons.join(' &middot; ')+'</span>':''}.
   ${!m.allow&&m.blamed?'Rejected because of <b>'+m.blamed+'</b>.':''}</div>
  <div class="views" style="margin-top:14px">
   ${svgView(m,0,1,430,300,'Top down — x across, y up the page',false)}
   ${svgView(m,0,2,430,220,'Side on — x across, height up',true)}
  </div>
  <div class="note">Line is the tool centre point through the motion; filled dot is the start,
  hollow dot the end. Green is the object it means to grasp, blue the bin it means to reach into,
  red whatever the screener blamed.</div>
  <table><thead><tr><th>object</th><th>role</th><th class="num">closest gap</th></tr></thead><tbody>
  ${g.map(([o,v])=>`<tr><td>${o}</td><td class="tag">${role(o)}</td>
   <td class="num" style="color:${v<0?'var(--bad)':'inherit'}">${(v*1000).toFixed(1)} mm</td></tr>`).join('')}
  </tbody></table>
  <div class="note">Gap is proxy surface to object surface: negative means the sphere proxy overlaps.
  Overlap is expected for the target (fingers close around it) and for the destination
  (the gripper enters the bin) — that is what the per-object licence is for.</div>`;
}
renderList();
</script></body></html>
"""
out = pathlib.Path("out/view.html")
out.write_text(HTML.replace("__DATA__", json.dumps(DATA)))
print("wrote", out, out.stat().st_size // 1024, "KB")
