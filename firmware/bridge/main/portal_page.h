#ifndef BRIDGE_PORTAL_PAGE_H
#define BRIDGE_PORTAL_PAGE_H

// The setup portal's page: one hand-written HTML file with plain JavaScript,
// no framework and no external resources (the phone has no internet on the
// bridge's network). It polls /api/state every 250 ms, which is also the
// portal heartbeat: a hidden or closed page stops polling, and the robot
// sleeps. It shows only what the robot reports; edits are requests.

static const char PORTAL_PAGE[] = R"PAGE(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Spooder setup</title>
<style>
body{font:15px system-ui,sans-serif;margin:0;padding:8px;background:#111;color:#eee}
h1{font-size:18px;margin:4px 0}h2{font-size:15px;margin:12px 0 4px}
.row{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:4px 0}
button{font:inherit;padding:8px 10px;border:0;border-radius:6px;background:#333;color:#eee;min-width:44px}
button:disabled{opacity:.4}
.sel{background:#2563eb}.ok{color:#4ade80}.warn{color:#fb923c}.bad{color:#f87171}
#estop{display:none;background:#b91c1c;color:#fff;font-weight:bold;font-size:20px;text-align:center;padding:10px;border-radius:6px}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:3px 4px;text-align:right}
tr.cur{background:#1e3a8a}tr{cursor:pointer}td:first-child,th:first-child{text-align:left}
.v{display:inline-block;min-width:76px;font-weight:bold}.lbl{display:inline-block;min-width:64px}
textarea{width:100%;height:200px;font:12px monospace;background:#000;color:#eee}
#msg{min-height:1.2em}
</style></head><body>
<h1>Spooder setup</h1>
<div class="row"><span id="bridge">Bridge: ...</span><span id="link"></span><span id="src"></span></div>
<div id="estop">EMERGENCY STOP</div>
<div class="row"><span id="body"></span><span id="power"></span></div>
<div class="row"><span id="faults"></span></div>
<h2>Mode</h2>
<div class="row">
<button onclick="mode('stand')">Stand</button><button onclick="mode('calibrate')">Calibrate</button>
<button onclick="mode('sleep')">Sleep</button><button onclick="mode('clear')">Clear faults</button>
<span id="req"></span></div>
<h2>Joint</h2>
<div id="hint" class="warn"></div>
<div class="row" id="legs"></div>
<div class="row" id="joints"></div>
<div id="editor">
<div class="row"><span class="lbl">Offset</span><span class="v" id="e-offset"></span></div>
<div class="row" id="n-offset"></div>
<div class="row"><span class="lbl">0&deg;</span><span class="v" id="e-p0"></span></div>
<div class="row" id="n-p0"></div>
<div class="row"><span class="lbl">180&deg;</span><span class="v" id="e-p180"></span></div>
<div class="row" id="n-p180"></div>
<div class="row"><span class="lbl">Direction</span><span class="v" id="e-rev"></span>
<button id="b-rev" onclick="change('rev',1)">Reverse</button></div>
<div class="row"><span id="pend"></span><span id="res"></span></div>
</div>
<div id="msg" class="bad"></div>
<h2>Table</h2>
<table><thead><tr><th>Joint</th><th>Offset</th><th>0&deg;</th><th>180&deg;</th><th>Dir</th><th></th></tr></thead>
<tbody id="tbl"></tbody></table>
<h2>Export</h2>
<div class="row"><button onclick="exportTable()">Export</button><button onclick="copyExport()">Copy</button>
<span id="copied"></span></div>
<textarea id="out" readonly placeholder="calibration_defaults.cpp"></textarea>
<h2>Leave setup mode</h2>
<div class="row"><button onclick="leave()">Exit to normal operation</button></div>
<script>
var MODES=['Initialise','Stand','Walk','Sleep','Calibrate'];
var JOINTS=['coxa','femur','tibia'];
var FAULTS=['overcurrent','overload','low V','brown-out','high V','e-stop'];
var RESULTS=['accepted','clamped','rejected','none yet'];
var leg=0,joint=0,state=null,polling=true,timer=null;
function $(id){return document.getElementById(id)}
function txt(id,t,c){var e=$(id);e.textContent=t;e.className=c||''}
function cur(){return leg*3+joint}
function deg(c){return (c/100).toFixed(2)+'°'}
function btn(parent,label,fn){var b=document.createElement('button');b.textContent=label;b.onclick=fn;parent.appendChild(b);return b}
function build(){
  for(var l=0;l<6;l++)(function(l){btn($('legs'),'Leg '+(l+1),function(){leg=l;render()})})(l);
  for(var j=0;j<3;j++)(function(j){btn($('joints'),JOINTS[j],function(){joint=j;render()})})(j);
  [-500,-100,-50,50,100,500].forEach(function(d){btn($('n-offset'),(d>0?'+':'')+d/100+'°',function(){change('offset',d)})});
  ['p0','p180'].forEach(function(f){[-50,-10,10,50].forEach(function(d){btn($('n-'+f),(d>0?'+':'')+d+' µs',function(){change(f,d)})})});
  for(var i=0;i<18;i++)(function(i){var tr=document.createElement('tr');tr.onclick=function(){leg=Math.floor(i/3);joint=i%3;render()};
    for(var k=0;k<6;k++)tr.appendChild(document.createElement('td'));$('tbl').appendChild(tr)})(i);
}
function post(url,params){
  var body=new URLSearchParams(params);
  return fetch(url,{method:'POST',body:body}).then(function(r){return r.text().then(function(t){
    if(!r.ok)throw new Error(t||('HTTP '+r.status));return t})});
}
function report(e){txt('msg',e?String(e.message||e):'','bad')}
function mode(m){post('/api/mode',{m:m}).then(function(){report()},report)}
function change(f,d){post('/api/joint',{j:cur(),f:f,v:d,rel:1}).then(function(){report()},report)}
function render(){
  var s=state;
  [].forEach.call($('legs').children,function(b,i){b.className=i==leg?'sel':''});
  [].forEach.call($('joints').children,function(b,i){b.className=i==joint?'sel':''});
  if(!s)return;
  txt('bridge','Bridge: ok','ok');
  txt('link',s.link?'Robot link: ok':'Robot link: LOST',s.link?'ok':'bad');
  var t=s.tel,fl=t.flags;
  txt('src',!s.link?'':(fl&1)?'Source CONFLICT with USB':(fl&2)?'USB in control':(fl&4)?'Portal in control':'',
      (fl&1)?'bad':(fl&2)?'warn':'ok');
  $('estop').style.display=(s.estop||(s.link&&(t.faults&32)))?'block':'none';
  txt('body',s.link?'Body '+(MODES[t.mode]||'?'):'Body --');
  txt('power',s.link?('Out '+(t.out?'on':'off')+'  '+(t.mv/1000).toFixed(2)+' V  '+(t.ma/1000).toFixed(2)+' A'):'');
  var names=[];for(var b=0;b<16;b++)if(t.faults&(1<<b))names.push(FAULTS[b]||('0x'+(1<<b).toString(16)));
  txt('faults',!s.link?'':names.length?'Faults: '+names.join(', '):'Faults: none',names.length?'bad':'ok');
  txt('req','Requesting '+s.req+(s.portal?'':' (not connected)'));
  var c=s.cal,calib=s.link&&t.mode==4;
  txt('hint',!s.link?'Robot link lost.':!c?'No calibration table yet: press Calibrate (the robot reports it in Calibrate).':
      calib?'':'Changes are rejected outside Calibrate.','warn');
  var j=c?c.j[cur()]:null;
  txt('e-offset',j?deg(j[0]):'--');txt('e-p0',j?j[1]+' µs':'--');txt('e-p180',j?j[2]+' µs':'--');
  txt('e-rev',j?(j[3]?'reversed':'normal'):'--');
  [].forEach.call(document.querySelectorAll('#editor button'),function(b){b.disabled=!c});
  txt('pend',j&&j[4]?'Waiting for the robot...':'',j&&j[4]?'warn':'');
  txt('res',c&&c.res<3?'Last change: '+RESULTS[c.res]:'',c&&c.res==2?'bad':c&&c.res==1?'warn':'ok');
  var rows=$('tbl').children;
  for(var i=0;i<18;i++){var r=rows[i],v=c?c.j[i]:null,d=r.children;r.className=i==cur()?'cur':'';
    d[0].textContent=(Math.floor(i/3)+1)+' '+JOINTS[i%3];
    d[1].textContent=v?deg(v[0]):'--';d[2].textContent=v?v[1]:'--';d[3].textContent=v?v[2]:'--';
    d[4].textContent=v?(v[3]?'rev':'norm'):'--';d[5].textContent=v&&v[4]?'*':''}
}
function poll(){
  timer=null;if(!polling||document.hidden)return;
  var ctl=new AbortController(),to=setTimeout(function(){ctl.abort()},1000);
  fetch('/api/state',{signal:ctl.signal,cache:'no-store'}).then(function(r){return r.json()}).then(function(s){
    state=s;render()},function(){state=null;txt('bridge','Bridge: LOST','bad');txt('link','')})
  .then(function(){clearTimeout(to);if(polling&&!timer)timer=setTimeout(poll,250)});
}
document.addEventListener('visibilitychange',function(){if(!document.hidden&&!timer)poll()});
function exportTable(){
  fetch('/api/export',{cache:'no-store'}).then(function(r){return r.text().then(function(t){
    if(!r.ok)throw new Error(t);$('out').value=t;report()})}).catch(report);
}
function copyExport(){
  var o=$('out');if(!o.value)return;
  function done(){txt('copied','Copied','ok')}
  function fallback(){o.focus();o.select();try{document.execCommand('copy')?done():txt('copied','Select and copy by hand','warn')}
    catch(e){txt('copied','Select and copy by hand','warn')}}
  if(navigator.clipboard&&window.isSecureContext)navigator.clipboard.writeText(o.value).then(done,fallback);else fallback();
}
function leave(){
  if(!confirm('Restart the bridge into normal operation? The robot sleeps.'))return;
  polling=false;post('/api/exit',{}).then(function(){txt('bridge','Bridge restarting into normal operation','warn')},report);
}
build();render();poll();
</script></body></html>
)PAGE";

// Kept small: it is sent whole on every page load.
static_assert(sizeof(PORTAL_PAGE) < 16 * 1024, "the portal page must stay under 16 KB");

#endif // BRIDGE_PORTAL_PAGE_H
