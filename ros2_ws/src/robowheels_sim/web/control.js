const $ = id => document.getElementById(id);
const remote = new URL(`http://${location.hostname}:16080/vnc.html`);
remote.search = new URLSearchParams({autoconnect:'1',resize:'scale',reconnect:'1',reconnect_delay:'1000',show_dot:'0'});
$('desktop').src=remote.href;$('fullscreen').href=remote.href;
let names=[];let initialized=false;
const labels={shoulder_roll:'肩部侧摆',shoulder_pitch:'肩部俯仰',elbow_pitch:'肘部俯仰',wrist_pitch:'腕部俯仰',wrist_roll:'腕部旋转'};
function build(joints){
  names=joints;
  for(const side of ['right','left']){
    const heading=document.createElement('div');heading.className='arm-label';heading.textContent=side==='right'?'右臂 / RIGHT':'左臂 / LEFT';$('joints').append(heading);
    for(const name of joints.filter(name=>name.startsWith(side+'_'))){
      const label=labels[name.slice(side.length+1)];const row=document.createElement('div');row.className='joint';
      row.innerHTML=`<div class="joint-head"><label for="${name}">${label}</label><span id="actual-${name}">反馈 0.000</span></div><div class="joint-row"><input type="range" min="-0.65" max="0.65" step="0.01" value="0" aria-label="${side==='right'?'右':'左'}${label}" id="${name}"><input type="number" min="-0.65" max="0.65" step="0.01" value="0" aria-label="${side==='right'?'右':'左'}${label}目标值" id="number-${name}"></div>`;
      $('joints').append(row);
      $(name).oninput=()=>{$(`number-${name}`).value=$(name).value;};
      $(`number-${name}`).oninput=()=>{$(name).value=$(`number-${name}`).value;};
    }
  }
  initialized=true;
}
async function send(path,body){
  try{
    const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const data=await response.json();if(!response.ok)throw new Error(data.error||'请求失败');
    $('message').textContent=path.includes('cancel')?'已请求取消轨迹':`已提交 ${data.points} 个轨迹点，总时长 ${data.duration.toFixed(1)} 秒`;
  }catch(error){$('message').textContent=error.message;}
}
$('demo').onclick=()=>send('/api/trajectory',{preset:'demo'});
$('home').onclick=()=>send('/api/trajectory',{preset:'home',duration:4});
$('cancel').onclick=()=>send('/api/cancel',{});
$('execute').onclick=()=>send('/api/trajectory',{positions:names.map(name=>Number($(`number-${name}`).value)),duration:Number($('duration').value)});
$('duration').oninput=()=>{$('duration-label').textContent=`${$('duration').value} 秒`;};
let pending=false;
async function poll(){
  if(pending)return;pending=true;
  try{
    const response=await fetch('/api/state',{cache:'no-store'});if(!response.ok)throw new Error();const state=await response.json();
    if(!initialized)build(state.joints);
    $('connection').textContent=state.ready?'虚拟硬件已连接':'等待控制器就绪';$('connection').classList.toggle('live',state.ready);
    const base=state.base;
    if(base){
      $('base-status').textContent=base.ready?'已连接':'等待里程计';
      $('base-linear').textContent=`${base.linear.toFixed(2)} m/s`;
      $('base-angular').textContent=`${base.angular.toFixed(2)} rad/s`;
      $('base-position').textContent=`X ${base.x.toFixed(3)} · Y ${base.y.toFixed(3)} · 航向 ${(base.yaw*180/Math.PI).toFixed(1)}°`;
      $('wheel-feedback').textContent=`左轮 ${base.left_position.toFixed(2)} rad · 右轮 ${base.right_position.toFixed(2)} rad`;
      document.querySelectorAll('[data-base]').forEach(button=>{button.disabled=!base.ready;});
    }
    const active=['queued','running','canceling'].includes(state.execution.status);
    for(const id of ['demo','home','execute'])$(id).disabled=!state.ready||active;
    $('cancel').disabled=!active;
    $('execution').textContent=state.execution.message;
    const maxError=Math.max(0,...state.execution.error.map(Math.abs));$('error-value').textContent=`最大位置跟随误差 ${maxError.toFixed(5)} rad`;
    for(const [name,value]of Object.entries(state.positions))$(`actual-${name}`).textContent=`反馈 ${value.toFixed(3)}`;
  }catch{$('connection').textContent='连接中断 · 正在重连';$('connection').classList.remove('live');baseStop();for(const id of ['demo','home','execute'])$(id).disabled=true;}
  finally{pending=false;}
}
setInterval(poll,200);poll();

let baseDirection=null,baseSending=false;
const baseKeys={w:'forward',s:'backward',a:'left',d:'right',ArrowUp:'forward',ArrowDown:'backward',ArrowLeft:'left',ArrowRight:'right'};
async function baseCommand(stopping=false){
  if(baseSending&&!stopping)return;
  const speed=Number($('base-speed').value);
  const command={linear:baseDirection==='forward'?speed:baseDirection==='backward'?-speed:0,angular:baseDirection==='left'?.6:baseDirection==='right'?-.6:0};
  baseSending=true;
  try{await fetch('/api/cmd_vel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(command),keepalive:stopping});}
  catch{}finally{baseSending=false;}
}
function baseDrive(direction){baseDirection=direction;document.querySelectorAll('[data-base]').forEach(button=>button.classList.toggle('active',button.dataset.base===direction));baseCommand();}
function baseStop(){baseDirection=null;document.querySelectorAll('[data-base]').forEach(button=>button.classList.remove('active'));baseCommand(true);}
for(const button of document.querySelectorAll('[data-base]')){
  button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture(event.pointerId);baseDrive(button.dataset.base);});
  for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,baseStop);
}
$('base-stop').onclick=baseStop;
$('base-speed').oninput=()=>{$('base-speed-label').textContent=`${Number($('base-speed').value).toFixed(2)} m/s`;};
window.addEventListener('keydown',event=>{
  if(event.code==='Space'){event.preventDefault();baseStop();return;}
  if(event.target.matches('input')||event.repeat)return;
  const key=event.key.length===1?event.key.toLowerCase():event.key;
  if(baseKeys[key]){event.preventDefault();baseDrive(baseKeys[key]);}
});
window.addEventListener('keyup',event=>{const key=event.key.length===1?event.key.toLowerCase():event.key;if(baseKeys[key]===baseDirection)baseStop();});
window.addEventListener('blur',baseStop);window.addEventListener('pagehide',baseStop);
document.addEventListener('visibilitychange',()=>{if(document.hidden)baseStop();});
setInterval(()=>{if(baseDirection)baseCommand();},100);
