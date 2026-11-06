// ===== Rear load case engine (pure functions, no DOM) =====
// Units: inches, lbf, psi. Axes: X lateral (+ outboard), Y vertical (+ up), Z longitudinal.
const LBF_TO_N = 4.448222;
const PSI_TO_MPA = 0.00689476;

const MATERIALS = {
  '6061-T6 Al':      { Sy: 40000, E: 10.0e6 },
  '7075-T6 Al':      { Sy: 73000, E: 10.4e6 },
  '4130 Steel (N)':  { Sy: 63100, E: 29.7e6 },
};

const POINT_KEYS = [
  ['taIn',   'TA pivot (TA IN)'],
  ['taOut',  'TA OUT (upright mount)'],
  ['uclIn',  'Upper link IN'],
  ['uclOut', 'Upper link OUT'],
  ['lclIn',  'Lower link IN'],
  ['lclOut', 'Lower link OUT'],
  ['shIn',   'Shock IN (chassis)'],
  ['shOut',  'Shock OUT (TA)'],
  ['contact','Tire contact patch'],
];

const DEFAULTS = {
  vehicle: {
    carWeight: 420, driverWeight: 140, rearBias: 0.521, rearWheels: 2,
    cgh: 25.2, wheelbase: 58, track: 49.5, rearRollDist: 0.521,
  },
  geometry: {
    bump: { label: 'Max bump', pts: {
      taIn:[-93.6,14.02,115.36], taOut:[-85.93,19.12,94.11],
      uclIn:[-103.15,14.72,89.27], uclOut:[-85.74,21.58,92.87],
      lclIn:[-103.15,11.72,89.66], lclOut:[-85.36,18.59,91.39],
      shIn:[-91.04,27.51,112.35], shOut:[-88.7,18.34,101.62],
      contact:[-81.78,9.23,89.05] } },
    static: { label: 'Static ride height', pts: {
      taIn:[35.35,54.78,58.65], taOut:[43.71,49.72,37.65],
      uclIn:[25.8,55.48,32.55], uclOut:[44.17,51.4,35.5],
      lclIn:[25.8,52.48,32.95], lclOut:[44.25,48.04,35.44],
      shIn:[37.9,68.28,55.64], shOut:[40.81,52.44,44.64],
      contact:[46.85,38.29,37.43] } },
    droop: { label: 'Full droop', pts: {
      taIn:[-93.6,14.02,115.36], taOut:[-86.32,5.98,94.9],
      uclIn:[-103.15,14.72,89.27], uclOut:[-85.88,7.34,92.54],
      lclIn:[-103.15,11.72,89.66], lclOut:[-85.93,4.0,92.91],
      shIn:[-91.04,27.51,112.35], shOut:[-88.82,9.68,101.58],
      contact:[-83.6,-5.51,96.04] } },
  },
  // Wheel force model (per case):
  //   base   = static corner load, or total vehicle weight (basis = 'total')
  //   Fy     = vertG*base + [transfer ? longT*longG - latT*latG : 0]
  //   Fx     = latG * Fy        (lateral tire force)
  //   Fz     = longG * Fy       (longitudinal tire force)
  cases: [
    { id:'bump',    name:'Max Bump',            geom:'bump',   basis:'corner', vertG:5, longG:0,    latG:0,    transfer:false,
      why:'Landing a jump: 5 g vertical on one rear corner.' },
    { id:'onewheel',name:'One-Wheel Landing',   geom:'bump',   basis:'total',  vertG:5, longG:0,    latG:0,    transfer:false,
      why:'Worst case: the whole car (5 g) lands on one wheel. Very conservative.' },
    { id:'corner',  name:'Cornering',           geom:'static', basis:'corner', vertG:1, longG:0,    latG:-1.0, transfer:true,
      why:'1.0 g turn (about 2x the 0.47 g measured on skidpad), outside rear wheel. Lateral weight transfer adds load.' },
    { id:'brake',   name:'Max Brake',           geom:'droop',  basis:'corner', vertG:1, longG:-0.6, latG:0,    transfer:true,
      why:'0.6 g braking. Rear unloads toward droop; tire drags backward.' },
    { id:'accel',   name:'Max Accel',           geom:'static', basis:'corner', vertG:1, longG:0.3,  latG:0,    transfer:true,
      why:'0.3 g acceleration. Weight shifts onto the rear.' },
    { id:'impact',  name:'Frontal Impact',      geom:'static', basis:'corner', vertG:1, longG:-3,   latG:0,    transfer:false,
      why:'Wheel hits an obstacle: 3× corner load pushing rearward. No weight transfer.' },
  ],
  links: {
    upper: { name:'Upper camber link', mat:'6061-T6 Al',     section:'tube', od:0.75, id:0.62 },
    lower: { name:'Lower camber link', mat:'4130 Steel (N)', section:'tube', od:0.75, id:0.63 },
  },
  check: { designFactor: 1.5, K: 1.0 },
};

// ---------- vector helpers ----------
const sub = (a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const add = (a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const scl = (a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const dot = (a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm = a=>Math.sqrt(dot(a,a));
const unit = a=>scl(a,1/norm(a));

// ---------- step 1: vehicle ----------
function vehicleLoads(v){
  const W = v.carWeight + v.driverWeight;
  const corner = W*v.rearBias/v.rearWheels;
  const longT = W*v.cgh/v.wheelbase/v.rearWheels;
  const latT  = W*v.cgh/v.track*v.rearRollDist;
  return { W, corner, longT, latT };
}

// ---------- step 2: wheel force ----------
function wheelForce(c, vl){
  const base = c.basis==='total' ? vl.W : vl.corner;
  const transferY = c.transfer ? (vl.longT*c.longG - vl.latT*c.latG) : 0;
  const Fy = c.vertG*base + transferY;
  return { base, transferY, Fx: c.latG*Fy, Fy, Fz: c.longG*Fy };
}

// ---------- linear solve (Gaussian elimination, partial pivot) ----------
function solveLinear(A, b){
  const n=b.length, M=A.map((r,i)=>[...r,b[i]]);
  for(let k=0;k<n;k++){
    let p=k; for(let i=k+1;i<n;i++) if(Math.abs(M[i][k])>Math.abs(M[p][k])) p=i;
    if(Math.abs(M[p][k])<1e-12) return null;
    [M[k],M[p]]=[M[p],M[k]];
    for(let i=k+1;i<n;i++){ const f=M[i][k]/M[k][k]; for(let j=k;j<=n;j++) M[i][j]-=f*M[k][j]; }
  }
  const x=new Array(n).fill(0);
  for(let i=n-1;i>=0;i--){ let s=M[i][n]; for(let j=i+1;j<n;j++) s-=M[i][j]*x[j]; x[i]=s/M[i][i]; }
  return x;
}

// ---------- step 3/4: statics ----------
function solveCase(c, state){
  const vl = vehicleLoads(state.vehicle);
  const wf = wheelForce(c, vl);
  const P = state.geometry[c.geom].pts;
  const Fw = [wf.Fx, wf.Fy, wf.Fz];
  // unit vectors: links point OUT -> IN (positive force = tension), shock points IN -> OUT (positive = compression)
  const uclVec = sub(P.uclIn, P.uclOut), lclVec = sub(P.lclIn, P.lclOut), shVec = sub(P.shOut, P.shIn);
  const u1 = unit(uclVec), u2 = unit(lclVec), u3 = unit(shVec);
  const ru = sub(P.uclOut, P.taIn), rl = sub(P.lclOut, P.taIn), rs = sub(P.shOut, P.taIn), rc = sub(P.contact, P.taIn);
  const m1 = cross(ru,u1), m2 = cross(rl,u2), m3 = cross(rs,u3), mw = cross(rc,Fw);
  const A = [
    [1,0,0, u1[0],u2[0],u3[0]],
    [0,1,0, u1[1],u2[1],u3[1]],
    [0,0,1, u1[2],u2[2],u3[2]],
    [0,0,0, m1[0],m2[0],m3[0]],
    [0,0,0, m1[1],m2[1],m3[1]],
    [0,0,0, m1[2],m2[2],m3[2]],
  ];
  const b = [-Fw[0],-Fw[1],-Fw[2],-mw[0],-mw[1],-mw[2]];
  const x = solveLinear(A,b);
  if(!x) return { error:'Matrix is singular: two members are lined up or a hardpoint is duplicated.', c, wf, P };
  const [Rx,Ry,Rz,F1,F2,Fs] = x;
  const R=[Rx,Ry,Rz];
  // equilibrium residual check
  const sumF = add(add(add(add(R, scl(u1,F1)), scl(u2,F2)), scl(u3,Fs)), Fw);
  const sumM = add(add(add(cross(ru,scl(u1,F1)), cross(rl,scl(u2,F2))), cross(rs,scl(u3,Fs))), mw);
  // upright free body -> loads into trailing arm at TA OUT
  const fU = scl(u1,F1), fL = scl(u2,F2);
  const toTA = add(add(fU,fL),Fw);
  const momTA = add(add(cross(sub(P.uclOut,P.taOut),fU), cross(sub(P.lclOut,P.taOut),fL)), cross(sub(P.contact,P.taOut),Fw));
  return {
    c, vl, wf, P, Fw, u1,u2,u3, ru,rl,rs,rc, m1,m2,m3, mw, A, b, x,
    R, Rmag:norm(R), F1, F2, Fs,
    Lu:norm(uclVec), Ll:norm(lclVec), Ls:norm(shVec),
    residF:norm(sumF), residM:norm(sumM),
    toTA, toTAmag:norm(toTA), momTA, momTAmag:norm(momTA),
  };
}

// ---------- step 5: member check ----------
function sectionProps(l){
  const id = l.section==='bar' ? 0 : l.id;
  const A = Math.PI*(l.od**2 - id**2)/4;
  const I = Math.PI*(l.od**4 - id**4)/64;
  return { A, I, r: Math.sqrt(I/A) };
}
function checkLink(F, L, link, chk){
  const m = MATERIALS[link.mat], s = sectionProps(link);
  const stress = Math.abs(F)/s.A;
  const tension = F>=0;
  const slender = chk.K*L/s.r;
  const transition = Math.sqrt(2*Math.PI**2*m.E/m.Sy);
  const euler = Math.PI**2*m.E/slender**2;
  const johnson = m.Sy - (m.Sy**2/(4*Math.PI**2*m.E))*slender**2;
  const regime = slender < transition ? 'Johnson' : 'Euler';
  const buckle = regime==='Johnson' ? johnson : euler;
  const allow = tension ? m.Sy : Math.min(buckle, m.Sy);
  const fos = stress>0 ? allow/stress : Infinity;
  const margin = fos/chk.designFactor - 1;
  return { F, L, stress, tension, slender, transition, euler, johnson, regime, buckle, allow, fos, margin, mat:m, sec:s,
           Pcr: buckle*s.A };
}

// ---------- run everything ----------
function runAll(state){
  const vl = vehicleLoads(state.vehicle);
  const cases = state.cases.map(c=>solveCase(c,state));
  const checks = cases.map(r=> r.error ? null : ({
    upper: checkLink(r.F1, r.Lu, state.links.upper, state.check),
    lower: checkLink(r.F2, r.Ll, state.links.lower, state.check),
  }));
  const ok = cases.filter(r=>!r.error && r.c.env!==false);
  // envelope: largest value of fn over included cases, with the case that produced it
  const env = (fn)=>{ let best={v:0,name:'—',r:null}; ok.forEach(r=>{ const v=fn(r); if(v>best.v) best={v,name:r.c.name,r}; }); return best; };
  const joints = {
    uclT: env(r=>Math.max(0,r.F1)),  uclC: env(r=>Math.max(0,-r.F1)),
    lclT: env(r=>Math.max(0,r.F2)),  lclC: env(r=>Math.max(0,-r.F2)),
    shC:  env(r=>Math.max(0,r.Fs)),  shT:  env(r=>Math.max(0,-r.Fs)),
    pivot: env(r=>r.Rmag), taF: env(r=>r.toTAmag), taM: env(r=>r.momTAmag),
  };
  return { vl, cases, checks, joints };
}

if (typeof module !== 'undefined') module.exports = { DEFAULTS, MATERIALS, runAll, solveCase, checkLink, vehicleLoads, wheelForce, LBF_TO_N };
