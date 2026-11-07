import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { heightAt } from '../../src/sim/terrain.js';
await RAPIER.init();
const sim = new Sim(RAPIER); const A = sim.autonomy, R = sim.rover;
A.start('cycle', A.missionCycle());
while (A.gen && sim.time < 372) sim.stepOnce();
const P = A.planner; P.buildCostmap(); const p = R.pose();
console.log('pos', p.x.toFixed(0), p.z.toFixed(0));
const cx = Math.round(p.x/8)*8, cz = Math.round(p.z/8)*8;
for (let z=cz-120; z<=cz+120; z+=8) { let row=''; for (let x=cx-200; x<=cx+240; x+=8) {
  const k=P.idx(x,z); const c=P.cost[k];
  const b=4,t=sim.terrain; const gx=(heightAt(t,x+b,z)-heightAt(t,x-b,z))/(2*b), gz=(heightAt(t,x,z+b)-heightAt(t,x,z-b))/(2*b); const sl=Math.atan(Math.hypot(gx,gz))*57.3;
  let ch = c>=1e9?'#': c>=5e8?'X': c===P.unknownCost?(sl>8.5?'^':'.'): c>6?'+':c>2?'-':' ';
  if (Math.abs(x-p.x)<5 && Math.abs(z-p.z)<5) ch='R';
  row+=ch;} console.log(String(z).padStart(5), row); }
console.log('x from', cx-200, 'step 8');
