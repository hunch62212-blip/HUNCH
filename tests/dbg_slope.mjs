import { generateTerrain, heightAt } from '../src/sim/terrain.js';
import { WORLD } from '../src/sim/config.js';
const t = generateTerrain();
// slope map at 4 m scale (rover footprint ~10 m): max slope over 8 m baseline
const N = 512, S = 4, H = 1024;
const sl = new Float32Array(N*N);
for (let j=0;j<N;j++) for (let i=0;i<N;i++){ const x=i*S-H+S/2, z=j*S-H+S/2; const b=5;
  const gx=(heightAt(t,x+b,z)-heightAt(t,x-b,z))/(2*b), gz=(heightAt(t,x,z+b)-heightAt(t,x,z-b))/(2*b);
  sl[j*N+i]=Math.atan(Math.hypot(gx,gz))*57.3; }
for (const lim of [4.5, 6, 8.5, 12]) {
  // BFS from pad to habitat
  const idx=(x,z)=>Math.floor((z+H)/S)*N+Math.floor((x+H)/S);
  const s=idx(-850+30,-520+30), g=idx(830-30,560-30); const seen=new Uint8Array(N*N); const q=[s]; seen[s]=1; let ok=false;
  while(q.length){const k=q.pop(); if(k===g){ok=true;break;} const i=k%N,j=(k/N)|0; for(const [di,dj] of [[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]]){const a=i+di,b=j+dj; if(a<0||b<0||a>=N||b>=N)continue; const kk=b*N+a; if(seen[kk]||sl[kk]>lim)continue; seen[kk]=1;q.push(kk);} }
  let cnt=0; for(let k=0;k<N*N;k++) if(sl[k]>lim) cnt++;
  console.log('limit',lim,'reachable',ok,'steep frac',(cnt/N/N).toFixed(3));
}
