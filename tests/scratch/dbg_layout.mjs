import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { WORLD } from '../../src/sim/config.js';
await RAPIER.init();
const sim = new Sim(RAPIER);
const deg = (a) => (a * 180 / Math.PI).toFixed(0);
console.log('pad', WORLD.pad, 'lander', sim.landerPos, 'hab', WORLD.habitat);
const hd = Math.atan2(-(WORLD.habitat.z - WORLD.pad.z), WORLD.habitat.x - WORLD.pad.x);
console.log('bearing pad->hab (yaw convention) deg', deg(hd));
for (const c of sim.cargo) { const t = c.body.translation(); console.log(c.id, 'pos', t.x.toFixed(0), t.z.toFixed(0), 'yaw', deg(c.yaw), 'fwd axis', Math.cos(c.yaw).toFixed(2), (-Math.sin(c.yaw)).toFixed(2), 'dist to pad centre', Math.hypot(t.x - WORLD.pad.x, t.z - WORLD.pad.z).toFixed(0), 'dest', c.dest,
  'angle between module fwd axis and hab bearing', deg(Math.atan2(Math.sin(c.yaw - hd), Math.cos(c.yaw - hd)))); }
console.log('staging', sim.terrain.pad.staging, 'rover', sim.rover.pose().x.toFixed(0), sim.rover.pose().z.toFixed(0));
