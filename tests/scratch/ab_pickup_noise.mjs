// First-attempt pick-up success under different sensor-noise realisations: argv[2] cargo index, argv[3] = number of noise draws discarded
// before the run (shifts the whole pseudo-random stream). Prints one line.
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { ROVER } from '../../src/sim/config.js';
import { gauss } from '../../src/sim/sensors.js';
await RAPIER.init();
if (process.env.FOV) ROVER.camera.hFovDeg = +process.env.FOV; if (process.env.STEREO === '0') ROVER.camera.stereo = false; if (process.env.CH) ROVER.lidar.channels = +process.env.CH; if (process.env.NOIL) ROVER.lidar.interlace = false;
const TAG = ['FOV', 'STEREO', 'CH', 'NOIL', 'NOCRAWL', 'NOFF'].filter((k) => process.env[k]).map((k) => k + '=' + process.env[k]).join(' ');
const ci = Number(process.argv[2] || 0), skip = Number(process.argv[3] || 0);
const sim = new Sim(RAPIER), A = sim.autonomy, R = sim.rover, c = sim.cargo[ci], T = c.T, ct = c.body.translation();
for (let i = 0; i < skip; i++) gauss();
let fails = [], lastDetail = ''; sim.on('log', (e) => { if (/not centred|Clamp attempt|did not lift|lateral/.test(e.msg)) fails.push(`${e.msg.slice(0, 70)} <${lastDetail.slice(0, 90)}>`); });
const back = T.length / 2 + ROVER.frame.halfLen + 22 - (T.carryX || 0);
R.teleport(ct.x - Math.cos(c.yaw) * back, ct.z + Math.sin(c.yaw) * back, c.yaw);
for (let i = 0; i < 480; i++) sim.stepOnce();
R.brake = false; sim.slam.clear(); sim.slam.relocalize(R.pose()); A.cargoEst.clear(); sim.streamRocks([{ x: ct.x, z: ct.z }]);
const t0 = sim.time; A.start('pickup', A.acquire(c));
if (process.env.NOCRAWL) Object.defineProperty(R, 'crawlZone', { get: () => false, set: () => {} });
while (A.gen && sim.time - t0 < 900) { sim.stepOnce(); if (/Drive under/.test(A.phase) && A.detail) lastDetail = A.detail; }
console.log(`[${TAG}] cargo ${c.id} skip ${String(skip).padStart(4)}: ${R.loaded ? 'held' : 'FAILED'} in ${(sim.time - t0).toFixed(0)} s, ${fails.length ? 'retries: ' + fails.join(' | ') : 'first attempt'}`);
