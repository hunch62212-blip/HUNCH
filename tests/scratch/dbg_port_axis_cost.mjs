// Costmap along each port's approach axis for a loaded rover (is the lead-in / pre-dock line clear of the keep-outs?)
import RAPIER from '@dimforge/rapier3d-compat';
import { Sim } from '../../src/sim/sim.js';
import { LETHAL } from '../../src/sim/planner.js';
await RAPIER.init();
const sim = new Sim(RAPIER), P = sim.autonomy.planner, R = sim.rover;
for (const ci of [0, 1]) {
  const c = sim.cargo[ci]; R.lift.cargo = c; // planner reads the held cargo for its tall-structure margin
  P.buildCostmap();
  for (const port of sim.ports) {
    const row = [];
    for (let v = 10; v <= 70; v += 4) { const k = P.idx(port.x + port.nx * v, port.z + port.nz * v), cst = P.cost[k]; row.push(cst >= LETHAL ? 'X' : cst >= LETHAL * 0.5 ? 'x' : cst.toFixed(0)); }
    console.log(`${c.id} ${port.name} axis cost, 10..70 m out from the port: ${row.join(' ')}`);
  }
}
R.lift.cargo = null;
