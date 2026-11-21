// sokoban-solver v1 — push-based A* per room.
//
// The level decomposes into a 1-wide hallway plus 1-4 rooms (each in a 6x6 slot,
// one door onto the hallway). A box pushed through a door into the hallway can
// never be pushed back, so every box stays in its room and the level's
// push-optimal count is the sum of the rooms' optima. Each room is therefore
// solved independently, push-optimally, from its door, and the hallway walks
// between rooms are concatenated.
//
// Per room: A* over pushes (not moves — the score counts pushes), with
//   - admissible heuristic: minimum assignment of boxes to goals by
//     shortest-path (push) distance, brute-forced over goal permutations
//   - deadlock pruning: pull-reachability dead cells (a box that can never be
//     pulled to any goal is dead) plus frozen 2x2 blocks of walls/boxes
//   - transposition table on (player, boxes), binary heap on pushes+estimate
// Deterministic: budgets are counted in expansions, never wall-clock time.
// Runs as: node /bot/main.ts  (Node 22 strips types natively, erasable syntax only)
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Total expansion budget for the whole set (5 instances), plus a per-room cap so
// one pathological room cannot starve the rest. Deterministic (never clock).
// Measured: ~180us/expansion, practice set uses ~31k total -> 200k is ~6x
// headroom and worst case ~36s of the 60s wall.
const GLOBAL_BUDGET = 200_000;
const ROOM_BUDGET = 80_000;
const budget = { left: GLOBAL_BUDGET };

const DIRS: Array<[number, number, string]> = [[0, -1, 'u'], [0, 1, 'd'], [-1, 0, 'l'], [1, 0, 'r']];

interface Level {
  w: number;
  h: number;
  wall: boolean[];
  goals: number[];
  boxes: number[];
  player: number;
}

function step(c: number, dx: number, dy: number, w: number, h: number): number {
  const x = c % w;
  const y = (c / w) | 0;
  const nx = x + dx;
  const ny = y + dy;
  if (nx < 0 || nx >= w || ny < 0 || ny >= h) return -1;
  return ny * w + nx;
}

function parse(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall: boolean[] = new Array(w * h).fill(false);
  const goals: number[] = [];
  let boxes: number[] = [];
  let player = 0;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#';
      const i = y * w + x;
      if (ch === '#') wall[i] = true;
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS distances from src over walkable cells. -1 = unreachable.
function bfsDist(src: number, walkable: boolean[], w: number, h: number): Int32Array {
  const dist = new Int32Array(w * h).fill(-1);
  if (!walkable[src]) return dist;
  const q: number[] = [src];
  dist[src] = 0;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    for (const [dx, dy] of DIRS) {
      const n = step(c, dx, dy, w, h);
      if (n >= 0 && walkable[n] && dist[n] < 0) {
        dist[n] = dist[c]! + 1;
        q.push(n);
      }
    }
  }
  return dist;
}

// BFS parents from `from` over walkable cells avoiding boxSet. Returns parent
// array (-1 unreached, parent[from] = from) and the step letter into each cell.
function bfsParents(from: number, walkable: boolean[], boxSet: Set<number>, w: number, h: number): { parent: Int32Array; pch: string[] } {
  const parent = new Int32Array(w * h).fill(-1);
  const pch: string[] = new Array(w * h).fill('');
  const q: number[] = [from];
  parent[from] = from;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi]!;
    for (const [dx, dy, ch] of DIRS) {
      const n = step(c, dx, dy, w, h);
      if (n < 0 || !walkable[n] || boxSet.has(n) || parent[n] >= 0) continue;
      parent[n] = c;
      pch[n] = ch;
      q.push(n);
    }
  }
  return { parent, pch };
}

function pathTo(to: number, from: number, parent: Int32Array, pch: string[]): string | null {
  if (parent[to] < 0) return null;
  let s = '';
  let cur = to;
  while (cur !== from) {
    s = pch[cur] + s;
    cur = parent[cur]!;
  }
  return s;
}

function canReach(from: number, to: number, walkable: boolean[], boxSet: Set<number>, w: number, h: number): boolean {
  if (from === to) return true;
  const { parent } = bfsParents(from, walkable, boxSet, w, h);
  return parent[to] >= 0;
}

// Shortest walk from `from` to `to` avoiding boxes, as move letters, or null.
function walkPath(from: number, to: number, walkable: boolean[], boxSet: Set<number>, w: number, h: number): string | null {
  if (from === to) return '';
  const { parent, pch } = bfsParents(from, walkable, boxSet, w, h);
  return pathTo(to, from, parent, pch);
}

// Pull-reachability dead cells: a cell is dead when a box on it can never be
// pulled (ignoring other boxes) onto any goal. Reverse BFS over (box, player)
// pairs from every (goal, any player position). Sound: anything unreachable
// here is unreachable in the real puzzle too.
function computeDead(cells: boolean[], goals: number[], w: number, h: number): boolean[] {
  const N = w * h;
  const visited = new Uint8Array(N * N);
  const queue: number[] = [];
  const pushPair = (b: number, p: number): void => {
    const k = b * N + p;
    if (!visited[k]) {
      visited[k] = 1;
      queue.push(k);
    }
  };
  for (const g of goals) {
    for (let p = 0; p < N; p++) {
      if (cells[p] && p !== g) pushPair(g, p);
    }
  }
  for (let qi = 0; qi < queue.length; qi++) {
    const k = queue[qi]!;
    const b = Math.floor(k / N);
    const p = k % N;
    for (const [dx, dy] of DIRS) {
      const p2 = step(p, dx, dy, w, h);
      if (p2 >= 0 && cells[p2] && p2 !== b) pushPair(b, p2);
    }
    const bx = b % w;
    const by = (b / w) | 0;
    const px = p % w;
    const py = (p / w) | 0;
    if (Math.abs(bx - px) + Math.abs(by - py) === 1) {
      const rx = 2 * px - bx;
      const ry = 2 * py - by;
      if (rx >= 0 && rx < w && ry >= 0 && ry < h) {
        const rp = ry * w + rx;
        if (cells[rp]) pushPair(p, rp);
      }
    }
  }
  const dead: boolean[] = new Array(N).fill(false);
  for (let c = 0; c < N; c++) {
    if (!cells[c]) continue;
    let good = false;
    for (let p = 0; p < N; p++) {
      if (visited[c * N + p]) {
        good = true;
        break;
      }
    }
    dead[c] = !good;
  }
  return dead;
}

interface Room {
  cells: boolean[]; // walkable cells: room floor + entry cell (or whole level for fallback)
  door: number; // room cell adjacent to hallway (-1 for fallback)
  entry: number; // hallway cell adjacent to door (-1 for fallback)
  boxes: number[];
  goals: number[];
  dead: boolean[];
  goalDist: Int32Array[];
  windows: number[][]; // 2x2 blocks, all cells wall-or-room-cell
}

// Pack (player, sorted boxes) into one number. Boards are at most 15x17 = 255
// cells, so cell indices fit in a byte and 255 is a safe "no box" pad.
function pack(p: number, boxes: number[]): number {
  let k = p;
  for (const b of boxes) k = k * 256 + b;
  for (let i = boxes.length; i < 4; i++) k = k * 256 + 255;
  return k;
}

function unpackBoxes(k: number): number[] {
  const boxes: number[] = [];
  for (let i = 0; i < 4; i++) {
    const b = k % 256;
    k = Math.floor(k / 256);
    if (b !== 255) boxes.unshift(b);
  }
  return boxes;
}

function unpackPlayer(k: number): number {
  return Math.floor(k / 256 ** 4);
}

const PERMS: Record<number, number[][]> = (() => {
  const out: Record<number, number[][]> = {};
  for (const n of [1, 2, 3, 4]) {
    const arr = Array.from({ length: n }, (_, i) => i);
    const res: number[][] = [];
    const gen = (a: number[], m: number): void => {
      if (m === 1) {
        res.push(a.slice());
        return;
      }
      for (let i = 0; i < m; i++) {
        gen(a, m - 1);
        const j = m % 2 === 1 ? 0 : i;
        [a[j], a[m - 1]] = [a[m - 1]!, a[j]!];
      }
    };
    gen(arr, n);
    out[n] = res;
  }
  return out;
})();

interface SearchResult {
  moves: string;
  endPlayer: number;
  endBoxes: number[];
  expansions: number;
}

function solveRoom(room: Room, startPlayer: number, startBoxes: number[], needExit: boolean, w: number, h: number): SearchResult | null {
  const goalSet = new Set(room.goals);
  const n = room.boxes.length;
  const perms = PERMS[n]!;
  const hCache = new Map<number, number>();
  // Reused BFS scratch: parent/dir/seen-token + queue, so expansions do not
  // allocate per state (allocation was the main cost: GC ~18% of runtime).
  const N = w * h;
  const bfsParent = new Int32Array(N);
  const bfsDir = new Uint8Array(N);
  const bfsSeen = new Int32Array(N);
  const bfsQ = new Int32Array(N);
  const boxAt = new Uint8Array(N);
  let bfsTok = 0;
  const bfsFrom = (from: number): void => {
    bfsTok++;
    let qh = 0;
    let qt = 0;
    bfsQ[qt++] = from;
    bfsSeen[from] = bfsTok;
    bfsParent[from] = from;
    while (qh < qt) {
      const c = bfsQ[qh++]!;
      for (let d = 0; d < 4; d++) {
        const dd = DIRS[d]!;
        const nn = step(c, dd[0], dd[1], w, h);
        if (nn < 0 || !room.cells[nn] || boxAt[nn] || bfsSeen[nn] === bfsTok) continue;
        bfsSeen[nn] = bfsTok;
        bfsParent[nn] = c;
        bfsDir[nn] = d;
        bfsQ[qt++] = nn;
      }
    }
  };
  const bfsPath = (to: number, from: number): string | null => {
    if (bfsSeen[to] !== bfsTok) return null;
    let s = '';
    let cur = to;
    while (cur !== from) {
      s = DIRS[bfsDir[cur]!]![2] + s;
      cur = bfsParent[cur]!;
    }
    return s;
  };
  const heuristic = (boxes: number[]): number => {
    const key = pack(0, boxes);
    const hit = hCache.get(key);
    if (hit !== undefined) return hit;
    let best = Infinity;
    for (const perm of perms) {
      let s = 0;
      let ok = true;
      for (let i = 0; i < n; i++) {
        const d = room.goalDist[perm[i]!]![boxes[i]!]!;
        if (d < 0) {
          ok = false;
          break;
        }
        s += d;
      }
      if (ok && s < best) best = s;
    }
    hCache.set(key, best);
    return best;
  };
  // Only windows containing the moved box can newly freeze: any window frozen
  // before the move would already have pruned the parent state.
  const windowsByCell = new Map<number, number[][]>();
  for (const win of room.windows) {
    for (const c of win) {
      const list = windowsByCell.get(c);
      if (list) list.push(win);
      else windowsByCell.set(c, [win]);
    }
  }
  const frozen2x2 = (boxes: number[], moved: number): boolean => {
    const wins = windowsByCell.get(moved);
    if (!wins) return false;
    for (const win of wins) {
      let allBlocked = true;
      let hasBox = false;
      let boxOnGoal = true;
      for (const c of win) {
        if (room.cells[c]) {
          if (boxes.includes(c)) {
            hasBox = true;
            if (!goalSet.has(c)) boxOnGoal = false;
          } else {
            allBlocked = false;
          }
        }
      }
      if (allBlocked && hasBox && !boxOnGoal) return true;
    }
    return false;
  };
  const isGoal = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));

  const h0 = heuristic(startBoxes);
  if (!isFinite(h0)) return null;
  for (const b of startBoxes) if (room.dead[b]!) return null;

  // State arrays; state 0 is the start.
  const sp: number[] = [startPlayer];
  const sb: number[] = [pack(startPlayer, startBoxes)];
  const sg: number[] = [0];
  const sparent: number[] = [-1];
  const sseg: string[] = [''];
  const best = new Map<number, number>([[sb[0]!, 0]]);
  // Heap of [f, stateIndex].
  const heap: Array<[number, number]> = [[h0, 0]];
  const less = (a: [number, number], b: [number, number]): boolean => a[0] < b[0] || (a[0] === b[0] && a[1] < b[1]);
  const hpush = (e: [number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const hpop = (): [number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  let expansions = 0;
  const roomStart = budget.left;
  while (heap.length > 0) {
    const [, s] = hpop();
    const g = sg[s]!;
    const key = sb[s]!;
    if (best.get(key) !== g) continue; // stale entry
    const player = sp[s]!;
    const boxes = unpackBoxes(key);
    if (isGoal(boxes)) {
      if (!needExit || canReach(player, room.entry, room.cells, new Set(boxes), w, h)) {
        let moves = '';
        for (let t = s; t > 0; t = sparent[t]!) moves = sseg[t] + moves;
        return { moves, endPlayer: player, endBoxes: boxes, expansions };
      }
      // All boxes home but the player is trapped inside: keep searching.
    }
    if (budget.left <= 0 || roomStart - budget.left >= ROOM_BUDGET) break;
    budget.left--;
    expansions++;
    boxAt.fill(0);
    for (const b of boxes) boxAt[b] = 1;
    bfsFrom(player);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi]!;
      for (const [dx, dy, ch] of DIRS) {
        const target = step(b, dx, dy, w, h);
        if (target < 0 || !room.cells[target] || boxAt[target]) continue;
        const pushPos = step(b, -dx, -dy, w, h);
        if (pushPos < 0 || bfsSeen[pushPos] !== bfsTok) continue;
        const nb = boxes.slice();
        nb[bi] = target;
        nb.sort((a, z) => a - z);
        let deadHit = false;
        for (const x of nb) {
          if (room.dead[x]!) {
            deadHit = true;
            break;
          }
        }
        if (deadHit) continue;
        const nk = pack(b, nb);
        const ng = g + 1;
        const seen = best.get(nk);
        if (seen !== undefined && seen <= ng) continue;
        if (frozen2x2(nb, target)) continue;
        const hv = heuristic(nb);
        if (!isFinite(hv)) continue;
        const walk = bfsPath(pushPos, player)!;
        const t = sp.length;
        sp.push(b);
        sb.push(nk);
        sg.push(ng);
        sparent.push(s);
        sseg.push(walk + ch);
        best.set(nk, ng);
        hpush([ng + hv, t]);
      }
    }
  }
  return null;
}

function findRooms(lv: Level): Room[] | null {
  const H = (lv.player / lv.w) | 0;
  const walkable = lv.wall.map((x) => !x);
  // No goal may sit in the hallway row; every box/goal must belong to a room.
  for (const g of lv.goals) if (((g / lv.w) | 0) === H) return null;
  const seen: boolean[] = new Array(lv.w * lv.h).fill(false);
  const rooms: Room[] = [];
  const claimedBox = new Set<number>();
  const claimedGoal = new Set<number>();
  for (let i = 0; i < lv.w * lv.h; i++) {
    if (!walkable[i] || ((i / lv.w) | 0) === H || seen[i]) continue;
    const comp: number[] = [];
    const compSet: boolean[] = new Array(lv.w * lv.h).fill(false);
    const q: number[] = [i];
    seen[i] = true;
    compSet[i] = true;
    while (q.length > 0) {
      const c = q.pop()!;
      comp.push(c);
      for (const [dx, dy] of DIRS) {
        const n = step(c, dx, dy, lv.w, lv.h);
        if (n >= 0 && walkable[n] && ((n / lv.w) | 0) !== H && !seen[n]) {
          seen[n] = true;
          compSet[n] = true;
          q.push(n);
        }
      }
    }
    // The room's one door: its cell adjacent to a hallway-row floor cell.
    const doors: number[] = [];
    const entries: number[] = [];
    for (const c of comp) {
      for (const [dx, dy] of DIRS) {
        const n = step(c, dx, dy, lv.w, lv.h);
        if (n >= 0 && ((n / lv.w) | 0) === H && walkable[n]) {
          doors.push(c);
          entries.push(n);
        }
      }
    }
    if (doors.length !== 1) return null;
    const boxes = lv.boxes.filter((b) => compSet[b]).sort((a, b) => a - b);
    const goals = lv.goals.filter((g) => compSet[g]);
    if (boxes.length === 0 || boxes.length !== goals.length) return null;
    for (const b of boxes) claimedBox.add(b);
    for (const g of goals) claimedGoal.add(g);
    const cells: boolean[] = new Array(lv.w * lv.h).fill(false);
    for (const c of comp) cells[c] = true;
    cells[entries[0]!] = true;
    const dead = computeDead(cells, goals, lv.w, lv.h);
    const goalDist = goals.map((g) => bfsDist(g, cells, lv.w, lv.h));
    const windows: number[][] = [];
    for (let y = 0; y < lv.h - 1; y++) {
      for (let x = 0; x < lv.w - 1; x++) {
        const win = [y * lv.w + x, y * lv.w + x + 1, (y + 1) * lv.w + x, (y + 1) * lv.w + x + 1];
        if (win.every((c) => !walkable[c] || cells[c])) windows.push(win);
      }
    }
    rooms.push({ cells, door: doors[0]!, entry: entries[0]!, boxes, goals, dead, goalDist, windows });
  }
  if (claimedBox.size !== lv.boxes.length || claimedGoal.size !== lv.goals.length) return null;
  return rooms;
}

function solveLevel(text: string): string {
  const lv = parse(text);
  const rooms = findRooms(lv);
  if (rooms) {
    rooms.sort((a, b) => a.door - b.door);
    let moves = '';
    let curPlayer = lv.player;
    let curBoxes = lv.boxes.slice();
    const walkableAll = lv.wall.map((x) => !x);
    for (let ri = 0; ri < rooms.length; ri++) {
      const room = rooms[ri]!;
      const last = ri === rooms.length - 1;
      const walk = walkPath(curPlayer, room.entry, walkableAll, new Set(curBoxes), lv.w, lv.h);
      if (walk === null) return '';
      moves += walk;
      const res = solveRoom(room, room.entry, room.boxes, !last, lv.w, lv.h);
      if (!res) return '';
      moves += res.moves;
      curPlayer = res.endPlayer;
      const roomBoxSet = new Set(room.boxes);
      curBoxes = curBoxes.filter((b) => !roomBoxSet.has(b)).concat(res.endBoxes).sort((a, b) => a - b);
    }
    return moves;
  }
  // Fallback: one global push-A* over the whole level.
  const cells = lv.wall.map((x) => !x);
  const room: Room = {
    cells,
    door: -1,
    entry: -1,
    boxes: lv.boxes,
    goals: lv.goals,
    dead: computeDead(cells, lv.goals, lv.w, lv.h),
    goalDist: lv.goals.map((g) => bfsDist(g, cells, lv.w, lv.h)),
    windows: [],
  };
  for (let y = 0; y < lv.h - 1; y++) {
    for (let x = 0; x < lv.w - 1; x++) {
      const win = [y * lv.w + x, y * lv.w + x + 1, (y + 1) * lv.w + x, (y + 1) * lv.w + x + 1];
      if (win.every((c) => !cells[c] || true)) room.windows.push(win);
    }
  }
  const res = solveRoom(room, lv.player, lv.boxes, false, lv.w, lv.h);
  return res ? res.moves : '';
}

for (const id of readdirSync(I).sort()) {
  const t0 = budget.left;
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
  console.error(`${id}: moves=${moves.length} expansions=${t0 - budget.left} budgetLeft=${budget.left}`);
}
