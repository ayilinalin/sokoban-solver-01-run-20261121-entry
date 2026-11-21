# sokoban-solver — push-optimal Sokoban solver

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/ayilinalin/sokoban-solver-01-run-20261121-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/ayilinalin/sokoban-solver-01-run-20261121-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

Entry for the [sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty on make.fun: a push-optimal Sokoban solver in a single `main.ts` (Node 22, no dependencies). It answers a set of 5 levels in one run, inside the 60-second wall budget.

## How it works

- **Room decomposition.** Every level is a 1-wide hallway plus 1–4 rooms (6×6 slots, one door each). A box pushed through a door into the hallway can never come back, so every box stays in its room and the level's push-optimal count is the sum of the rooms' optima. Each room is solved independently from its door; hallway walks between rooms are concatenated.
- **Push-based A\*.** The search runs over pushes, not moves (the score counts pushes): a state is the player square plus the boxes, each step costs one push, and the estimate is the minimum assignment of boxes to goals by shortest-path push distance.
- **Deadlock pruning.** Dead cells come from pull-reachability analysis (a box that can never be pulled onto any goal is dead), plus frozen 2×2 blocks of walls/boxes.
- **Deterministic budgets.** The search is capped by expansion counts (200,000 for the set, 80,000 per room) — never by the wall clock — so two runs write byte-identical answers.

## Practice results

All 5 practice instances solved in exactly P\* pushes — 1,000 points each, **score 1,000,000**. The house baseline plain-astar 1.0.0 scores 200,000 on the same set (it solves only the first level, and gives up on the rest).

## Layout

- `main.ts` — the solver: parse, decompose, push-A\* per room, write `/output/<id>/solution.txt`.
- `.makefun/owner.json` — payout wallet (Solana).
- `LICENSE` — MIT.

## License

MIT. See [LICENSE](LICENSE).
