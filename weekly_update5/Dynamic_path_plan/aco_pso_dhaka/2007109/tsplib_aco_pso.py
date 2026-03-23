import math
import random
import argparse
from typing import List, Tuple, Dict, Optional

# TSPLIB loader (NODE_COORD_SECTION) for EUC_2D instances like eil51

def load_tsplib_coords(tsp_path: str) -> List[Tuple[float, float]]:
    """
    Minimal TSPLIB .tsp parser for NODE_COORD_SECTION datasets (e.g., eil51, berlin52).
    Returns coords indexed 0..n-1 in the order found in the file.
    """
    coords: List[Tuple[float, float]] = []
    in_section = False

    with open(tsp_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("NODE_COORD_SECTION"):
                in_section = True
                continue

            if line.startswith("EOF"):
                break

            if in_section:
                parts = line.split()
                if len(parts) >= 3:
                    # format: node_id x y
                    _, x, y = parts[:3]
                    coords.append((float(x), float(y)))

    if not coords:
        raise ValueError("Could not parse coordinates. Ensure the file has NODE_COORD_SECTION.")
    return coords


def tsplib_euc_2d(a: Tuple[float, float], b: Tuple[float, float]) -> int:
    """
    TSPLIB EUC_2D distance: d(i,j) = int(sqrt(dx^2 + dy^2) + 0.5)
    This matches TSPLIB rounding for eil51.
    """
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return int(math.sqrt(dx * dx + dy * dy) + 0.5)


def build_distance_matrix(coords: List[Tuple[float, float]]) -> List[List[int]]:
    n = len(coords)
    dist = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            d = tsplib_euc_2d(coords[i], coords[j])
            dist[i][j] = d
            dist[j][i] = d
    return dist


# ACO for TSP (tour visits all nodes once and returns)

def roulette_choice(items: List[Tuple[int, float]]) -> int:
    """items: list of (node, weight). weight must be >=0."""
    total = sum(w for _, w in items)
    if total <= 0:
        # fallback uniform
        return random.choice([node for node, _ in items])

    r = random.random() * total
    cum = 0.0
    for node, w in items:
        cum += w
        if r <= cum:
            return node
    return items[-1][0]


def tour_length(tour: List[int], dist: List[List[int]]) -> int:
    """Closed tour length (return to start)."""
    n = len(tour)
    s = 0
    for i in range(n):
        a = tour[i]
        b = tour[(i + 1) % n]
        s += dist[a][b]
    return s


def aco_tsp(
    dist: List[List[int]],
    alpha: float,
    beta: float,
    rho: float,
    num_ants: int = 10,
    max_iters: int = 1000,
    stall_limit: int = 200,  # stop kore aikhane 
    Q: float = 1.0,
    seed: Optional[int] = None,
) -> Tuple[List[int], int]:
    """
    Classic Ant System for TSP:
    - Transition probability uses tau^alpha * (1/d)^beta
    - Pheromone update: tau = (1-rho)*tau + sum_k Q/Lk on edges used by ant k
    Stops early if no improvement for stall_limit iterations.
    """
    if seed is not None:
        random.seed(seed)

    n = len(dist)

    # initialize pheromone matrix
    tau = [[1.0] * n for _ in range(n)]
    for i in range(n):
        tau[i][i] = 0.0  # no self loops

    best_tour: List[int] = []
    best_len = float("inf")
    stall = 0

    for _it in range(max_iters):
        ant_solutions: List[Tuple[List[int], int]] = []

        # construct solutions
        for _k in range(num_ants):
            start = random.randrange(n)
            visited = [False] * n
            visited[start] = True
            tour = [start]
            current = start

            while len(tour) < n:
                candidates = []
                for j in range(n):
                    if not visited[j]:
                        d = dist[current][j]
                        eta = (1.0 / d) if d > 0 else 0.0  # heuristic
                        desirability = (tau[current][j] ** alpha) * (eta ** beta)
                        candidates.append((j, desirability))

                nxt = roulette_choice(candidates)
                visited[nxt] = True
                tour.append(nxt)
                current = nxt

            Lk = tour_length(tour, dist)
            ant_solutions.append((tour, Lk))

            if Lk < best_len:
                best_len = Lk
                best_tour = tour
                stall = 0

        stall += 1
        if stall >= stall_limit:
            break

        # evaporation
        evap = 1.0 - rho
        for i in range(n):
            row = tau[i]
            for j in range(n):
                row[j] *= evap

        # deposit
        for tour, Lk in ant_solutions:
            if Lk <= 0:
                continue
            delta = Q / Lk
            for i in range(n):
                a = tour[i]
                b = tour[(i + 1) % n]
                tau[a][b] += delta
                tau[b][a] += delta  # symmetric

    return best_tour, int(best_len)



# PSO to tune (alpha, beta, rho) with verbose logging

def round_to_step(x: float, step: float) -> float:
    """Round to nearest step without Python's bankers rounding."""
    return math.floor(x / step + 0.5) * step


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def pso_tune_aco_params(
    dist: List[List[int]],
    swarm_size: int = 20,
    pso_iters: int = 30,
    w: float = 0.7,
    c1: float = 1.5,
    c2: float = 1.5,
    # paper-style bounds
    alpha_bounds: Tuple[float, float] = (0.5, 1.5),
    beta_bounds: Tuple[float, float]  = (1.0, 5.0),
    rho_bounds: Tuple[float, float]   = (0.5, 1.0),
    step: float = 0.1,
    # evaluation settings (keep smaller than final run for speed)
    eval_aco_iters: int = 200,
    eval_aco_stall: int = 80,
    eval_runs: int = 3,
    seed: Optional[int] = None,
    verbose: bool = True,
) -> Tuple[Tuple[float, float, float], int]:
    """
    PSO searches for (alpha, beta, rho) minimizing best tour length produced by ACO.
    Uses step=0.1 discretization.
    """
    if seed is not None:
        random.seed(seed)

    cache: Dict[Tuple[float, float, float], int] = {}

    def fitness(a: float, b: float, r: float) -> int:
        key = (a, b, r)
        if key in cache:
            return cache[key]

        bests = []
        for rr in range(eval_runs):
            _tour, L = aco_tsp(
                dist,
                alpha=a, beta=b, rho=r,
                num_ants=10,
                max_iters=eval_aco_iters,
                stall_limit=eval_aco_stall,
                Q=1.0,
                seed=1000 + rr,
            )
            bests.append(L)

        val = int(sum(bests) / len(bests))
        cache[key] = val
        return val

    particles = []
    gbest_pos: Optional[List[float]] = None
    gbest_val = float("inf")

    # ---- init swarm ----
    for idx in range(swarm_size):
        a = round_to_step(random.uniform(*alpha_bounds), step)  
        b = round_to_step(random.uniform(*beta_bounds), step)
        r = round_to_step(random.uniform(*rho_bounds), step)

        va = random.uniform(-0.2, 0.2)        # Swarm initialization
        vb = random.uniform(-0.2, 0.2)
        vr = random.uniform(-0.2, 0.2)

        val = fitness(a, b, r)

        p = {   # each aprticle has these things 
            "pos": [a, b, r],
            "vel": [va, vb, vr],
            "pbest_pos": [a, b, r],
            "pbest_val": val,
        }
        particles.append(p)

        if val < gbest_val:
            gbest_val = val
            gbest_pos = [a, b, r]

        if verbose:
            print(f"[PSO init] particle {idx+1}/{swarm_size} fitness={val}  gbest={gbest_val}")

    assert gbest_pos is not None

    # ---- main PSO loop ----
    for it in range(pso_iters):
        for p in particles:
            # update velocity
            for d in range(3):
                r1 = random.random()
                r2 = random.random()
                p["vel"][d] = (
                    w * p["vel"][d]
                    + c1 * r1 * (p["pbest_pos"][d] - p["pos"][d])
                    + c2 * r2 * (gbest_pos[d] - p["pos"][d])
                )

            # update position + clamp + discretize
            p["pos"][0] = round_to_step(clamp(p["pos"][0] + p["vel"][0], *alpha_bounds), step)
            p["pos"][1] = round_to_step(clamp(p["pos"][1] + p["vel"][1], *beta_bounds), step)
            p["pos"][2] = round_to_step(clamp(p["pos"][2] + p["vel"][2], *rho_bounds), step)

            val = fitness(p["pos"][0], p["pos"][1], p["pos"][2])

            if val < p["pbest_val"]:
                p["pbest_val"] = val
                p["pbest_pos"] = p["pos"][:]

            if val < gbest_val:
                gbest_val = val
                gbest_pos = p["pos"][:]

        if verbose:
            a, b, r = gbest_pos
            print(
                f"[PSO iter {it+1}/{pso_iters}] gbest={gbest_val} "
                f"at alpha={a:.1f}, beta={b:.1f}, rho={r:.1f} | cache={len(cache)}"
            )

    return (gbest_pos[0], gbest_pos[1], gbest_pos[2]), int(gbest_val)


# Helpers: deviation rate


def deviation_rate(avg_best: float, best_known: float) -> float:
    return abs(avg_best - best_known) / best_known * 100.0



# Main


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tsp", type=str, required=True, help="Path to TSPLIB .tsp file (e.g., eil51.tsp)")
    parser.add_argument("--runs", type=int, default=10, help="How many final ACO runs to average")
    parser.add_argument("--plot", action="store_true", help="Plot best tour (needs matplotlib)")

    # PSO knobs (so you can make a fast run first)
    parser.add_argument("--pso-swarm", type=int, default=20)
    parser.add_argument("--pso-iters", type=int, default=30)
    parser.add_argument("--eval-aco-iters", type=int, default=200)
    parser.add_argument("--eval-aco-stall", type=int, default=80)
    parser.add_argument("--eval-runs", type=int, default=3)
    parser.add_argument("--quiet", action="store_true", help="Disable PSO progress logs")

    args = parser.parse_args()

    coords = load_tsplib_coords(args.tsp)
    dist = build_distance_matrix(coords)
    n = len(coords)

    known_best = 426 if n == 51 else None

    print(f"Loaded {args.tsp} with n={n}")
    if known_best is not None:
        print(f"Known best (eil51) = {known_best}")

    print("\n[1] PSO tuning (alpha, beta, rho) ...")
    best_params, pso_est = pso_tune_aco_params(
        dist,
        swarm_size=args.pso_swarm,
        pso_iters=args.pso_iters,
        eval_aco_iters=args.eval_aco_iters,
        eval_aco_stall=args.eval_aco_stall,
        eval_runs=args.eval_runs,
        seed=42,
        verbose=(not args.quiet),
    )
    alpha, beta, rho = best_params
    print(f"\nPSO best params: alpha={alpha:.1f}, beta={beta:.1f}, rho={rho:.1f}")
    print(f"PSO estimated avg-best (during eval) ≈ {pso_est}")

    print("\n[2] Final ACO runs using PSO-tuned params ...")
    best_lengths = []
    best_overall_tour = None
    best_overall_len = float("inf")

    for r in range(args.runs):
        tour, L = aco_tsp(
            dist,
            alpha=alpha, beta=beta, rho=rho,
            num_ants=10,
            max_iters=1000,
            stall_limit=200,
            Q=1.0,
            seed=100 + r,
        )
        best_lengths.append(L)
        if L < best_overall_len:
            best_overall_len = L
            best_overall_tour = tour
        print(f"  run {r+1:02d}: best_len={L}")

    avg_best = sum(best_lengths) / len(best_lengths)
    print("\n[RESULT]")
    print(f"Average best length over {args.runs} runs: {avg_best:.2f}")
    print(f"Best length found: {best_overall_len}")

    if known_best is not None:
        dev = deviation_rate(avg_best, known_best)
        print(f"Deviation rate vs known best: {dev:.2f}%")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not installed. Run: pip install matplotlib")
            return

        if best_overall_tour is None:
            print("No tour to plot.")
            return

        xs = [coords[i][0] for i in best_overall_tour] + [coords[best_overall_tour[0]][0]]
        ys = [coords[i][1] for i in best_overall_tour] + [coords[best_overall_tour[0]][1]]

        plt.figure(figsize=(7, 7))
        plt.plot(xs, ys, marker="o")
        plt.title(f"Best tour length = {best_overall_len}")
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    main()
