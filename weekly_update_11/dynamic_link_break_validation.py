#!/usr/bin/env python3
"""
================================================================================
  dynamic_link_break_validation.py
  --------------------------------
  Standalone validation utility: simulates dynamic link breakage on a graph CSV.

  Purpose:
    Given a graph (edge-list CSV) and a break-schedule CSV, this script
    deterministically breaks links at specified iterations and produces an
    output CSV with all original columns plus a `broken_iteration` column.

  Author:  Auto-generated for academic validation
  Date:    2026-03-31
================================================================================

USAGE:
  python dynamic_link_break_validation.py \
      --graph   map_data.csv \
      --schedule break_schedule.csv \
      --output  graph_after_breaks.csv \
      --seed    42

ARGUMENTS:
  --graph      Path to the input graph CSV (one row per edge).
  --schedule   Path to the break-schedule CSV (columns: iteration, number).
  --output     Path for the output CSV (default: output_broken_graph.csv).
  --seed       Random seed for reproducibility (default: 42).

BREAK-SCHEDULE FORMAT (example):
  iteration,number
  1,2
  3,3
  5,1

  This means:
    - At iteration 1, break 2 links (chosen randomly from unbroken links).
    - At iteration 3, break 3 more links.
    - At iteration 5, break 1 more link.
  Once a link is broken, it stays broken for all subsequent iterations.

OUTPUT:
  A new CSV file containing every original column plus:
    broken_iteration  — the iteration at which the link was broken,
                        or -1 if the link was never broken.

EXAMPLE (inline):
  Suppose map_data.csv has 30 edges and break_schedule.csv says:
    iteration,number
    1,3
    4,2

  Running:
    python dynamic_link_break_validation.py \
        --graph map_data.csv --schedule break_schedule.csv

  Will produce output_broken_graph.csv where:
    - 3 edges have broken_iteration = 1
    - 2 edges have broken_iteration = 4
    - The remaining 25 edges have broken_iteration = -1
================================================================================
"""

import csv
import random
import argparse
import sys
import os


# ---------------------------------------------------------------------------
# Helper: read a CSV file into a list of OrderedDict-like rows
# ---------------------------------------------------------------------------
def read_csv(filepath):
    """Read a CSV file and return (header_list, list_of_row_dicts)."""
    if not os.path.isfile(filepath):
        print(f"ERROR: File not found: {filepath}", file=sys.stderr)
        sys.exit(1)

    with open(filepath, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        header = reader.fieldnames
        rows = list(reader)

    if not header or len(rows) == 0:
        print(f"ERROR: '{filepath}' is empty or has no data rows.", file=sys.stderr)
        sys.exit(1)

    return header, rows


# ---------------------------------------------------------------------------
# Helper: write rows to a CSV file
# ---------------------------------------------------------------------------
def write_csv(filepath, header, rows):
    """Write a list of dicts to a CSV file with the given header order."""
    with open(filepath, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Output written to: {filepath}")


# ---------------------------------------------------------------------------
# Core logic: apply the break schedule to the graph edges
# ---------------------------------------------------------------------------
def apply_break_schedule(graph_rows, schedule_rows, seed):
    """
    For each entry in the break schedule, randomly choose `number` links
    from the currently-unbroken set and mark them as broken at `iteration`.

    Parameters
    ----------
    graph_rows : list[dict]   — rows from the graph CSV
    schedule_rows : list[dict] — rows from the break-schedule CSV
    seed : int                 — random seed for reproducibility

    Returns
    -------
    graph_rows (modified in place) with a new key 'broken_iteration' added
    to every row.
    """
    rng = random.Random(seed)  # isolated RNG instance — does not affect global state

    # Initialise every edge as unbroken (broken_iteration = -1)
    for row in graph_rows:
        row["broken_iteration"] = -1

    # Build list of edge indices that are still unbroken
    unbroken_indices = list(range(len(graph_rows)))

    # Sort the schedule by iteration so we process them in order
    schedule_sorted = sorted(schedule_rows, key=lambda r: int(r["iteration"]))

    total_broken = 0

    for entry in schedule_sorted:
        iteration = int(entry["iteration"])
        number = int(entry["number"])

        # Validate: can't break more links than remain
        if number > len(unbroken_indices):
            print(
                f"WARNING: Iteration {iteration} requests {number} breaks, "
                f"but only {len(unbroken_indices)} unbroken links remain. "
                f"Breaking all remaining links.",
                file=sys.stderr,
            )
            number = len(unbroken_indices)

        if number <= 0:
            continue

        # Randomly choose which links to break (deterministic via seed)
        chosen = rng.sample(unbroken_indices, number)

        for idx in chosen:
            graph_rows[idx]["broken_iteration"] = iteration
            unbroken_indices.remove(idx)

        total_broken += number
        print(
            f"  Iteration {iteration}: broke {number} link(s)  "
            f"[total broken so far: {total_broken}/{len(graph_rows)}]"
        )

    # Summary
    still_unbroken = len(graph_rows) - total_broken
    print(f"\nSummary: {total_broken} link(s) broken, {still_unbroken} link(s) intact.")

    return graph_rows


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Simulate dynamic link breakage on a graph CSV for validation.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
  python dynamic_link_break_validation.py \\
      --graph map_data.csv \\
      --schedule break_schedule.csv \\
      --output graph_after_breaks.csv \\
      --seed 42
        """,
    )
    parser.add_argument(
        "--graph",
        required=True,
        help="Path to the input graph CSV (edge list).",
    )
    parser.add_argument(
        "--schedule",
        required=True,
        help="Path to the break-schedule CSV (columns: iteration, number).",
    )
    parser.add_argument(
        "--output",
        default="output_broken_graph.csv",
        help="Path for the output CSV (default: output_broken_graph.csv).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility (default: 42).",
    )
    args = parser.parse_args()

    # --- 1. Read the graph ----
    print(f"Reading graph from:    {args.graph}")
    graph_header, graph_rows = read_csv(args.graph)
    print(f"  -> {len(graph_rows)} edges loaded.")

    # --- 2. Read the break schedule ---
    print(f"Reading schedule from: {args.schedule}")
    sched_header, schedule_rows = read_csv(args.schedule)

    # Quick validation of schedule columns
    required_cols = {"iteration", "number"}
    if not required_cols.issubset(set(sched_header)):
        print(
            f"ERROR: Break-schedule CSV must have columns {required_cols}. "
            f"Found: {sched_header}",
            file=sys.stderr,
        )
        sys.exit(1)

    total_requested = sum(int(r["number"]) for r in schedule_rows)
    print(
        f"  -> {len(schedule_rows)} schedule entries, "
        f"{total_requested} total break(s) requested."
    )

    # --- 3. Apply the break schedule ---
    print(f"\nApplying break schedule (seed={args.seed}):")
    apply_break_schedule(graph_rows, schedule_rows, args.seed)

    # --- 4. Write the output CSV ---
    output_header = graph_header + ["broken_iteration"]
    write_csv(args.output, output_header, graph_rows)

    print("\nDone.")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    main()
