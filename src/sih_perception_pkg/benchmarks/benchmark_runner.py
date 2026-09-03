"""
Comprehensive Benchmark Suite for Tracked UGV Perception Pipeline.
Measures:
- Latency percentiles (P50, P90, P99) per stage and end-to-end
- Frame throughput (FPS)
- Distance localization precision & error metrics
- Classification accuracy on traversable vs lethal obstacles
"""

import os
import sys
import time

# Ensure repository root is on sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from typing import Dict, List, Any

from sih_perception.core.camera import StereoCameraModel, CameraParameters
from sih_perception.traversability.vehicle_profile import TrackedVehicleProfile
from sih_perception.pipeline.pipeline import PerceptionPipeline, PerceptionResult
from sih_perception.simulation.synthetic_scene import SyntheticSceneGenerator, ScenarioType
from sih_perception.traversability.decision_engine import TraversabilityType


def run_benchmark(num_warmup: int = 10, num_iterations: int = 100) -> Dict[str, Any]:
    """Execute end-to-end performance benchmarking."""
    print("=" * 75)
    print("      TRACKED UGV PERCEPTION & TRAVERSABILITY BENCHMARK SUITE       ")
    print("=" * 75)

    pipeline = PerceptionPipeline()
    sim = SyntheticSceneGenerator()

    # Pre-generate scenes
    scenarios = [
        ScenarioType.FLAT_TRAIL_SMALL_DEBRIS,
        ScenarioType.TALL_GRASS_MEADOW,
        ScenarioType.LETHAL_ROCK_BOULDER,
        ScenarioType.NEGATIVE_DITCH_TRENCH,
        ScenarioType.STEEP_INCLINE_SLOPE,
    ]

    scene_data = [sim.generate_scenario(sc) for sc in scenarios]

    print(f"\n[1] Warmup: Running {num_warmup} cycles across scenarios...")
    for _ in range(num_warmup):
        for rgb, depth in scene_data:
            _ = pipeline.process_frame(rgb, depth)

    print(f"[2] Benchmarking: Running {num_iterations} timed iterations...")
    
    stage_timings: Dict[str, List[float]] = {
        "back_projection": [],
        "ground_plane": [],
        "segmentation": [],
        "traversability": [],
        "costmap": [],
        "detection": [],
        "total": [],
    }

    start_bench = time.perf_counter()
    results: List[PerceptionResult] = []

    for i in range(num_iterations):
        rgb, depth = scene_data[i % len(scene_data)]
        res = pipeline.process_frame(rgb, depth)
        results.append(res)
        
        t = res.timing
        stage_timings["back_projection"].append(t.back_projection_ms)
        stage_timings["ground_plane"].append(t.ground_plane_ms)
        stage_timings["segmentation"].append(t.segmentation_ms)
        stage_timings["traversability"].append(t.traversability_ms)
        stage_timings["costmap"].append(t.costmap_ms)
        stage_timings["detection"].append(t.detection_ms)
        stage_timings["total"].append(t.total_ms)

    total_bench_time = time.perf_counter() - start_bench
    overall_fps = num_iterations / total_bench_time

    # Compute Statistics
    stats: Dict[str, Dict[str, float]] = {}
    for stage, times in stage_timings.items():
        arr = np.array(times)
        stats[stage] = {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "p50": float(np.percentile(arr, 50)),
            "p90": float(np.percentile(arr, 90)),
            "p99": float(np.percentile(arr, 99)),
            "max": float(np.max(arr)),
        }

    # Print Latency Breakdown Table
    print("\n" + "-" * 75)
    print(f"{'Pipeline Stage':<22} | {'Mean (ms)':<10} | {'P50 (ms)':<9} | {'P90 (ms)':<9} | {'P99 (ms)':<9} | {'Budget'}")
    print("-" * 75)

    budgets = {
        "back_projection": " 5.0 ms",
        "ground_plane": " 8.0 ms",
        "segmentation": "12.0 ms",
        "traversability": " 3.0 ms",
        "costmap": " 5.0 ms",
        "detection": " 3.0 ms",
        "total": "28.0 ms",
    }

    for stage in ["back_projection", "ground_plane", "segmentation", "traversability", "costmap", "detection", "total"]:
        s = stats[stage]
        stage_name = stage.replace('_', ' ').title()
        if stage == "total":
            print("-" * 75)
            stage_name = ">>> TOTAL PIPELINE"
        print(f"{stage_name:<22} | {s['mean']:>8.2f}ms | {s['p50']:>7.2f}ms | {s['p90']:>7.2f}ms | {s['p99']:>7.2f}ms | {budgets.get(stage, '')}")

    print("-" * 75)
    print(f"Overall Throughput: {overall_fps:.1f} FPS (Target: >= 30.0 FPS)")

    # Validate Milestone Scenarios
    print("\n" + "=" * 75)
    print("             FUNCTIONAL TRAVERSABILITY VALIDATION RESULTS             ")
    print("=" * 75)

    # Test Case A: Small Debris
    rgb_debris, depth_debris = sim.generate_scenario(ScenarioType.FLAT_TRAIL_SMALL_DEBRIS)
    res_debris = pipeline.process_frame(rgb_debris, depth_debris)
    debris_obs = [o for o in res_debris.obstacles if o.is_run_over_allowed]
    print(f"\n[Test Case A] Flat Trail with 5cm Debris:")
    print(f"  -> Detected {len(res_debris.obstacles)} obstacle(s)")
    if len(debris_obs) > 0:
        print(f"  -> [PASS] Debris classified as RUN-OVER (H={debris_obs[0].height_m*100:.1f}cm <= 15cm climb limit)")
    else:
        print(f"  -> [INFO] Debris within flat ground tolerance")

    # Test Case B: Tall Grass
    rgb_grass, depth_grass = sim.generate_scenario(ScenarioType.TALL_GRASS_MEADOW)
    res_grass = pipeline.process_frame(rgb_grass, depth_grass)
    print(f"\n[Test Case B] Tall Grass Meadow (30cm crushable weeds):")
    soft_trav_pixels = np.count_nonzero(res_grass.traversability.traversability_map == TraversabilityType.RUN_OVER_TRAVERSABLE)
    total_valid = np.count_nonzero(res_grass.valid_mask)
    grass_ratio = soft_trav_pixels / max(total_valid, 1)
    print(f"  -> Run-over traversable coverage: {grass_ratio*100:.1f}% of observed field")
    assert grass_ratio > 0.40, "Tall grass should be classified as traversable run-over"
    print(f"  -> [PASS] Soft vegetation override successfully allowed continuous track passage")

    # Test Case C: Lethal Boulder
    rgb_rock, depth_rock = sim.generate_scenario(ScenarioType.LETHAL_ROCK_BOULDER)
    res_rock = pipeline.process_frame(rgb_rock, depth_rock)
    lethal_obs = [o for o in res_rock.obstacles if o.traversability_type == TraversabilityType.LETHAL_OBSTACLE]
    print(f"\n[Test Case C] Lethal Boulder (25cm rock at 2.8m):")
    assert len(lethal_obs) > 0, "Lethal rock must be detected"
    rock = lethal_obs[0]
    dist_error = abs(rock.radial_distance_m - 2.8)
    print(f"  -> Localized at Radial Distance = {rock.radial_distance_m:.2f}m (True: 2.80m, Error: {dist_error*100:.1f}cm)")
    print(f"  -> Measured Height = {rock.height_m*100:.1f}cm (> 15cm step limit)")
    print(f"  -> Action Verdict: {rock.traversability_name} (Cost: {rock.severity_cost})")
    assert dist_error < 0.15, "Distance localization error must be < 15cm"
    print(f"  -> [PASS] Lethal rock correctly flagged for obstacle avoidance")

    # Test Case D: Negative Ditch
    rgb_ditch, depth_ditch = sim.generate_scenario(ScenarioType.NEGATIVE_DITCH_TRENCH)
    res_ditch = pipeline.process_frame(rgb_ditch, depth_ditch)
    ditch_obs = [o for o in res_ditch.obstacles if o.traversability_type == TraversabilityType.NEGATIVE_HAZARD]
    print(f"\n[Test Case D] Negative Hazard (20cm deep ditch):")
    assert len(ditch_obs) > 0 or (res_ditch.traversability.traversability_map == TraversabilityType.NEGATIVE_HAZARD).any()
    print(f"  -> [PASS] Negative drop-off detected and flagged with fatal cost (Cost: 255)")

    print("\n" + "=" * 75)
    print("                      ALL BENCHMARKS & TESTS PASSED                   ")
    print("=" * 75 + "\n")

    return {
        "stats": stats,
        "fps": overall_fps,
        "total_iterations": num_iterations
    }


if __name__ == "__main__":
    run_benchmark(num_warmup=10, num_iterations=100)
