"""
Interactive Demonstration & Diagnostic Dashboard Generator.
Runs perception pipeline on off-road terrain scenarios, generates visual diagnostic
dashboards, and outputs ROS2 navigation payloads.
"""

import os
import sys
import argparse
import time
from typing import List

# Ensure repository root is on sys.path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from sih_perception.pipeline.pipeline import PerceptionPipeline
from sih_perception.pipeline.ros2_bridge import ROS2MessageBridge
from sih_perception.simulation.synthetic_scene import SyntheticSceneGenerator, ScenarioType
from sih_perception.visualization.visualizer import PerceptionVisualizer
from sih_perception.traversability.decision_engine import TraversabilityType


def run_demo(scenarios: List[ScenarioType], output_dir: str = "outputs") -> None:
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 80)
    print("  SIH PERCEPTION: TRACKED UGV TRAVERSABILITY & DEPTH LOCALIZATION DEMO  ")
    print("=" * 80)

    pipeline = PerceptionPipeline()
    sim = SyntheticSceneGenerator()
    viz = PerceptionVisualizer(panel_width=640, panel_height=480)

    for idx, scenario in enumerate(scenarios, 1):
        print(f"\n[{idx}/{len(scenarios)}] Processing Scenario: {scenario.value.upper()}")
        print("-" * 80)

        # 1. Generate synthetic sensor data
        rgb, depth = sim.generate_scenario(scenario)

        # 2. Process frame through full perception pipeline
        t0 = time.perf_counter()
        result = pipeline.process_frame(rgb, depth)
        proc_time = (time.perf_counter() - t0) * 1000.0

        # 3. Save multi-panel dashboard image
        out_img_path = os.path.join(output_dir, f"dashboard_{scenario.value}.png")
        viz.save_dashboard(result, out_img_path)
        print(f"  -> Saved Composite Visual Dashboard: {out_img_path}")

        # 4. Print telemetry and obstacle verdicts
        plane = result.ground_plane
        print(f"  -> Ground Plane: {plane.a:.3f}x + {plane.b:.3f}y + {plane.c:.3f}z + {plane.d:.3f} = 0 (Slope: {plane.slope_deg:.1f} deg)")
        print(f"  -> Pipeline Latency: {result.timing.total_ms:.1f} ms ({result.timing.fps:.1f} FPS)")
        print(f"  -> Detected Obstacles ({len(result.obstacles)} total):")

        for obs in result.obstacles:
            if obs.is_run_over_allowed:
                action = "[RUN-OVER ALLOWED]"
            elif obs.traversability_type == TraversabilityType.NEGATIVE_HAZARD:
                action = "[DITCH HAZARD - STOP/AVOID]"
            else:
                action = "[LETHAL OBSTACLE - STOP/AVOID]"

            print(f"     * ID #{obs.id:02d} | Range: {obs.radial_distance_m:5.2f}m | Azimuth: {obs.azimuth_deg:+5.1f} deg | H: {obs.height_m*100:4.1f}cm | Cost: {obs.severity_cost:3d} | {action} ({obs.semantic_name.split('(')[0].strip()})")

        # 5. Export ROS2 Sample Payload
        ros_occupancy = ROS2MessageBridge.create_occupancy_grid_msg(result)
        ros_detections = ROS2MessageBridge.create_detection_3d_array_msg(result)
        
        json_path = os.path.join(output_dir, f"ros2_detections_{scenario.value}.json")
        with open(json_path, "w") as f:
            f.write(ROS2MessageBridge.export_summary_json(result))
        print(f"  -> Exported ROS2 Diagnostic JSON: {json_path}")

    print("\n" + "=" * 80)
    print(f"Demo complete! All diagnostic outputs saved in '{output_dir}/'.")
    print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Tracked UGV Perception & Traversability Demo")
    parser.add_argument(
        "--scenario",
        type=str,
        default="all",
        choices=["all", "debris", "grass", "rock", "ditch", "slope"],
        help="Test scenario to evaluate"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs",
        help="Directory to save dashboard images and ROS2 JSON payloads"
    )
    args = parser.parse_args()

    scenario_map = {
        "debris": ScenarioType.FLAT_TRAIL_SMALL_DEBRIS,
        "grass": ScenarioType.TALL_GRASS_MEADOW,
        "rock": ScenarioType.LETHAL_ROCK_BOULDER,
        "ditch": ScenarioType.NEGATIVE_DITCH_TRENCH,
        "slope": ScenarioType.STEEP_INCLINE_SLOPE,
    }

    if args.scenario == "all":
        scenarios = list(scenario_map.values())
    else:
        scenarios = [scenario_map[args.scenario]]

    run_demo(scenarios, args.output_dir)


if __name__ == "__main__":
    main()
