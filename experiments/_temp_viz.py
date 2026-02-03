
import sys
sys.path.insert(0, "/home/abdullah/masteroppgave/revolve2/experiments")
from contact_detection import simulate_with_contact_detection

print("Visualizing: tripod, bounds=[-1,1], lambda=2.75, run=11")
simulate_with_contact_detection(
    robot_name="tripod",
    simulation_time=30,
    verbose=False,
    cpg_params=[0.36391420343075653, -0.14607602613429205, -0.9427110065783805, 0.00045809887987601096, 0.9999892484957438, 0.9262171265975657, -0.1843756827759364, -0.2905264379560524, 0.7030729654620399],
    headless=False,
    warmup_time=0.0,
    track_camera=True,
    cast_shadows=False,
)
