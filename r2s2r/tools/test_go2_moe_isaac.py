"""Bounded Isaac Sim smoke/command tests, with no ROS/TCP or sensor startup."""
import argparse,json,math,time
parser=argparse.ArgumentParser()
parser.add_argument("--cases",default="stand,forward,yaw_pos,yaw_neg")
parser.add_argument("--duration",type=float,default=30)
parser.add_argument("--no-pi",action="store_true")
parser.add_argument("--stairs",action="store_true")
parser.add_argument("--output",help="Optional CSV prefix for recorded body state")
args=parser.parse_args()

from isaacsim import SimulationApp
app=SimulationApp({"headless":True,"width":640,"height":480})
import sys
sys.path.insert(0,"/workspace")
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, GroundPlane
from isaacsim.core.api.materials import PhysicsMaterial
from r2s2r.core.go2_moe_policy import Go2MoeCTSPolicy
from r2s2r.core.go2_moe_runtime import PHYSICS_DT,rotation_matrix

world=World(stage_units_in_meters=1.,physics_dt=PHYSICS_DT,rendering_dt=.02)
material=PhysicsMaterial("/World/GroundMaterial",static_friction=1.,dynamic_friction=1.,restitution=0.)
world.scene.add(GroundPlane("/World/Ground",size=100.,physics_material=material))
if args.stairs:
    for i in range(10):
        world.scene.add(FixedCuboid(prim_path=f"/World/Step{i}",name=f"step{i}",
            position=np.array([2.15+.3*i,0,.095+.19*i]),
            scale=np.array([.3,6.,.19]),color=np.array([.7,.7,.7]),physics_material=material))
    world.scene.add(FixedCuboid(prim_path="/World/Ramp",name="ramp",
        position=np.array([7.0465,0,.95]),orientation=np.array([math.cos(.2),0,math.sin(.2),0]),
        scale=np.array([4.493,6.,.2]),physics_material=material))
robot=Go2MoeCTSPolicy("/World/Go2",position=np.array([0,0,.5]),pi_enabled=not args.no_pi)
world.reset()
world.play()
commands={"stand":[0,0,0],"forward":[.4,0,0],"yaw_pos":[0,0,.4],"yaw_neg":[0,0,-.4]}
results=[]
try:
    for case in args.cases.split(","):
        robot.robot.set_world_pose(np.array([0,0,.5]),np.array([1.,0,0,0]))
        robot.initialize()
        samples=[];start=time.monotonic();fall=False;next_progress=2.
        total=args.duration+3.
        for step in range(round(total/PHYSICS_DT)):
            elapsed=step*PHYSICS_DT
            command=np.array(commands[case] if elapsed>=3 else [0,0,0])
            robot.forward(PHYSICS_DT,command)
            world.step(render=False)
            if step%10==0:
                position,quat=robot.robot.get_world_pose()
                R=rotation_matrix(quat)
                v=R.T@robot.robot.get_linear_velocity()
                w=R.T@robot.robot.get_angular_velocity()
                if elapsed>=3:
                    sample=[elapsed-3,*position,*v,w[2],R[2,2]]
                    samples.append(sample)
                    if position[2]<.12 or R[2,2]<.5:fall=True
                if elapsed>=next_progress:
                    print("GO2_MOE_PROGRESS",case,round(elapsed,2),"pos",position.tolist(),"up",float(R[2,2]),flush=True)
                    next_progress+=5
        samples=np.asarray(samples)
        if args.output:
            np.savetxt(args.output+"_"+case+".csv",samples,delimiter=",",header="time,x,y,z,vx,vy,vz,wz,up")
        scored=samples[samples[:,0]>=max(0,args.duration-15)]
        result=dict(case=case,pi=not args.no_pi,duration=args.duration,wall_s=time.monotonic()-start,
            mean_vx=float(scored[:,4].mean()),mean_vy=float(scored[:,5].mean()),
            mean_wz=float(scored[:,7].mean()),vx_mae=float(np.abs(scored[:,4]-commands[case][0]).mean()),
            wz_mae=float(np.abs(scored[:,7]-commands[case][2]).mean()),min_z=float(scored[:,3].min()),
            max_tilt_deg=float(np.degrees(np.arccos(np.clip(scored[:,8],-1,1))).max()),
            max_abs_y=float(np.abs(samples[:,2]).max()),max_x=float(samples[:,1].max()),
            max_z=float(samples[:,3].max()),final_position=samples[-1,1:4].tolist(),fall_detected=fall)
        results.append(result)
        print("GO2_MOE_CASE",json.dumps(result),flush=True)
    print("GO2_MOE_RESULT",json.dumps(results),flush=True)
finally:
    world.stop()
    app.close()
