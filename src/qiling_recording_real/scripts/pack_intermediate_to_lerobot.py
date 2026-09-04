#!/usr/bin/env python3
"""Python 3.12 / so101_lerobot stage: pack intermediate files as LeRobot v3."""
import argparse, json
from pathlib import Path
import cv2, numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

JOINTS=["right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"]
def vec(n,names): return {"dtype":"float32","shape":(n,),"names":names}
def image(): return {"dtype":"video","shape":(3,480,640),"names":["channels","height","width"]}
def load_rgb(p):
    x=cv2.imread(str(p),cv2.IMREAD_COLOR)
    if x is None: raise RuntimeError(f"cannot decode {p}")
    return cv2.cvtColor(x,cv2.COLOR_BGR2RGB)
def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument("intermediate_root"); ap.add_argument("--output-root",required=True); ap.add_argument("--repo-id",default="local/qiling_right_arm_o6"); ap.add_argument("--fps",type=float,default=30.0); a=ap.parse_args()
    inp=Path(a.intermediate_root); out=Path(a.output_root)
    if out.exists(): raise RuntimeError(f"output exists: {out}")
    entries=json.loads((inp/"index.json").read_text(encoding="utf-8"))["episodes"]
    features={"observation.images.head":image(),"observation.images.left":image(),"observation.images.right":image(),"observation.state":vec(7,JOINTS),"observation.velocity":vec(7,["d_"+x for x in JOINTS]),"action":vec(8,JOINTS+["right_o6_closed"])}
    ds=LeRobotDataset.create(repo_id=a.repo_id,fps=a.fps,features=features,root=out,robot_type="qiling_right_arm_o6",use_videos=True,image_writer_processes=0,image_writer_threads=4)
    try:
        for entry in entries:
            folder=inp/entry["folder"]; data=np.load(folder/"data.npz"); count=len(data["q"])
            for i in range(count):
                ds.add_frame({"observation.images.head":load_rgb(folder/"images"/f"{i:06d}_head.jpg"),"observation.images.left":load_rgb(folder/"images"/f"{i:06d}_left.jpg"),"observation.images.right":load_rgb(folder/"images"/f"{i:06d}_right.jpg"),"observation.state":data["q"][i],"observation.velocity":data["dq"][i],"action":np.concatenate((data["q_target"][i],[data["gripper"][i]])).astype(np.float32),"task":entry["task"]})
            ds.save_episode(parallel_encoding=False); print(f"Packed {entry['segment_id']}: {count} frames",flush=True)
    finally: ds.finalize()
    print(f"LeRobot dataset written to {out}")
if __name__=="__main__": main()
