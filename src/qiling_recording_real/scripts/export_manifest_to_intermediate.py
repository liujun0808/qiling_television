#!/usr/bin/env python3
"""Python 3.10 / ROS Humble stage: export manifest-selected MCAP frames.

The output contains JPEG bytes plus q, dq, q_target and O6 labels.  It is the
only conversion stage that imports rosbag2_py; the next stage runs in Conda.
"""
import argparse, bisect, json, shutil
from pathlib import Path
import cv2, numpy as np, yaml
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

def read_yaml(p): return yaml.safe_load(Path(p).read_text(encoding="utf-8")) or {}
def near(stream, times, t, tol):
    i=bisect.bisect_left(times,t); choices=[x for x in (i-1,i) if 0<=x<len(times)]
    if not choices: return None
    j=min(choices,key=lambda x:abs(times[x]-t)); return stream[j][1] if abs(times[j]-t)<=tol else None
def streams(bag, topics):
    r=SequentialReader(); r.open(StorageOptions(uri=str(bag),storage_id="mcap"),ConverterOptions(input_serialization_format="cdr",output_serialization_format="cdr"))
    tm={x.name:x.type for x in r.get_all_topics_and_types()}; mt={t:get_message(tm[t]) for t in topics}
    out={t:[] for t in topics}
    while r.has_next():
        t,raw,stamp=r.read_next()
        if t in mt: out[t].append((int(stamp),deserialize_message(raw,mt[t])))
    return out, {t:[x[0] for x in v] for t,v in out.items()}
def main():
    ap=argparse.ArgumentParser(description=__doc__); ap.add_argument("manifest"); ap.add_argument("--output-root",required=True); ap.add_argument("--camera-tolerance-sec",type=float,default=.025); ap.add_argument("--state-tolerance-sec",type=float,default=.015); a=ap.parse_args()
    m=json.loads(Path(a.manifest).read_text(encoding="utf-8")); root=Path(a.output_root)
    if root.exists(): raise RuntimeError(f"output exists: {root}")
    root.mkdir(parents=True); shutil.copy2(a.manifest,root/"source_manifest.json")
    index=[]; ct,st=int(a.camera_tolerance_sec*1e9),int(a.state_tolerance_sec*1e9)
    for n,seg in enumerate(m["training_segments"]):
        session=read_yaml(Path(seg["source_path"])/"session.yaml"); d=session["derived_topics"]; c=session["cameras"]
        t={"head":c["head"]["recorded_topic"],"left":c["left"]["recorded_topic"],"right":c["right"]["recorded_topic"],"obs":d["observation_joint_state_topic"],"act":d["action_joint_position_topic"],"grip":d["right_gripper_action_topic"]}
        ss,ts=streams(Path(seg["source_path"])/"rosbag",list(t.values())); selected=[]
        for stamp,head in ss[t["head"]]:
            if not (seg["start_time_ns"]<=stamp<=seg["end_time_ns"]): continue
            vals=[near(ss[t[k]],ts[t[k]],stamp,ct if k in ("left","right") else st) for k in ("left","right","obs","act","grip")]
            if any(x is None for x in vals): continue
            selected.append((head,*vals))
        if not selected: continue
        folder=root/f"episode_{len(index):05d}"; img=folder/"images"; img.mkdir(parents=True)
        q=[]; dq=[]; act=[]; grip=[]
        for i,(head,left,right,obs,cmd,g) in enumerate(selected):
            for name,msg in (("head",head),("left",left),("right",right)):
                (img/f"{i:06d}_{name}.jpg").write_bytes(bytes(msg.data))
            q.append(obs.position); dq.append(obs.velocity); act.append(cmd.position); grip.append(int(g.data))
        np.savez_compressed(folder/"data.npz",q=np.asarray(q,np.float32),dq=np.asarray(dq,np.float32),q_target=np.asarray(act,np.float32),gripper=np.asarray(grip,np.float32))
        info={k:seg[k] for k in ("segment_id","source_episode","task","fps","start_time_ns","end_time_ns")}; info["frames"]=len(selected)
        (folder/"segment.json").write_text(json.dumps(info,ensure_ascii=False,indent=2)+"\n",encoding="utf-8"); index.append({"folder":folder.name,**info}); print(f"Exported {info['segment_id']}: {len(selected)} frames",flush=True)
    (root/"index.json").write_text(json.dumps({"format":"qiling_intermediate_v1","episodes":index},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(f"Exported {len(index)} episodes to {root}")
if __name__=="__main__": main()
