#!/usr/bin/env python3
"""YAML-driven launcher for LeRobot 0.5 XVLA full fine-tuning."""
import argparse, json, os, subprocess
from pathlib import Path
import yaml
def f(k,v): return f"--{k}={str(v).lower() if isinstance(v,bool) else v}"
ap=argparse.ArgumentParser(); ap.add_argument("--config",required=True); ap.add_argument("--dry-run",action="store_true"); ap.add_argument("extra",nargs="*"); a=ap.parse_args()
c=yaml.safe_load(Path(a.config).read_text(encoding="utf-8")); e,d,p,t,l=(c[x] for x in ("environment","dataset","xvla","training","logging"))
if not (Path(d["root"])/"meta/info.json").is_file(): raise RuntimeError(f"dataset unavailable: {d['root']}")
if Path(t["output_dir"]).exists() and not a.dry_run: raise RuntimeError(f"output exists: {t['output_dir']}")
args=[f("dataset.repo_id",d["repo_id"]),f("dataset.root",d["root"]),f("dataset.video_backend",d["video_backend"]),f("rename_map",json.dumps(d["rename_map"])),f("output_dir",t["output_dir"]),f("job_name",t["job_name"]),f("seed",t["seed"]),f("batch_size",t["batch_size"]),f("num_workers",t["num_workers"]),f("steps",t["steps"]),f("log_freq",t["log_freq"]),f("save_freq",t["save_freq"]),f("eval_freq",t["eval_freq"]),f("save_checkpoint",t["save_checkpoint"]),f("wandb.enable",l["wandb_enabled"]),f("wandb.project",l["wandb_project"]),f("policy.path",p["pretrained_policy_path"]),f("policy.device",p["device"]),f("policy.dtype",p["dtype"]),f("policy.action_mode",p["action_mode"]),f("policy.max_action_dim",p["max_action_dim"]),f("policy.chunk_size",p["chunk_size"]),f("policy.n_action_steps",p["n_action_steps"]),f("policy.num_image_views",p["num_image_views"]),f("policy.empty_cameras",p["empty_cameras"]),f("policy.freeze_vision_encoder",p["freeze_vision_encoder"]),f("policy.freeze_language_encoder",p["freeze_language_encoder"]),f("policy.train_policy_transformer",p["train_policy_transformer"]),f("policy.train_soft_prompts",p["train_soft_prompts"]),f("policy.optimizer_lr",t["learning_rate"]),f("policy.scheduler_warmup_steps",t["warmup_steps"]),f("policy.scheduler_decay_steps",t["decay_steps"]),f("policy.scheduler_decay_lr",t["decay_learning_rate"]),f("policy.push_to_hub",False)]+a.extra
cmd=[str(Path(e["python"]).with_name("lerobot-train"))]+args; env=os.environ.copy(); env.update(CUDA_VISIBLE_DEVICES=str(e["cuda_visible_devices"]),HF_HOME=str(e["hf_home"]))
print(" ".join(cmd),flush=True)
if not a.dry_run: subprocess.run(cmd,env=env,check=True)
