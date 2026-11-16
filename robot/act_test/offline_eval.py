"""Offline check of a trained ACT checkpoint (ACT feasibility test, 2026-11-10): from a few frames of recorded
episodes, predict one action chunk and compare it with the recorded actions. Writes runs/NAME/eval.json for the
dashboard (one chunk to plot + mean absolute error over all tried starts). Note: unless the run excluded
episodes, these are TRAINING episodes - it shows the pipeline works and fits, not generalisation.
Usage: offline_eval.py RUN_NAME [--episodes 0 10 20] [--offsets 0.2 0.5]"""
import argparse, json, os
import numpy as np
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies import make_pre_post_processors
from lerobot.policies.act.modeling_act import ACTPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--episodes", type=int, nargs="+", default=None, help="default: up to 5 spread over the dataset")
    ap.add_argument("--offsets", type=float, nargs="+", default=[0.2, 0.5])
    a = ap.parse_args()
    d = os.path.expanduser(f"~/fmp_train/runs/{a.run}")
    meta = json.load(open(os.path.join(d, "meta.json")))
    pp = os.path.join(d, "out", "checkpoints", "last", "pretrained_model")
    policy = ACTPolicy.from_pretrained(pp).to("cuda").eval()
    pre, post = make_pre_post_processors(policy_cfg=policy.config, pretrained_path=pp,
                                         preprocessor_overrides={"device_processor": {"device": "cuda"}})
    ds = LeRobotDataset(meta["repo"], root=meta.get("root") or None)
    names = [n.replace("main_", "") for n in (ds.meta.features["action"].get("names") or [])]
    task = str(ds.meta.tasks.index[0])
    if a.episodes is None:
        n_ep = ds.meta.total_episodes
        a.episodes = sorted({round(k * (n_ep - 1) / 4) for k in range(5)}) if n_ep > 1 else [0]
    errs, plot = [], None
    for epi in a.episodes:
        ep = ds.meta.episodes[epi]
        lo, hi = ep["dataset_from_index"], ep["dataset_to_index"]
        for off in a.offsets:
            s = lo + int(off * (hi - lo))
            n = min(policy.config.chunk_size, hi - s)
            item = ds[s]
            batch = {k: (v.unsqueeze(0) if torch.is_tensor(v) else v) for k, v in item.items()}
            batch["task"] = [task]
            with torch.no_grad():
                chunk = policy.predict_action_chunk(pre(batch))[0]  # (chunk, action_dim), normalised
                pred = post(chunk).cpu().numpy()[:n]
            true = np.array(ds.hf_dataset.select(range(s, s + n))["action"])
            err = float(np.abs(pred - true).mean())
            errs.append(err)
            print(f"episode {epi} frame {s - lo}: mean abs error {err:.2f} deg over {n} steps")
            if plot is None:
                plot = dict(episode=epi, start=s - lo, fps=ds.meta.fps, names=names, pred=pred.tolist(),
                            true=true.tolist(), mae_this=err)
    plot["mae"] = float(np.mean(errs))
    plot["n_checks"] = len(errs)
    json.dump(plot, open(os.path.join(d, "eval.json"), "w"))
    print(f"overall mean abs error {plot['mae']:.2f} deg over {len(errs)} chunks -> eval.json")


if __name__ == "__main__":
    main()
