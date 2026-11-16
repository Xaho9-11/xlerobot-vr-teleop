"""Dataset preview for the training dashboard (ACT feasibility test, 2026-11-10): a few camera frames and the
joint trajectories of one episode, written to ~/fmp_train/preview/. Usage: make_preview.py REPO_ID [EPISODE]"""
import json, os, sys
import numpy as np
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset

OUT = os.path.expanduser("~/fmp_train/preview")


def main():
    repo = sys.argv[1]
    epi = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    os.makedirs(OUT, exist_ok=True)
    ds = LeRobotDataset(repo)
    ep = ds.meta.episodes[epi]
    a, b = ep["dataset_from_index"], ep["dataset_to_index"]
    cams = [k for k, v in ds.meta.features.items() if v.get("dtype") in ("video", "image")]
    picks = [a + int(f * (b - a - 1)) for f in (0.0, 0.25, 0.5, 0.75, 1.0)]
    frames = []
    for n, i in enumerate(picks):
        item = ds[i]
        row = {"t": round(float(item["timestamp"]), 2), "imgs": {}}
        for c in cams:
            img = (item[c].permute(1, 2, 0).numpy() * 255).clip(0, 255).astype(np.uint8)
            name = f"{c.split('.')[-1]}_{n}.jpg"
            Image.fromarray(img).resize((320, 240)).save(os.path.join(OUT, name), quality=85)
            row["imgs"][c.split(".")[-1]] = name
        frames.append(row)
    rows = ds.hf_dataset.select(range(a, b))
    names = ds.meta.features["action"].get("names") or [f"j{k}" for k in range(6)]
    state = np.array(rows["observation.state"]).tolist()
    action = np.array(rows["action"]).tolist()
    info = dict(repo=repo, episode=epi, fps=ds.meta.fps, length=b - a, episodes=ds.meta.total_episodes,
                frames_total=ds.meta.total_frames, task=str(ds.meta.tasks.index[0]), cams=[c.split(".")[-1] for c in cams],
                joint_names=[n.replace("main_", "") for n in names], frames=frames, state=state, action=action)
    json.dump(info, open(os.path.join(OUT, "preview.json"), "w"))
    print("preview written:", OUT, len(frames), "frames,", b - a, "steps")


if __name__ == "__main__":
    main()
