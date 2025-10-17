import os, time, json
from pathlib import Path
import numpy as np

class RunLogger:
    def __init__(self, root='runs', tag=None):
        ts = time.strftime('%Y%m%d-%H%M%S')
        if tag: ts += '-' + tag
        self.root = Path(root) / ts
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / 'frames').mkdir(parents=True, exist_ok=True)

    def save_occ(self, occ_prob, step):
        import imageio.v2 as iio
        path = self.root / f'occ_{step:05d}.png'
        iio.imwrite(path, (occ_prob*255).astype('uint8'))
        np.save(self.root / f'occ_{step:05d}.npy', occ_prob)

    def save_pose(self, pose, est_pose, step):
        rec = {'step': int(step), 'true_pose': list(map(float, pose)), 'est_pose': list(map(float, est_pose))}
        with open(self.root / 'poses.jsonl', 'a') as f:
            f.write(json.dumps(rec) + '\n')

    def save_frame(self, fig, step):
        path = self.root / 'frames' / f'frame_{step:05d}.png'
        fig.savefig(path, dpi=120)

    def try_make_video(self, fps=10):
        # Requires imageio[ffmpeg]; if not present, we leave frames for manual stitching.
        try:
            import imageio.v2 as iio
            frames_dir = self.root / 'frames'
            files = sorted(frames_dir.glob('frame_*.png'))
            if not files:
                return None
            writer = iio.get_writer(self.root / 'demo.mp4', fps=fps)
            for fp in files:
                writer.append_data(iio.imread(fp))
            writer.close()
            return str(self.root / 'demo.mp4')
        except Exception as e:
            return None
