import numpy as np
import argparse
import os

def merge_segments(data_dir, trial_dir, train_ratio=0.7):
    """
    Merge segments
    """
    trial_path = os.path.join(data_dir, trial_dir)
    items_in_dir = os.listdir(trial_path)

    segments = []
    for fname in items_in_dir:
        if "segment" in fname:
            segments.append(fname)
    
    segments.sort(key=lambda x: int(x.replace(".", "_").split("_")[1]))  # segment_xxxx.npy  

    print(segments)

    segments_np = []
    for i, segment_fname in enumerate(segments):
        segment = np.load(os.path.join(trial_path, segment_fname))
        segments_np.append(segment)
        
    all_segments = np.concatenate(segments_np, axis=0)

    print("All segment shape: ", all_segments.shape)

    all_segments_fname = f"{trial_dir}_all.npy"

    print(all_segments.shape)

    np.save(os.path.join(trial_path, all_segments_fname), all_segments)


if __name__ == "__main__":
    # Parse command line arguments
    data_dir = "raw_data"
    parser = argparse.ArgumentParser(description='Merge segments')
    parser.add_argument('--trial_dir', type=str, help='Trial data dir')

    parser.parse_args()
    args = parser.parse_args()

    merge_segments(data_dir, args.trial_dir)

