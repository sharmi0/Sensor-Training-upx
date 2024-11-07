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
    
    # segments.sort()
    segments.sort(key=lambda x: int(x.replace(".", "_").split("_")[1]))  # segment_xxxx.npy  

    train_idx = int(len(segments) * 0.7)

    train_segments_np = []
    test_segments_np = []
    for i, segment_fname in enumerate(segments):
        segment = np.load(os.path.join(trial_path, segment_fname))
        if i<train_idx:
            train_segments_np.append(segment)
        else:
            test_segments_np.append(segment)
        
    train_segments = np.concatenate(train_segments_np, axis=0)
    test_segments = np.concatenate(test_segments_np, axis=0)

    print("Train segment shape: ", train_segments.shape)
    print("Test segment shape: ", test_segments.shape)

    train_segment_fname = f"{trial_dir}_train.npy"
    test_segment_fname = f"{trial_dir}_test.npy"

    np.save(os.path.join(trial_path, train_segment_fname), train_segments)
    np.save(os.path.join(trial_path, test_segment_fname), test_segments)

    key = np.load(os.path.join(trial_path, "key.npy"), allow_pickle=True)
    np.save(os.path.join(trial_path, f"{trial_dir}_key.npy"), key)


if __name__ == "__main__":
    # Parse command line arguments
    data_dir = "raw_data"
    parser = argparse.ArgumentParser(description='Merge segments')
    parser.add_argument('--trial_dir', type=str, help='Trial data dir')

    parser.parse_args()
    args = parser.parse_args()

    merge_segments(data_dir, args.trial_dir)

