import argparse
import os
from typing import Any

import numpy as np


def summarize_array(name: str, arr: np.ndarray, sample_rows: int = 5) -> None:
    print(f"[Summary] {name}")
    print(f"  shape: {arr.shape}")
    print(f"  dtype: {arr.dtype}")
    try:
        print(f"  min/max: {np.nanmin(arr)} / {np.nanmax(arr)}")
        print(f"  mean/std: {np.nanmean(arr):.6f} / {np.nanstd(arr):.6f}")
    except Exception:
        pass

    # 尝试二维视角打印前几行
    try:
        flat2 = arr.reshape(arr.shape[0], -1) if arr.ndim > 1 else arr.reshape(-1, 1)
        rows = min(sample_rows, flat2.shape[0])
        print(f"  head({rows}) flattened:")
        print(flat2[:rows])
    except Exception:
        # 回退：打印前几个元素
        print("  head elements:")
        print(arr.flat[: min(sample_rows * 8, arr.size)])


def load_npy(path: str) -> Any:
    # 支持 .npy 或 .npz
    if path.endswith(".npz"):
        data = np.load(path)
        print(f"Loaded NPZ with keys: {list(data.keys())}")
        return data
    return np.load(path, allow_pickle=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect G1 motion data (.npy/.npz)")
    default_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "assets",
            "data",
            "Walk_1_Skeleton0_g1_29dof_rev_1_0_Tpose_new.npy",
        )
    )
    parser.add_argument("--data", type=str, default=default_path, help="Path to .npy/.npz file")
    parser.add_argument("--rows", type=int, default=5, help="How many rows to preview")
    args = parser.parse_args()

    path = args.data
    print(f"Loading: {path}")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"File not found: {path}")

    data = load_npy(path)

    # 如果是npz，遍历各数组；如果是npy，根据内容类型分别处理
    if isinstance(data, np.lib.npyio.NpzFile):
        for key in data.files:
            arr = data[key]
            summarize_array(key, arr, sample_rows=args.rows)
    elif isinstance(data, np.ndarray):
        arr = data
        # 如果是对象数组，尽量展开前几个元素看看结构
        if arr.dtype == object:
            print(f"Loaded object array with shape {arr.shape}. Dumping first {min(args.rows, arr.size)} elements:")
            for i, elem in enumerate(arr.flat[: min(args.rows, arr.size)]):
                print(f"[elem {i}] type={type(elem)}")
                if isinstance(elem, np.ndarray):
                    summarize_array(f"elem_{i}", elem, sample_rows=args.rows)
                else:
                    print(repr(elem))
        else:
            summarize_array("array", arr, sample_rows=args.rows)
    else:
        # 其他类型（很少见）
        print(f"Loaded object of type {type(data)}")
        try:
            summarize_array("data", np.asarray(data), sample_rows=args.rows)
        except Exception:
            print(repr(data))


if __name__ == "__main__":
    main()


