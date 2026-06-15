#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np


DEFAULT_DICT_SIZE = 57961
DEFAULT_MARKER_BITS = 6
DEFAULT_CACHE_DIR = Path("/home/rc3/tmp")


def parse_args():
    parser = argparse.ArgumentParser(description="NUC sender: show a custom ArUco tag.")
    parser.add_argument("--id", type=int, default=None, help="Marker ID to display.")
    parser.add_argument(
        "--dict-size",
        type=int,
        default=DEFAULT_DICT_SIZE,
        help="Number of IDs in the dictionary. Default covers 0..57960.",
    )
    parser.add_argument("--marker-bits", type=int, default=DEFAULT_MARKER_BITS)
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
    parser.add_argument("--display-size", type=int, default=700)
    parser.add_argument("--border-px", type=int, default=80)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--force-generate", action="store_true")
    return parser.parse_args()


def prompt_id(dict_size):
    while True:
        raw = input(f"Input marker id [0-{dict_size - 1}]: ").strip()
        try:
            marker_id = int(raw)
        except ValueError:
            print("Please input an integer.")
            continue
        if 0 <= marker_id < dict_size:
            return marker_id
        print(f"ID out of range. Valid range is 0..{dict_size - 1}.")


def cache_path(cache_dir, dict_size, marker_bits):
    return cache_dir / f"aruco_custom_{marker_bits}x{marker_bits}_{dict_size}.npz"


def load_dictionary(path):
    data = np.load(path)
    dictionary = cv2.aruco.Dictionary()
    dictionary.bytesList = data["bytesList"]
    dictionary.markerSize = int(data["markerSize"])
    dictionary.maxCorrectionBits = int(data["maxCorrectionBits"])
    return dictionary


def save_dictionary(path, dictionary):
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        bytesList=dictionary.bytesList,
        markerSize=np.array(dictionary.markerSize),
        maxCorrectionBits=np.array(dictionary.maxCorrectionBits),
    )


def make_or_load_dictionary(dict_size, marker_bits, cache_dir, force_generate):
    path = cache_path(cache_dir, dict_size, marker_bits)
    if path.exists() and not force_generate:
        print(f"Loading dictionary cache: {path}", flush=True)
        return load_dictionary(path), path

    print(
        "Generating custom ArUco dictionary "
        f"({dict_size} markers, {marker_bits}x{marker_bits}).",
        flush=True,
    )
    print("Generate this once, then copy the .npz file to the Windows PC.", flush=True)
    dictionary = cv2.aruco.extendDictionary(dict_size, marker_bits, cv2.aruco.Dictionary(), 0)
    save_dictionary(path, dictionary)
    print(f"Saved dictionary cache: {path}", flush=True)
    return dictionary, path


def make_marker_image(dictionary, marker_id, display_size, border_px):
    marker = np.full((display_size, display_size), 255, dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, display_size, marker, 1)
    canvas_size = display_size + border_px * 2
    canvas = np.full((canvas_size, canvas_size, 3), 255, dtype=np.uint8)
    canvas[border_px : border_px + display_size, border_px : border_px + display_size] = (
        cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    )
    return canvas


def main():
    args = parse_args()
    marker_id = args.id if args.id is not None else prompt_id(args.dict_size)
    if marker_id < 0 or marker_id >= args.dict_size:
        raise ValueError(f"--id must be in range 0..{args.dict_size - 1}.")

    dictionary, dict_path = make_or_load_dictionary(
        args.dict_size,
        args.marker_bits,
        args.cache_dir,
        args.force_generate,
    )
    print(f"Copy this dictionary file to Windows: {dict_path}", flush=True)

    window_name = "aruco sender"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    start_id = marker_id
    print("Keys: q/Esc quit, n next, p previous, +/- step 10, r reset.", flush=True)
    while True:
        image = make_marker_image(dictionary, marker_id, args.display_size, args.border_px)
        cv2.putText(
            image,
            f"ID: {marker_id}",
            (args.border_px, max(40, args.border_px // 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window_name, image)
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("n"):
            marker_id = min(args.dict_size - 1, marker_id + 1)
        elif key == ord("p"):
            marker_id = max(0, marker_id - 1)
        elif key in (ord("+"), ord("=")):
            marker_id = min(args.dict_size - 1, marker_id + 10)
        elif key in (ord("-"), ord("_")):
            marker_id = max(0, marker_id - 10)
        elif key == ord("r"):
            marker_id = start_id

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
