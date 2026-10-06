# RGB/ToF experiment archive

Experiments from 6 October 2026 are stored in
[Drive: Experiments / Homer RGB-ToF — 2026-10-06](https://drive.google.com/drive/folders/1S_-sS2tz7Tsml5OSq2OYRu8bfb8DwL1c).

`homer-rgb-tof-experiments-2026-10-06.zip` contains 1,171 capture files, including
RGB/USB/IR images, raw ToF depth and confidence arrays, segmentation masks,
calibration fits, diagnostic results and logs. A source snapshot preserves the
pipeline and provisional calibration used for the experiments. The older
committed capture is also retained, including arrays later moved locally.

- Archive size: 173,539,635 bytes.
- SHA-256: `8e76c6b17bbf415c0b98bdcd31af6ab92d43653c6cb64a6c8ad691f8ba1b12cb`.
- `MANIFEST.json` records each capture file's SHA-256, size and path.

Local captures are preserved under `captures/` and ignored by Git. Future
captures use timestamped directories; they do not overwrite previous captures.
Local archive bundles are stored under ignored `experiment-archives/`.

Reported object distance is the median Euclidean range from the ToF camera
over accepted surface samples. Burst distance is the median of frame medians.
Minima are diagnostic extremes. Physical accuracy and autonomous motion remain
unqualified; see `pi/approach_object/tools/SURFACE_DISTANCE.md` for limitations.
