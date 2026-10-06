# RGB/ToF experiment archive

Experiments from 6 October 2026 are stored in
[Drive: Experiments / Homer RGB-ToF — 2026-10-06](https://drive.google.com/drive/folders/1S_-sS2tz7Tsml5OSq2OYRu8bfb8DwL1c).

The two `homer-rgb-tof-experiments-2026-10-06-part-*.zip` archives contain 1,171 capture files, including
RGB/USB/IR images, raw ToF depth and confidence arrays, segmentation masks,
calibration fits, diagnostic results and logs. A source snapshot preserves the
pipeline and provisional calibration used for the experiments. The older
committed capture is also retained, including arrays later moved locally.

- Part 01: 94,317,164 bytes; SHA-256
  `0b6887eab85e073edd4ea8b5e1034170e85febe74e908a5bd3ae17f66ae3b36f`.
- Part 02: 79,222,493 bytes; SHA-256
  `bb90e396f118c016f22753efe77f6f2a16c340a9030f4352eebcc6ce7b6daa33`.
- `MANIFEST.json` records each capture file's SHA-256, size and path.

Extract both ZIPs into the same directory; each contains different files and
can be opened independently. The two-part arrangement keeps each upload below
the Drive connector's 100 MB limit. The manifest and source snapshot are in
part 02. The single-file local ZIP is also preserved but was too large to upload.

Local captures are preserved under `captures/` and ignored by Git. Future
captures use timestamped directories; they do not overwrite previous captures.
Local archive bundles are stored under ignored `experiment-archives/`.

Reported object distance is the median Euclidean range from the ToF camera
over accepted surface samples. Burst distance is the median of frame medians.
Minima are diagnostic extremes. Physical accuracy and autonomous motion remain
unqualified; see `pi/approach_object/tools/SURFACE_DISTANCE.md` for limitations.
