"""Readable previews of raw sensor values; no geometric correction."""
import cv2
import numpy as np


def data_file(folder, name):
    nested = folder / 'data' / name
    return nested if nested.is_file() else folder / name


def depth_preview(depth_mm, mask=None, amplitude=None):
    depth = np.asarray(depth_mm, dtype=float)
    valid = np.isfinite(depth) & (depth > 0)
    # Fixed centimetre scale, identical between captures. Black is invalid.
    values = np.clip(np.where(valid, depth, 0) / 2000 * 255, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(values, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    colored = cv2.resize(colored, (720, 540), interpolation=cv2.INTER_NEAREST)
    if mask is not None and np.any(mask):
        enlarged = cv2.resize((mask > 0).astype(np.uint8), (720, 540), interpolation=cv2.INTER_NEAREST)
        contours, _ = cv2.findContours(enlarged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(colored, contours, -1, (255, 255, 255), 2)
    canvas = np.full((630, 830, 3), 24, np.uint8)
    canvas[60:600, :720] = colored
    bar = np.linspace(0, 255, 540).astype(np.uint8)[:, None]
    canvas[60:600, 735:755] = cv2.applyColorMap(np.repeat(bar, 20, axis=1), cv2.COLORMAP_TURBO)
    for cm in (0, 50, 100, 150, 200):
        y = 60 + int(cm / 200 * 539)
        cv2.putText(canvas, f'{cm}', (765, min(y + 6, 600)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    cv2.putText(canvas, 'ToF depth | sensor cm | black = invalid', (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    if mask is not None:
        cv2.putText(canvas, 'White outline = estimated RGB projection; alignment uncalibrated', (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1)
    cv2.putText(canvas, 'Scale capped at 200 cm; sensor depth interpretation unverified', (12, 620), cv2.FONT_HERSHEY_SIMPLEX, .47, (255, 255, 255), 1)
    if amplitude is None:
        return canvas
    signal = np.asarray(amplitude, dtype=float)
    valid_signal = np.isfinite(signal) & (signal > 0)
    log_signal = np.log1p(np.where(valid_signal, signal, 0))
    gray = np.zeros(signal.shape, np.uint8)
    if valid_signal.any():
        lo, hi = np.percentile(log_signal[valid_signal], [1, 99])
        gray[valid_signal] = np.clip((log_signal[valid_signal]-lo) / max(hi-lo, 1e-6) * 255, 0, 255).astype(np.uint8)
    combined = np.full((630, 1560, 3), 24, np.uint8)
    combined[:, :830] = canvas
    combined[60:600, 840:1560] = cv2.cvtColor(cv2.resize(gray, (720, 540), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    cv2.putText(combined, 'ToF infrared return | brightness enhanced', (850, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    cv2.putText(combined, 'Same ToF frame; brightness is not distance', (850, 620), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    return combined
