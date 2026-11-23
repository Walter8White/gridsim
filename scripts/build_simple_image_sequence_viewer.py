#!/usr/bin/env python3
"""Build a simple HTML viewer that lays out GridCapture images by pass and order."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from PIL import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--thumb-width", type=int, default=420)
    return parser.parse_args()


def load_records(session_dir: Path) -> list[dict]:
    records = []
    with (session_dir / "manifest.jsonl").open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("type") == "capture":
                records.append(record)
    records.sort(key=lambda item: int(item.get("global_capture_index", 0)))
    return records


def image_filename(record: dict) -> str:
    result = record.get("iphone_result") or {}
    return result.get("photo_filename") or f"{record['capture_id']}.jpg"


def make_thumbnails(session_dir: Path, output_dir: Path, records: list[dict], thumb_width: int) -> None:
    thumb_dir = output_dir / "thumbs"
    thumb_dir.mkdir(parents=True, exist_ok=True)
    for record in records:
        filename = image_filename(record)
        source = session_dir / "images" / filename
        target = thumb_dir / filename
        if target.exists():
            continue
        with Image.open(source) as image:
            image = image.convert("RGB")
            width, height = image.size
            thumb_height = max(1, round(height * thumb_width / width))
            image = image.resize((thumb_width, thumb_height), Image.Resampling.LANCZOS)
            image.save(target, quality=88, optimize=True)


def build_html(session_dir: Path, output_dir: Path, records: list[dict], thumb_width: int) -> None:
    passes = sorted({int(record.get("pass_index", 0)) for record in records})
    by_pass = {pass_index: [] for pass_index in passes}
    for record in records:
        by_pass[int(record.get("pass_index", 0))].append(record)

    for pass_records in by_pass.values():
        pass_records.sort(key=lambda item: float(item.get("measured_position_mm", 0)), reverse=True)

    rows = max(len(items) for items in by_pass.values())
    grid_cells = []
    for row_index in range(rows):
        for pass_index in passes:
            items = by_pass[pass_index]
            if row_index >= len(items):
                grid_cells.append('<div class="empty"></div>')
                continue
            record = items[row_index]
            filename = image_filename(record)
            z_m = (float(record["measured_position_mm"]) + 150.0) / 1000.0
            x_m = float(record.get("x_offset_m", 0.0))
            grid_cells.append(
                f"""
                <figure class="tile">
                  <img src="thumbs/{filename}" loading="lazy" alt="{record['capture_id']}">
                  <figcaption>
                    <b>p{pass_index:02d}</b> #{int(record.get('pass_capture_index', 0)):02d}
                    <span>z={z_m:.3f}m x={x_m:.3f}m</span>
                  </figcaption>
                </figure>
                """
            )

    sequence_cells = []
    for record in records:
        filename = image_filename(record)
        sequence_cells.append(
            f"""
            <figure class="tile">
              <img src="thumbs/{filename}" loading="lazy" alt="{record['capture_id']}">
              <figcaption>
                <b>#{int(record.get('global_capture_index', 0)):03d}</b>
                p{int(record.get('pass_index', 0)):02d}-{int(record.get('pass_capture_index', 0)):02d}
                <span>z={float(record['measured_position_mm']) / 1000.0:.3f}m</span>
              </figcaption>
            </figure>
            """
        )

    pass_headers = "".join(f"<div class=\"pass-header\">Pass {p:02d}</div>" for p in passes)
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Simple Image Sequence Viewer</title>
  <style>
    :root {{
      --tile-width: {thumb_width}px;
      --gap: 10px;
    }}
    body {{
      margin: 0;
      background: #101418;
      color: #e6edf3;
      font-family: Arial, sans-serif;
    }}
    header {{
      position: sticky;
      top: 0;
      z-index: 10;
      background: rgba(16, 20, 24, 0.94);
      border-bottom: 1px solid rgba(230,237,243,0.16);
      padding: 14px 18px;
    }}
    h1 {{ margin: 0 0 6px; font-size: 20px; }}
    .meta {{ color: #9da7b1; font-size: 14px; }}
    .tabs {{ display: flex; gap: 8px; margin-top: 12px; }}
    button {{
      border: 1px solid rgba(230,237,243,0.2);
      background: #1f2933;
      color: #e6edf3;
      padding: 8px 12px;
      border-radius: 7px;
      cursor: pointer;
    }}
    button.active {{ background: #238636; border-color: #2ea043; }}
    main {{ padding: 18px; }}
    .view {{ display: none; }}
    .view.active {{ display: block; }}
    .pass-grid {{
      display: grid;
      grid-template-columns: repeat({len(passes)}, var(--tile-width));
      gap: var(--gap);
      align-items: start;
      width: max-content;
    }}
    .pass-header {{
      position: sticky;
      top: 92px;
      z-index: 5;
      background: #18212b;
      border: 1px solid rgba(230,237,243,0.16);
      border-radius: 7px;
      padding: 8px;
      text-align: center;
      font-weight: bold;
    }}
    .sequence-grid {{
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(var(--tile-width), 1fr));
      gap: var(--gap);
    }}
    .tile {{
      margin: 0;
      background: #18212b;
      border: 1px solid rgba(230,237,243,0.13);
      border-radius: 8px;
      overflow: hidden;
    }}
    .tile img {{
      display: block;
      width: 100%;
      height: auto;
    }}
    figcaption {{
      display: flex;
      justify-content: space-between;
      gap: 8px;
      padding: 7px 9px;
      font-size: 13px;
      color: #c9d1d9;
    }}
    figcaption span {{ color: #79c0ff; }}
    .empty {{
      min-height: 40px;
      border: 1px dashed rgba(230,237,243,0.1);
      border-radius: 8px;
    }}
  </style>
</head>
<body>
  <header>
    <h1>Simple Image Sequence Viewer</h1>
    <div class="meta">
      Session: <code>{session_dir.name}</code> |
      captures: {len(records)} |
      passes: {len(passes)} |
      top row = highest Zaber position
    </div>
    <div class="tabs">
      <button class="tab active" data-view="ordered">Ordered Grid</button>
      <button class="tab" data-view="sequence">Capture Sequence</button>
    </div>
  </header>
  <main>
    <section id="ordered" class="view active">
      <div class="pass-grid">
        {pass_headers}
        {''.join(grid_cells)}
      </div>
    </section>
    <section id="sequence" class="view">
      <div class="sequence-grid">
        {''.join(sequence_cells)}
      </div>
    </section>
  </main>
  <script>
    for (const tab of document.querySelectorAll('.tab')) {{
      tab.addEventListener('click', () => {{
        document.querySelectorAll('.tab').forEach(item => item.classList.remove('active'));
        document.querySelectorAll('.view').forEach(item => item.classList.remove('active'));
        tab.classList.add('active');
        document.getElementById(tab.dataset.view).classList.add('active');
      }});
    }}
  </script>
</body>
</html>
"""
    (output_dir / "viewer.html").write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or args.session_dir / "simple_image_viewer"
    output_dir.mkdir(parents=True, exist_ok=True)
    records = load_records(args.session_dir)
    make_thumbnails(args.session_dir, output_dir, records, args.thumb_width)
    build_html(args.session_dir, output_dir, records, args.thumb_width)
    print(f"[simple-viewer] output: {output_dir / 'viewer.html'}")
    print(f"[simple-viewer] captures: {len(records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
