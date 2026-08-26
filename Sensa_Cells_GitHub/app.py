from __future__ import annotations

import base64
import io
import json
import os
import re
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from dash import ALL, Dash, Input, Output, State, ctx, dcc, html, no_update

# Optional integrations. The app still runs without OPENAI_API_KEY.
try:
    from dotenv import load_dotenv
    load_dotenv()
except Exception:
    pass

try:
    from openai import OpenAI
except Exception:
    OpenAI = None

try:
    from supabase import create_client
except Exception:
    create_client = None

APP_DIR = Path(__file__).resolve().parent
ASSETS = APP_DIR / "assets"
DATA_DIR = APP_DIR / "data"
DATABASE_FILE = DATA_DIR / "sensa_cells.json"
SUPABASE_URL = os.getenv("SUPABASE_URL", "").strip()
SUPABASE_SERVICE_KEY = os.getenv("SUPABASE_SERVICE_KEY", "").strip()
SUPABASE_BUCKET = os.getenv("SUPABASE_BUCKET", "archivos-celdas").strip()
SUPABASE_TABLE = os.getenv("SUPABASE_TABLE", "Celdas").strip()


def get_supabase():
    if not (create_client and SUPABASE_URL and SUPABASE_SERVICE_KEY):
        return None
    return create_client(SUPABASE_URL, SUPABASE_SERVICE_KEY)


def storage_upload(client, path: str, content: bytes, content_type: str) -> None:
    client.storage.from_(SUPABASE_BUCKET).upload(
        path=path,
        file=content,
        file_options={"content-type": content_type, "upsert": "true"},
    )


def storage_download(client, path: str) -> bytes:
    result = client.storage.from_(SUPABASE_BUCKET).download(path)
    return bytes(result)

BLUE = "#0B438D"
BLUE_2 = "#0A4CA8"
YELLOW = "#F7B900"
INK = "#111827"
MUTED = "#64748B"
GREEN = "#20A83A"
BORDER = "#DCE4EF"
ROI_VSET_MIN = 0.2
ROI_VSET_MAX = 0.6
SOFT = "#F7F9FC"

app = Dash(__name__, title="SENSA Cells", suppress_callback_exceptions=True)
server = app.server
app.index_string = """<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            #researcher-panel .Select-control {
                min-height: 44px !important;
                border: 1px solid #CBD5E1 !important;
                border-radius: 9px !important;
                box-shadow: none !important;
            }
            #researcher-panel .is-focused:not(.is-open) > .Select-control,
            #researcher-panel .Select-control:hover {
                border-color: #0B438D !important;
                box-shadow: 0 0 0 3px rgba(11,67,141,.10) !important;
            }
            #researcher-panel .Select-placeholder,
            #researcher-panel .Select-value-label,
            #researcher-panel .Select-input {
                line-height: 42px !important;
                color: #334155 !important;
            }
            #researcher-panel .Select-menu-outer {
                border-color: #CBD5E1 !important;
                border-radius: 0 0 9px 9px !important;
                box-shadow: 0 10px 24px rgba(15,23,42,.14) !important;
                z-index: 2100 !important;
            }
            #new-researcher-name:focus {
                outline: none !important;
                border-color: #0B438D !important;
                box-shadow: 0 0 0 3px rgba(11,67,141,.10) !important;
            }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>{%config%}{%scripts%}{%renderer%}</footer>
    </body>
</html>"""


# -----------------------------------------------------------------------------
# Data helpers
# -----------------------------------------------------------------------------
def empty_database() -> dict[str, Any]:
    return {"cells": [], "reports_generated": 0, "researchers": [], "current_researcher": None, "next_cell_number": 1}


def load_local_database() -> dict[str, Any]:
    if not DATABASE_FILE.exists():
        return empty_database()
    try:
        data = json.loads(DATABASE_FILE.read_text(encoding="utf-8"))
        if not isinstance(data.get("cells"), list):
            raise ValueError("Formato inválido")
        data.setdefault("reports_generated", 0)
        data.setdefault("researchers", [])
        data.setdefault("current_researcher", None)
        if "next_cell_number" not in data:
            existing_numbers = []
            for cell in data["cells"]:
                match = re.search(r"(\d+)$", str(cell.get("id", "")))
                if match:
                    existing_numbers.append(int(match.group(1)))
            data["next_cell_number"] = max(existing_numbers, default=0) + 1
        return data
    except Exception:
        # No se inventan datos si el archivo está vacío o dañado.
        return empty_database()


def load_database() -> dict[str, Any]:
    """Carga Supabase cuando está configurado; usa el JSON local como respaldo."""
    client = get_supabase()
    if client is None:
        return load_local_database()
    try:
        rows = client.table(SUPABASE_TABLE).select("*").order("fecha_creacion").execute().data or []
        cells_by_name = {}
        for row in rows:
            route = str(row.get("ruta_storage") or "").strip("/")
            if not route:
                continue
            try:
                manifest = json.loads(storage_download(client, f"{route}/manifest.json").decode("utf-8"))
                manifest["supabase_row_id"] = row.get("id")
                manifest["storage_route"] = route
                # La tabla de Supabase es la fuente canónica para el nombre de la celda.
                # Así un manifest histórico no puede ocultar otra celda por tener un ID repetido.
                cell_name = str(row.get("nombre") or manifest.get("id") or "").strip()
                if not cell_name:
                    continue
                manifest["id"] = cell_name
                cells_by_name[cell_name] = manifest
            except Exception:
                # Un archivo dañado no impide mostrar el resto de celdas.
                continue
        cells = list(cells_by_name.values())
        researchers = sorted({c.get("researcher") for c in cells if c.get("researcher")}, key=str.casefold)
        numbers = []
        for cell in cells:
            match = re.search(r"(\d+)$", str(cell.get("id", "")))
            if match:
                numbers.append(int(match.group(1)))
        return {
            "cells": cells,
            "reports_generated": sum(bool(c.get("report_generated")) for c in cells),
            "researchers": researchers,
            "current_researcher": researchers[0] if researchers else None,
            "next_cell_number": max(numbers, default=0) + 1,
        }
    except Exception as exc:
        local = load_local_database()
        local["storage_warning"] = f"Supabase no disponible: {exc}"
        return local


def save_database(data: dict[str, Any]) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    temporary = DATABASE_FILE.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    temporary.replace(DATABASE_FILE)


def next_cell_id(database: dict[str, Any]) -> str:
    next_number = int(database.get("next_cell_number", 1))
    return f"CELDA-{next_number:03d}"


def save_completed_cell(database: dict[str, Any], cell: dict[str, Any]) -> dict[str, Any]:
    if any(item.get("id") == cell.get("id") for item in database.get("cells", [])):
        raise ValueError("La celda ya fue guardada.")
    database.setdefault("cells", []).append(cell)
    database["next_cell_number"] = int(database.get("next_cell_number", 1)) + 1
    save_database(database)
    return database


def save_cell_to_supabase(cell: dict[str, Any], pdf_b64: str | None) -> dict[str, Any]:
    """Guarda los CSV originales, PDF, manifiesto y fila de índice de una celda."""
    client = get_supabase()
    if client is None:
        return cell
    route = str(cell["id"]).lower()
    stored_files = []
    manifest = json.loads(json.dumps(cell))
    for record, manifest_record in zip(cell.get("records", []), manifest.get("records", [])):
        source_b64 = record.get("source_b64")
        if not source_b64:
            raise ValueError(f"No se conservó el CSV original: {record.get('filename', 'sin nombre')}")
        filename = Path(str(record.get("filename", "ensayo.csv"))).name
        storage_upload(client, f"{route}/csv/{filename}", base64.b64decode(source_b64), "text/csv")
        stored_files.append({"name": filename, "path": f"{route}/csv/{filename}", "type": "csv"})
        manifest_record.pop("source_b64", None)
    if pdf_b64:
        pdf_name = str(cell.get("report_filename") or f"SENSA_Informe_{cell['id']}.pdf")
        storage_upload(client, f"{route}/informe/{pdf_name}", base64.b64decode(pdf_b64), "application/pdf")
        stored_files.append({"name": pdf_name, "path": f"{route}/informe/{pdf_name}", "type": "pdf"})
    manifest["storage_route"] = route
    manifest["storage_files"] = stored_files
    storage_upload(
        client,
        f"{route}/manifest.json",
        json.dumps(manifest, ensure_ascii=False).encode("utf-8"),
        "application/json",
    )
    composition = cell.get("composition") or {}
    period = assay_period(cell.get("records", []))
    date_value = None
    try:
        date_value = datetime.strptime(
            f"{period['date']} {period['start_time']}", "%d/%m/%Y %H:%M:%S"
        ).isoformat()
    except (TypeError, ValueError):
        pass
    payload = {
        "nombre": cell["id"],
        "fecha_ensayo": date_value,
        "investigador": cell.get("researcher"),
        "concentracion_cu_ppm": cell.get("concentration_ppm"),
        "volumen_kcl_ml": composition.get("initial_volume_ml"),
        "concentracion_kcl_m": composition.get("electrolyte_concentration"),
        "volumen_cobre_ml": composition.get("copper_added_ml"),
        "concentracion_final_ppm": composition.get("final_copper_ppm"),
        "descripcion": cell.get("observations") or None,
        "ruta_storage": route,
    }
    existing = (
        client.table(SUPABASE_TABLE)
        .select("id")
        .eq("nombre", cell["id"])
        .limit(1)
        .execute()
        .data
        or []
    )
    if existing:
        response = (
            client.table(SUPABASE_TABLE)
            .update(payload)
            .eq("id", existing[0]["id"])
            .execute()
        )
    else:
        response = client.table(SUPABASE_TABLE).insert(payload).execute()
    if response.data:
        manifest["supabase_row_id"] = response.data[0].get("id")
    return manifest


def infer_metadata(filename: str) -> dict[str, Any]:
    name = filename.lower().replace("µ", "u")
    technique = None
    if re.search(r"(^|[^a-z])lsv([^a-z]|$)", name):
        technique = "LSV"
    elif re.search(r"(^|[^a-z])cv([^a-z]|$)", name):
        technique = "CV"
    elif re.search(r"(^|[^a-z])ca([^a-z]|$)", name):
        technique = "CA"

    stage = "blanco" if any(k in name for k in ["blank", "blanco", "white", "0ppm", "0_ppm", "0ppb", "0_ppb"]) else "cobre"

    concentration = None
    m = re.search(r"(\d+(?:[.,]\d+)?)\s*(ppm|ppb)", name)
    if m:
        value = float(m.group(1).replace(",", "."))
        concentration = value / 1000 if m.group(2) == "ppb" else value
    elif stage == "blanco":
        concentration = 0.0

    rep = None
    if technique == "LSV":
        mrep = re.search(r"lsv[\s_\-]*(?:rep|r)?[\s_\-]*([123])", name)
        if mrep:
            rep = int(mrep.group(1))

    return {
        "technique": technique,
        "stage": stage,
        "concentration_ppm": concentration,
        "rep": rep,
    }


def assay_period(records):
    """Obtiene fecha y horas reales desde nombres SENSA YYYYMMDD_HHMMSS."""
    moments = []
    for record in records or []:
        match = re.search(r"(20\d{6})[_-](\d{6})", str(record.get("filename", "")))
        if not match:
            continue
        try:
            moments.append(datetime.strptime("".join(match.groups()), "%Y%m%d%H%M%S"))
        except ValueError:
            continue
    if not moments:
        return {"date": "No identificada", "time": "No identificada", "start_time": "No identificada", "end_time": "No identificada"}
    start, end = min(moments), max(moments)
    date_text = start.strftime("%d/%m/%Y") if start.date() == end.date() else f"{start:%d/%m/%Y} – {end:%d/%m/%Y}"
    time_text = start.strftime("%H:%M:%S") if start == end else f"{start:%H:%M:%S} – {end:%H:%M:%S}"
    return {"date": date_text, "time": time_text, "start_time": start.strftime("%H:%M:%S"), "end_time": end.strftime("%H:%M:%S")}


def decode_csv(contents: str, filename: str) -> pd.DataFrame:
    _, payload = contents.split(",", 1)
    raw = base64.b64decode(payload)
    text = None
    for enc in ("utf-8-sig", "utf-8", "latin-1"):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        raise ValueError(f"No pude decodificar {filename}")

    attempts = [
        {"sep": None, "engine": "python"},
        {"sep": ";", "engine": "python"},
        {"sep": ",", "engine": "python"},
        {"sep": "\t", "engine": "python"},
    ]
    last_error = None
    for kwargs in attempts:
        try:
            df = pd.read_csv(io.StringIO(text), **kwargs)
            if df.shape[1] >= 2:
                return df
        except Exception as exc:
            last_error = exc
    raise ValueError(f"No pude leer {filename}: {last_error}")


def choose_xy(df: pd.DataFrame, technique: str | None) -> tuple[np.ndarray, np.ndarray, bool, str, str]:
    cleaned = df.copy()
    for c in cleaned.columns:
        if cleaned[c].dtype == object:
            cleaned[c] = pd.to_numeric(
                cleaned[c].astype(str).str.replace(",", ".", regex=False).str.strip(),
                errors="coerce",
            )

    numeric = [c for c in cleaned.columns if pd.api.types.is_numeric_dtype(cleaned[c]) and cleaned[c].notna().sum() > 5]
    if len(numeric) < 2:
        raise ValueError("El CSV necesita al menos dos columnas numéricas.")

    lower = {c: str(c).lower() for c in numeric}
    normalized = {c: re.sub(r"[^a-z0-9]+", "_", lower[c].replace("µ", "u")).strip("_") for c in numeric}
    current_keys = ("current", "corriente", "amp", "microamp", "ua", "ma", "i(", " i ")
    vset_keys = ("vset", "v_set", "v set", "voltaje_set", "voltaje set", "set_voltage", "set voltage")
    potential_keys = ("potential", "potencial", "voltage", "voltaje", "ewe", "v(")
    time_keys = ("time", "tiempo", "seconds", "seg", "s(")

    def pick(keys):
        for c in numeric:
            if any(k in lower[c] for k in keys):
                return c
        return None

    # Regla validada con archivos SENSA: CA usa corriente cruda en uA porque la
    # ventana exportada (100) excede sus 49-50 puntos; CV/LSV usan filtrada_uA.
    if technique == "CA":
        ycol = next((c for c in numeric if normalized[c] in ("corriente_ua", "current_ua")), None)
        source_is_filtered = False
    else:
        ycol = next((c for c in numeric if normalized[c] in ("corriente_filtrada_ua", "filtered_current_ua", "current_filtered_ua")), None)
        source_is_filtered = ycol is not None
    if ycol is None:
        ycol = pick(current_keys)
        source_is_filtered = any(k in normalized.get(ycol, "") for k in ("filtr", "smooth", "suav")) if ycol is not None else False
    vset_col = pick(vset_keys)
    # CA se interpreta contra tiempo; CV y LSV usan estrictamente Vset cuando existe.
    if technique in ("CV", "LSV") and vset_col is None:
        raise ValueError("No se encontró la columna Vset requerida para CV/LSV.")
    xcol = (pick(time_keys) or vset_col) if technique == "CA" else vset_col

    if xcol is None:
        xcol = numeric[0]
    if ycol is None or ycol == xcol:
        ycol = next((c for c in numeric if c != xcol), numeric[1])

    pair = cleaned[[xcol, ycol]].dropna()
    if len(pair) < 5:
        raise ValueError("No hay suficientes puntos válidos en las columnas detectadas.")
    return pair[xcol].to_numpy(), pair[ycol].to_numpy(), source_is_filtered, str(xcol), str(ycol)


def smooth_signal(y: np.ndarray, technique: str | None) -> tuple[np.ndarray, int]:
    """Filtrado adaptativo robusto: elimina impulsos y reduce ruido sin borrar picos."""
    values = np.asarray(y, dtype=float)
    n = len(values)
    if n < 7:
        return values, 1
    target = 5 if technique == "CA" else max(7, min(101, n // 50))
    if target % 2 == 0:
        target += 1
    target = min(target, n if n % 2 else n - 1)
    series = pd.Series(values)
    median_window = max(3, min(target, 11))
    filtered = series.rolling(median_window, center=True, min_periods=1).median()
    filtered = filtered.rolling(target, center=True, min_periods=1).mean()
    return filtered.to_numpy(dtype=float), int(target)


def parse_upload(contents_list, filenames) -> tuple[list[dict[str, Any]], list[str]]:
    records = []
    errors = []
    for contents, filename in zip(contents_list or [], filenames or []):
        try:
            meta = infer_metadata(filename)
            if meta["technique"] is None:
                raise ValueError("No detecté CA, CV o LSV en el nombre del archivo.")
            df = decode_csv(contents, filename)
            x, y_raw, source_is_filtered, x_column, y_column = choose_xy(df, meta["technique"])
            if meta["technique"] == "CA" and "ms" in x_column.lower():
                x = np.asarray(x, dtype=float) / 1000.0
                x_column = f"{x_column} convertido a s"
            if source_is_filtered:
                y = np.asarray(y_raw, dtype=float)
                window_col = next((c for c in df.columns if "ventana_filtro" in str(c).lower()), None)
                filter_window = int(pd.to_numeric(df[window_col], errors="coerce").dropna().iloc[0]) if window_col is not None and pd.to_numeric(df[window_col], errors="coerce").notna().any() else 100
            else:
                y, filter_window = smooth_signal(y_raw, meta["technique"])
            nonzero_steps = np.diff(x)
            nonzero_steps = nonzero_steps[np.abs(nonzero_steps) > 1e-12]
            initial_direction = "ascendente" if len(nonzero_steps) and nonzero_steps[0] > 0 else "descendente"
            if meta["technique"] == "CV" and initial_direction != "descendente":
                raise ValueError("El CV debe iniciar con Vset positivo y barrer hacia valores negativos.")
            if meta["technique"] == "LSV" and initial_direction != "ascendente":
                raise ValueError("La LSV debe barrer Vset desde negativo hacia positivo.")
            cycle_col = next((c for c in df.columns if str(c).lower().strip() in ("ciclo", "cycle")), None)
            cycle_count = int(pd.to_numeric(df[cycle_col], errors="coerce").nunique()) if cycle_col is not None else 1
            source_b64 = contents.split(",", 1)[1]
            records.append({"filename": filename, "source_b64": source_b64, "x": x.tolist(), "y": y.tolist(), "y_raw": y_raw.tolist(), "filter_window": filter_window, "x_column": x_column, "y_column": y_column, "sweep_direction": initial_direction, "cycle_count": cycle_count, "vset_start": float(x[0]), "vset_end": float(x[-1]), "vset_min": float(np.min(x)), "vset_max": float(np.max(x)), **meta})
        except Exception as exc:
            errors.append(f"{filename}: {exc}")

    # Number copper LSV files automatically when the filename does not include 1/2/3.
    unnumbered = [r for r in records if r["stage"] == "cobre" and r["technique"] == "LSV" and r["rep"] is None]
    for i, rec in enumerate(unnumbered, start=1):
        rec["rep"] = min(i, 3)
    return records, errors


def concentration_from_records(records: list[dict[str, Any]]) -> float | None:
    vals = [r.get("concentration_ppm") for r in records if r.get("stage") == "cobre" and r.get("concentration_ppm") not in (None, 0)]
    return float(vals[0]) if vals else None


def get_record(records, technique, stage, rep=None):
    for r in records:
        if r.get("technique") == technique and r.get("stage") == stage:
            if rep is None or r.get("rep") == rep:
                return r
    return None


def record_signal(record: dict[str, Any]) -> np.ndarray:
    y = np.asarray(record.get("y", []), dtype=float)
    if record.get("filter_window"):
        return y
    filtered, _ = smooth_signal(y, record.get("technique"))
    return filtered


def signal_in_roi(record: dict[str, Any]):
    """Devuelve Vset y corriente filtrada solo entre 0.2 y 0.6 V."""
    x = np.asarray(record.get("x", []), dtype=float)
    y = record_signal(record)
    size = min(len(x), len(y))
    x, y = x[:size], y[:size]
    mask = np.isfinite(x) & np.isfinite(y) & (x >= ROI_VSET_MIN) & (x <= ROI_VSET_MAX)
    return x[mask], y[mask]


def roi_segments(record: dict[str, Any]):
    """Separa cada paso de CV/LSV por la región sin mezclar ida y retorno."""
    x = np.asarray(record.get("x", []), dtype=float)
    y = record_signal(record)
    size = min(len(x), len(y))
    x, y = x[:size], y[:size]
    valid = np.isfinite(x) & np.isfinite(y) & (x >= ROI_VSET_MIN) & (x <= ROI_VSET_MAX)
    indices = np.where(valid)[0]
    if not len(indices):
        return []
    groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
    return [(x[group], y[group]) for group in groups if len(group) >= 5]


def analyze_roi_feature(record: dict[str, Any]):
    """Clasifica de forma trazable un pico, una meseta o una respuesta no definida."""
    candidates = []
    for segment_number, (x, y) in enumerate(roi_segments(record), start=1):
        order = np.argsort(x)
        xs, ys = x[order], y[order]
        amplitude = float(np.nanmax(ys) - np.nanmin(ys))
        if len(xs) < 5 or amplitude <= 1e-12:
            continue
        peak_index = int(np.nanargmax(ys))
        peak_x, peak_y = float(xs[peak_index]), float(ys[peak_index])
        baseline_at_peak = float(np.interp(peak_x, [xs[0], xs[-1]], [ys[0], ys[-1]]))
        prominence = peak_y - baseline_at_peak
        high_mask = ys >= peak_y - 0.15 * amplitude
        high_x, high_y = xs[high_mask], ys[high_mask]
        high_span = float(np.nanmax(high_x) - np.nanmin(high_x)) if len(high_x) else 0.0
        high_variation = float(np.nanmax(high_y) - np.nanmin(high_y)) if len(high_y) else np.nan
        interior_peak = ROI_VSET_MIN + 0.03 <= peak_x <= ROI_VSET_MAX - 0.03

        if high_span >= 0.08 and high_variation <= 0.18 * amplitude:
            feature_type = "meseta"
            feature_width = high_span
            center_x = float(np.nanmean(high_x))
            feature_current = float(np.nanmean(high_y))
            score = high_span * amplitude
        elif interior_peak and prominence >= 0.15 * amplitude:
            feature_type = "pico"
            half_level = baseline_at_peak + 0.5 * prominence
            above_half = xs[ys >= half_level]
            feature_width = float(np.nanmax(above_half) - np.nanmin(above_half)) if len(above_half) >= 2 else 0.0
            center_x, feature_current = peak_x, peak_y
            score = prominence
        else:
            feature_type = "sin rasgo definido"
            feature_width = 0.0
            center_x, feature_current = peak_x, peak_y
            score = 0.0
        candidates.append({
            "tipo": feature_type,
            "vset_centro_V": center_x,
            "corriente_uA": feature_current,
            "ancho_V": feature_width,
            "prominencia_uA": float(prominence),
            "amplitud_region_uA": amplitude,
            "segmento": segment_number,
            "puntos": int(len(xs)),
            "score": float(score),
        })
    if not candidates:
        return {"tipo": "sin datos suficientes", "vset_centro_V": None, "corriente_uA": None, "ancho_V": None, "prominencia_uA": None, "amplitud_region_uA": None, "segmento": None, "puntos": 0}
    priority = {"pico": 2, "meseta": 1, "sin rasgo definido": 0}
    best = max(candidates, key=lambda item: (priority[item["tipo"]], item["score"]))
    best.pop("score", None)
    return best


def describe_roi_feature(label, feature):
    feature_type = feature.get("tipo")
    if feature_type == "pico":
        return (f"{label}: se detecta un **pico** centrado en **{feature['vset_centro_V']:.3f} V**, "
                f"con corriente de **{feature['corriente_uA']:.3g} uA**, prominencia de **{feature['prominencia_uA']:.3g} uA** "
                f"y ancho aproximado de **{feature['ancho_V']:.3f} V**.")
    if feature_type == "meseta":
        return (f"{label}: se detecta una **meseta** centrada alrededor de **{feature['vset_centro_V']:.3f} V**, "
                f"con corriente media de **{feature['corriente_uA']:.3g} uA** y extensión aproximada de **{feature['ancho_V']:.3f} V**.")
    if feature_type == "sin rasgo definido":
        return (f"{label}: no se distingue un pico ni una meseta suficientemente definidos en **0.2–0.6 V**; "
                f"el mayor valor observado es **{feature['corriente_uA']:.3g} uA** a **{feature['vset_centro_V']:.3f} V**.")
    return f"{label}: no existen puntos suficientes para clasificar la forma en **0.2–0.6 V**."


def protocol_state(records):
    expected = [
        ("CA", "blanco", None, "CA Blanco"),
        ("CV", "blanco", None, "CV Blanco"),
        ("CA", "cobre", None, "CA Cu²⁺"),
        ("CV", "cobre", None, "CV Cu²⁺"),
        ("LSV", "cobre", 1, "LSV 1"),
        ("LSV", "cobre", 2, "LSV 2"),
        ("LSV", "cobre", 3, "LSV 3"),
    ]
    states = []
    for tech, stage, rep, label in expected:
        states.append((label, stage, get_record(records, tech, stage, rep) is not None))
    return states


def calculate_metrics(records):
    metrics = []
    for r in records:
        x = np.asarray(r["x"], dtype=float)
        y = record_signal(r)
        if r.get("technique") in ("CV", "LSV"):
            x, y = signal_in_roi(r)
        if len(y) == 0:
            continue
        item = {
            "archivo": r["filename"],
            "técnica": r["technique"],
            "etapa": r["stage"],
            "n": int(len(y)),
            "región_Vset_V": "0.2–0.6" if r.get("technique") in ("CV", "LSV") else "No aplica (CA vs tiempo)",
            "i_media": float(np.nanmean(y)),
            "i_min": float(np.nanmin(y)),
            "i_max": float(np.nanmax(y)),
            "i_std": float(np.nanstd(y)),
        }
        if r["technique"] in ("LSV", "CV"):
            idx = int(np.nanargmax(np.abs(y)))
            item["potencial_evento"] = float(x[idx])
            item["corriente_evento"] = float(y[idx])
            item["rasgo_roi"] = analyze_roi_feature(r)
        metrics.append(item)
    return metrics


# -----------------------------------------------------------------------------
# Figures
# -----------------------------------------------------------------------------
def base_layout(fig, x_title, y_title):
    fig.update_layout(
        template="plotly_white",
        margin=dict(l=45, r=15, t=22, b=42),
        height=240,
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="#FFFFFF",
        font=dict(family="Arial, sans-serif", size=11, color="#334155"),
        legend=dict(orientation="h", yanchor="bottom", y=1.01, xanchor="right", x=1, font=dict(size=9)),
        hovermode="x unified",
    )
    fig.update_xaxes(title=x_title, gridcolor="#EEF2F7", zeroline=False, linecolor="#CBD5E1")
    fig.update_yaxes(title=y_title, gridcolor="#EEF2F7", zeroline=False, linecolor="#CBD5E1")
    return fig


def add_roi_feature_marker(fig, record, color, label):
    feature = analyze_roi_feature(record)
    if feature.get("tipo") not in ("pico", "meseta"):
        return

    # Los rasgos se marcan de forma discreta y abierta para no crear
    # cuadros sólidos en la gráfica ni elementos redundantes en la leyenda.
    symbol = "diamond-open" if feature["tipo"] == "pico" else "square-open"
    fig.add_trace(go.Scatter(
        x=[feature["vset_centro_V"]],
        y=[feature["corriente_uA"]],
        mode="markers",
        marker=dict(
            size=11,
            symbol=symbol,
            color=color,
            line=dict(color=color, width=2),
        ),
        name=f"{feature['tipo'].capitalize()} {label}",
        showlegend=False,
        hovertemplate=(
            f"{feature['tipo'].capitalize()} {label}<br>Vset: %{{x:.3f}} V"
            f"<br>Corriente: %{{y:.3g}} uA<br>Ancho/extensión: {feature['ancho_V']:.3f} V<extra></extra>"
        ),
    ))


def make_ca_figure(records):
    fig = go.Figure()
    blank = get_record(records, "CA", "blanco")
    cu = get_record(records, "CA", "cobre")
    if blank:
        fig.add_trace(go.Scatter(x=blank["x"], y=record_signal(blank), mode="lines", name="CA Blanco filtrado", line=dict(color=BLUE, width=2)))
    if cu:
        fig.add_trace(go.Scatter(x=cu["x"], y=record_signal(cu), mode="lines", name="CA Cu²⁺ filtrado", line=dict(color=YELLOW, width=2)))
    return base_layout(fig, "Tiempo (s)", "Corriente filtrada (uA)")


def make_lsv_figure(records):
    fig = go.Figure()
    line_defs = [(1, YELLOW, "solid"), (2, "#334155", "solid"), (3, BLUE_2, "dash")]
    for rep, color, dash in line_defs:
        r = get_record(records, "LSV", "cobre", rep)
        if r:
            fig.add_trace(go.Scatter(x=r["x"], y=record_signal(r), mode="lines", name=f"LSV {rep} filtrada", line=dict(color=color, width=2, dash=dash)))
            add_roi_feature_marker(fig, r, color, f"LSV {rep}")
    fig.add_vrect(x0=ROI_VSET_MIN, x1=ROI_VSET_MAX, fillcolor="#F5B400", opacity=0.08, line_width=0, annotation_text="Región analizada 0.2–0.6 V", annotation_position="top left")
    return base_layout(fig, "Vset (V)", "Corriente filtrada (uA)")


def make_cv_figure(records):
    fig = go.Figure()
    blank = get_record(records, "CV", "blanco")
    cv = get_record(records, "CV", "cobre")
    if blank:
        fig.add_trace(go.Scatter(x=blank["x"], y=record_signal(blank), mode="lines", name="CV Blanco filtrado", line=dict(color=BLUE, width=2.2)))
        add_roi_feature_marker(fig, blank, BLUE, "CV blanco")
    if cv:
        fig.add_trace(go.Scatter(x=cv["x"], y=record_signal(cv), mode="lines", name="CV Cu²⁺ filtrado", line=dict(color=YELLOW, width=2.2)))
        add_roi_feature_marker(fig, cv, YELLOW, "CV con Cu")
    fig.add_vrect(x0=ROI_VSET_MIN, x1=ROI_VSET_MAX, fillcolor="#F5B400", opacity=0.08, line_width=0, annotation_text="Región analizada 0.2–0.6 V", annotation_position="top left")
    return base_layout(fig, "Vset (V)", "Corriente filtrada (uA)")


def make_lsv_mean_figure(records):
    fig = go.Figure()
    curves = [get_record(records, "LSV", "cobre", rep) for rep in (1, 2, 3)]
    curves = [curve for curve in curves if curve]
    if not curves:
        return base_layout(fig, "Vset (V)", "Corriente filtrada (uA)")
    xmin = max(min(curve["x"]) for curve in curves)
    xmax = min(max(curve["x"]) for curve in curves)
    grid = np.linspace(xmin, xmax, 300)
    aligned = []
    for curve in curves:
        x = np.asarray(curve["x"], dtype=float)
        y = record_signal(curve)
        order = np.argsort(x)
        aligned.append(np.interp(grid, x[order], y[order]))
    matrix = np.vstack(aligned)
    mean = np.nanmean(matrix, axis=0)
    std = np.nanstd(matrix, axis=0)
    fig.add_trace(go.Scatter(x=grid, y=mean + std, mode="lines", line=dict(width=0), showlegend=False, hoverinfo="skip"))
    fig.add_trace(go.Scatter(x=grid, y=mean - std, mode="lines", fill="tonexty", fillcolor="rgba(100,116,139,.22)", line=dict(width=0), name="± 1 DE"))
    fig.add_trace(go.Scatter(x=grid, y=mean, mode="lines", name="Promedio LSV", line=dict(color="#111111", width=3)))
    roi_indices = np.where((grid >= ROI_VSET_MIN) & (grid <= ROI_VSET_MAX))[0]
    if len(roi_indices):
        peak = int(roi_indices[np.nanargmax(mean[roi_indices])])
        fig.add_vline(x=float(grid[peak]), line_dash="dash", line_color="#EF6B6B", annotation_text=f"Máximo ROI {grid[peak]:.3f} V")
    fig.add_vrect(x0=ROI_VSET_MIN, x1=ROI_VSET_MAX, fillcolor="#F5B400", opacity=0.08, line_width=0)
    return base_layout(fig, "Vset (V)", "Corriente filtrada (uA)")
def make_general_evolution_figure(records):
    """
    Superpone la evolución voltamétrica general de la celda
    únicamente entre Vset = 0.0 y 0.6 V.

    Incluye:
    - CV Blanco
    - CV con Cu²+
    - LSV 1
    - LSV 2
    - LSV 3
    - Promedio de las LSV

    CA no se incluye porque se analiza contra tiempo.
    """
    fig = go.Figure()

    VMIN = 0.0
    VMAX = 0.6

    def add_record_segments(record, name, color, dash="solid", width=2.0):
        if not record:
            return

        x = np.asarray(record.get("x", []), dtype=float)
        y = record_signal(record)

        size = min(len(x), len(y))
        x = x[:size]
        y = y[:size]

        valid = (
            np.isfinite(x)
            & np.isfinite(y)
            & (x >= VMIN)
            & (x <= VMAX)
        )

        indices = np.where(valid)[0]
        if not len(indices):
            return

        # Evita unir artificialmente ramas distintas del CV.
        groups = np.split(
            indices,
            np.where(np.diff(indices) > 1)[0] + 1
        )

        first = True

        for group in groups:
            if len(group) < 3:
                continue

            fig.add_trace(go.Scatter(
                x=x[group],
                y=y[group],
                mode="lines",
                name=name,
                legendgroup=name,
                showlegend=first,
                line=dict(
                    color=color,
                    width=width,
                    dash=dash,
                ),
                hovertemplate=(
                    f"{name}"
                    "<br>Vset: %{x:.3f} V"
                    "<br>Corriente: %{y:.3g} uA"
                    "<extra></extra>"
                ),
            ))

            first = False

    # ---------------------------------------------------------
    # CV
    # ---------------------------------------------------------
    cv_blank = get_record(records, "CV", "blanco")
    cv_cu = get_record(records, "CV", "cobre")

    add_record_segments(
        cv_blank,
        "CV Blanco",
        BLUE,
        "solid",
        2.4,
    )

    add_record_segments(
        cv_cu,
        "CV Cu²⁺",
        YELLOW,
        "solid",
        2.4,
    )

    # ---------------------------------------------------------
    # LSV individuales
    # ---------------------------------------------------------
    lsv_definitions = [
        (1, "#334155", "solid"),
        (2, BLUE_2, "dash"),
        (3, "#64748B", "dot"),
    ]

    lsv_records = []

    for rep, color, dash in lsv_definitions:
        record = get_record(records, "LSV", "cobre", rep)

        if record:
            lsv_records.append(record)

        add_record_segments(
            record,
            f"LSV {rep}",
            color,
            dash,
            1.8,
        )

    # ---------------------------------------------------------
    # Promedio LSV
    # ---------------------------------------------------------
    if len(lsv_records) >= 2:

        grid = np.linspace(VMIN, VMAX, 250)
        aligned = []

        for record in lsv_records:
            x = np.asarray(record.get("x", []), dtype=float)
            y = record_signal(record)

            size = min(len(x), len(y))
            x = x[:size]
            y = y[:size]

            valid = np.isfinite(x) & np.isfinite(y)

            x = x[valid]
            y = y[valid]

            order = np.argsort(x)
            x = x[order]
            y = y[order]

            # Evita problemas de interpolación por Vset repetido.
            x_unique, unique_idx = np.unique(x, return_index=True)
            y_unique = y[unique_idx]

            if len(x_unique) >= 2:
                aligned.append(
                    np.interp(grid, x_unique, y_unique)
                )

        if len(aligned) >= 2:
            mean_lsv = np.nanmean(
                np.vstack(aligned),
                axis=0
            )

            fig.add_trace(go.Scatter(
                x=grid,
                y=mean_lsv,
                mode="lines",
                name="Promedio LSV",
                line=dict(
                    color="#111111",
                    width=3.2,
                ),
                hovertemplate=(
                    "Promedio LSV"
                    "<br>Vset: %{x:.3f} V"
                    "<br>Corriente: %{y:.3g} uA"
                    "<extra></extra>"
                ),
            ))

    fig = base_layout(
        fig,
        "Vset (V)",
        "Corriente filtrada (uA)",
    )

    fig.update_xaxes(
        range=[VMIN, VMAX],
        dtick=0.1,
    )

    fig.update_layout(
        height=330,
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="left",
            x=0,
            font=dict(size=9),
        ),
        margin=dict(
            l=55,
            r=20,
            t=65,
            b=50,
        ),
    )

    return fig



def make_pdf_chart_png(records, chart_kind: str) -> io.BytesIO:
    """Genera las figuras estáticas del informe con Matplotlib, sin Kaleido/Chrome."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    records = records or []
    fig, ax = plt.subplots(figsize=(9.5, 4.1), dpi=140)

    def finish(xlabel: str, ylabel: str, xlim=None):
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        if xlim is not None:
            ax.set_xlim(*xlim)
        ax.grid(True, alpha=0.22, linewidth=0.7)
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(
                loc="upper center",
                bbox_to_anchor=(0.5, 1.16),
                ncol=min(4, max(1, len(labels))),
                frameon=False,
                fontsize=8,
            )
        fig.tight_layout(rect=[0, 0, 1, 0.91])
        image = io.BytesIO()
        fig.savefig(image, format="png", dpi=160, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        image.seek(0)
        return image

    def plot_record(record, label, color, linestyle="-", linewidth=1.8, mask=None):
        if not record:
            return
        x = np.asarray(record.get("x", []), dtype=float)
        y = record_signal(record)
        size = min(len(x), len(y))
        x, y = x[:size], y[:size]
        valid = np.isfinite(x) & np.isfinite(y)
        if mask is not None:
            valid &= mask(x)
        indices = np.where(valid)[0]
        if not len(indices):
            return
        groups = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        first = True
        for group in groups:
            if len(group) < 2:
                continue
            ax.plot(
                x[group],
                y[group],
                label=label if first else None,
                color=color,
                linestyle=linestyle,
                linewidth=linewidth,
            )
            first = False

    if chart_kind == "ca":
        plot_record(get_record(records, "CA", "blanco"), "CA Blanco filtrado", BLUE, "-", 2.0)
        plot_record(get_record(records, "CA", "cobre"), "CA Cu²⁺ filtrado", YELLOW, "-", 2.0)
        return finish("Tiempo (s)", "Corriente filtrada (uA)")

    if chart_kind == "cv":
        plot_record(get_record(records, "CV", "blanco"), "CV Blanco filtrado", BLUE, "-", 2.0)
        plot_record(get_record(records, "CV", "cobre"), "CV Cu²⁺ filtrado", YELLOW, "-", 2.0)
        ax.axvspan(ROI_VSET_MIN, ROI_VSET_MAX, alpha=0.08)
        return finish("Vset (V)", "Corriente filtrada (uA)")

    if chart_kind == "lsv":
        definitions = [
            (1, YELLOW, "-"),
            (2, "#334155", "-"),
            (3, BLUE_2, "--"),
        ]
        for rep, color, linestyle in definitions:
            plot_record(get_record(records, "LSV", "cobre", rep), f"LSV {rep} filtrada", color, linestyle, 1.9)
        ax.axvspan(ROI_VSET_MIN, ROI_VSET_MAX, alpha=0.08)
        return finish("Vset (V)", "Corriente filtrada (uA)")

    if chart_kind == "lsv_mean":
        curves = [get_record(records, "LSV", "cobre", rep) for rep in (1, 2, 3)]
        curves = [curve for curve in curves if curve]
        if curves:
            xmin = max(min(curve["x"]) for curve in curves)
            xmax = min(max(curve["x"]) for curve in curves)
            grid = np.linspace(xmin, xmax, 300)
            aligned = []
            for curve in curves:
                x = np.asarray(curve["x"], dtype=float)
                y = record_signal(curve)
                size = min(len(x), len(y))
                x, y = x[:size], y[:size]
                valid = np.isfinite(x) & np.isfinite(y)
                x, y = x[valid], y[valid]
                order = np.argsort(x)
                x, y = x[order], y[order]
                x_unique, unique_idx = np.unique(x, return_index=True)
                y_unique = y[unique_idx]
                if len(x_unique) >= 2:
                    aligned.append(np.interp(grid, x_unique, y_unique))
            if aligned:
                matrix = np.vstack(aligned)
                mean = np.nanmean(matrix, axis=0)
                std = np.nanstd(matrix, axis=0)
                ax.fill_between(grid, mean - std, mean + std, alpha=0.20, label="± 1 DE")
                ax.plot(grid, mean, color="#111111", linewidth=2.4, label="Promedio LSV")
                roi_indices = np.where((grid >= ROI_VSET_MIN) & (grid <= ROI_VSET_MAX))[0]
                if len(roi_indices):
                    peak = int(roi_indices[np.nanargmax(mean[roi_indices])])
                    ax.axvline(float(grid[peak]), linestyle="--", linewidth=1.2, alpha=0.8)
        ax.axvspan(ROI_VSET_MIN, ROI_VSET_MAX, alpha=0.08)
        return finish("Vset (V)", "Corriente filtrada (uA)")

    if chart_kind == "general":
        vmin, vmax = 0.0, 0.6
        roi_mask = lambda x: (x >= vmin) & (x <= vmax)
        plot_record(get_record(records, "CV", "blanco"), "CV Blanco", BLUE, "-", 2.0, roi_mask)
        plot_record(get_record(records, "CV", "cobre"), "CV Cu²⁺", YELLOW, "-", 2.0, roi_mask)

        definitions = [
            (1, "#334155", "-"),
            (2, BLUE_2, "--"),
            (3, "#64748B", ":"),
        ]
        lsv_records = []
        for rep, color, linestyle in definitions:
            record = get_record(records, "LSV", "cobre", rep)
            if record:
                lsv_records.append(record)
            plot_record(record, f"LSV {rep}", color, linestyle, 1.6, roi_mask)

        if len(lsv_records) >= 2:
            grid = np.linspace(vmin, vmax, 250)
            aligned = []
            for record in lsv_records:
                x = np.asarray(record.get("x", []), dtype=float)
                y = record_signal(record)
                size = min(len(x), len(y))
                x, y = x[:size], y[:size]
                valid = np.isfinite(x) & np.isfinite(y)
                x, y = x[valid], y[valid]
                order = np.argsort(x)
                x, y = x[order], y[order]
                x_unique, unique_idx = np.unique(x, return_index=True)
                y_unique = y[unique_idx]
                if len(x_unique) >= 2:
                    aligned.append(np.interp(grid, x_unique, y_unique))
            if len(aligned) >= 2:
                mean = np.nanmean(np.vstack(aligned), axis=0)
                ax.plot(grid, mean, color="#111111", linewidth=2.5, label="Promedio LSV")

        ax.set_xticks(np.arange(0.0, 0.61, 0.1))
        return finish("Vset (V)", "Corriente filtrada (uA)", (vmin, vmax))

    plt.close(fig)
    raise ValueError(f"Tipo de gráfica PDF no reconocido: {chart_kind}")

# -----------------------------------------------------------------------------
# UI helpers
# -----------------------------------------------------------------------------
def metric_card(icon, label, value, note, note_class=""):
    return html.Div(
        className="metric-card",
        children=[
            html.Div(icon, className="metric-icon"),
            html.Div([
                html.Div(label, className="metric-label"),
                html.Div(value, className="metric-value"),
                html.Div(note, className=f"metric-note {note_class}"),
            ])
        ],
    )


def protocol_card(label, complete):
    return html.Div(
        className=f"protocol-card {'complete' if complete else 'missing'}",
        children=[
            html.Div([
                html.Div("⌁", className="protocol-mini-icon"),
                html.Div(label, className="protocol-name"),
            ], className="protocol-card-top"),
            html.Div([
                html.Span("Completado" if complete else "Faltante"),
                html.Span("✓" if complete else "!", className="status-dot"),
            ], className="protocol-status"),
        ],
    )


def chart_card(title, graph_id, figure):
    return html.Div(
        className="chart-card",
        style={"minHeight": "315px", "overflow": "visible"},
        children=[
            html.Div([html.Span(title), html.Span("⋮", className="chart-menu")], className="chart-title"),
            dcc.Graph(id=graph_id, figure=figure, style={"height": "270px", "width": "100%"}, config={"displayModeBar": False, "responsive": True}),
        ],
    )


def recent_cell(name, ppm, when, status="Incompleta", researcher="Sin asignar", deletable=False):
    delete_control = []
    if deletable:
        delete_control = [
            html.Button("Ver", id={"type": "select-cell", "index": name}, n_clicks=0, title=f"Abrir {name}", style={
                "border": "1px solid #0B438D", "borderRadius": "6px", "background": "white",
                "color": "#0B438D", "fontSize": "11px", "cursor": "pointer", "padding": "4px 7px"
            }),
            html.Button("Descargar", id={"type": "download-cell", "index": name}, n_clicks=0, title=f"Descargar {name} en ZIP", style={
                "border": "1px solid #0B438D", "borderRadius": "6px", "background": "#0B438D",
                "color": "white", "fontSize": "11px", "cursor": "pointer", "padding": "4px 7px"
            }),
            dcc.ConfirmDialogProvider(
            id={"type": "delete-cell", "index": name},
            message=f"¿Deseas eliminar {name}? Esta acción no se puede deshacer.",
            children=html.Button("×", title=f"Eliminar {name}", style={
                "border": "none", "background": "transparent", "color": "#DC2626",
                "fontSize": "22px", "cursor": "pointer", "padding": "0 0 0 8px"
            }),
            )
        ]
    return html.Div(className="recent-row", children=[
        html.Img(src="/assets/icono.png", className="recent-icon"),
        html.Div([html.Div(name, className="recent-name"), html.Div(status, className="recent-status"), html.Div(researcher, className="recent-time")], className="recent-main"),
        html.Div([html.Span(ppm, className="ppm-chip small"), html.Div(when, className="recent-time")]),
        *delete_control,
    ])


EMPTY_RECORDS: list[dict[str, Any]] = []
EMPTY_CA = make_ca_figure(EMPTY_RECORDS)
EMPTY_LSV = make_lsv_figure(EMPTY_RECORDS)
EMPTY_CV = make_cv_figure(EMPTY_RECORDS)
INITIAL_DATABASE = load_database()
INITIAL_CELL = INITIAL_DATABASE["cells"][-1] if INITIAL_DATABASE.get("cells") else None
INITIAL_RECORDS = INITIAL_CELL.get("records", []) if INITIAL_CELL else []
INITIAL_CELL_ID = INITIAL_CELL.get("id") if INITIAL_CELL else None

app.layout = html.Div(className="app-shell", children=[
    dcc.Store(id="records-store", data=INITIAL_RECORDS),
    dcc.Store(id="database-store", data=INITIAL_DATABASE),
    dcc.Interval(
    id="reload-database-on-start",
    interval=800,
    n_intervals=0,
    max_intervals=1,
    ),
    dcc.Store(id="current-cell-store", data=INITIAL_CELL_ID),
    dcc.Store(id="report-ready-store", data=bool(INITIAL_CELL and INITIAL_CELL.get("report_generated"))),
    dcc.Store(id="draft-mode-store", data=False),
    dcc.Store(id="analysis-store", data=""),
    dcc.Store(id="pdf-content-store", data=None),
    dcc.Store(id="active-page-store", data="cells"),
    dcc.Download(id="pdf-download"),
    dcc.Download(id="cell-zip-download"),
    html.Header(className="topbar", children=[
        html.Div(className="brand-wrap", children=[
            html.Img(src="/assets/sensa_logo_header.png", className="brand-logo"),
            html.Div(className="brand-divider"),
            html.Div([html.Span("SENSA ", className="brand-module-main"), html.Span("Cells", className="brand-module-accent")], className="brand-module"),
        ]),
        html.Nav(className="nav-links", children=[
            html.Div([html.Span("⌂"), " Inicio"], id="nav-home", n_clicks=0, className="nav-item", style={"cursor": "pointer"}),
            html.Div([html.Span("◉"), " Celdas"], id="nav-cells", n_clicks=0, className="nav-item active", style={"cursor": "pointer"}),
            html.Div([html.Span("⌁"), " Comparación"], id="nav-comparison", n_clicks=0, className="nav-item", style={"cursor": "pointer"}),
            html.Div([html.Span("⌇"), " Análisis"], id="nav-analysis", n_clicks=0, className="nav-item", style={"cursor": "pointer"}),
            html.Div([html.Span("✧"), " Machine Learning"], id="nav-ml", n_clicks=0, className="nav-item", style={"cursor": "pointer"}),
        ]),
        html.Div(id="profile-button", n_clicks=0, className="profile", style={"cursor": "pointer"}, children=[
            html.Div("--", id="profile-initials", className="avatar"),
            html.Div([html.Div("Seleccionar investigador", id="profile-name", className="profile-name"), html.Div("SENSA", id="profile-role", className="profile-role")]),
            html.Div("⌄", className="profile-chevron"),
        ]),
    ]),

    html.Div(id="researcher-panel", style={
        "display": "none", "position": "fixed", "right": "20px", "top": "84px",
        "zIndex": 2000, "width": "360px", "maxWidth": "calc(100vw - 32px)", "padding": "24px", "background": "white",
        "boxSizing": "border-box", "overflow": "visible", "border": "1px solid #DCE4EF", "borderRadius": "14px", "boxShadow": "0 16px 40px rgba(15,23,42,.18)"
    }, children=[
        html.H3("Investigador activo", style={"marginTop": 0}),
        html.P("Selecciona quién está analizando la celda.", style={"color": MUTED, "fontSize": "13px"}),
        dcc.Dropdown(
            id="researcher-dropdown",
            options=[{"label": name, "value": name} for name in INITIAL_DATABASE.get("researchers", [])],
            value=INITIAL_DATABASE.get("current_researcher"),
            placeholder="Seleccionar investigador",
            clearable=False,
            className="sensa-dropdown",
        ),
        html.Div(style={"height": "14px"}),
        html.Label("Registrar nuevo investigador", style={"display": "block", "marginBottom": "7px", "fontWeight": "600", "color": INK}),
        dcc.Input(id="new-researcher-name", placeholder="Nombre y apellido", style={
            "width": "100%", "height": "44px", "boxSizing": "border-box", "padding": "0 14px", "margin": "0",
            "fontSize": "14px", "lineHeight": "22px", "color": INK, "background": "#FFFFFF",
            "border": "1px solid #CBD5E1", "borderRadius": "9px"
        }),
        html.Button("Agregar investigador", id="add-researcher-btn", n_clicks=0, className="primary-action", style={
            "width": "100%", "marginTop": "14px", "height": "46px", "borderRadius": "9px", "fontSize": "15px"
        }),
        html.Div(id="researcher-status", style={"fontSize": "12px", "marginTop": "8px", "color": GREEN}),
    ]),

    html.Main(id="cells-page", className="page", children=[
        html.Section(className="page-heading", children=[
            html.H1("Gestión y análisis visual de celdas electroquímicas"),
            html.P("Carga, organiza y analiza tus celdas con visualizaciones avanzadas e inteligencia artificial."),
        ]),

        html.Section(className="metrics-grid", children=[
            metric_card("⌁", "Celdas registradas", html.Span("0", id="metric-cells"), "Total real guardado"),
            metric_card("✣", "Protocolos completos", html.Span("0", id="metric-complete"), "Celdas 7/7 completadas", "green-note"),
            metric_card("≋", "Concentraciones", html.Span("0", id="metric-concentrations"), html.Span("Sin datos", id="metric-concentration-note")),
            metric_card("▧", "Informes PDF", html.Span("0", id="metric-reports"), "Generados realmente"),
        ]),

        html.Section(className="dashboard-grid", children=[
            # Left column
            html.Aside(className="left-column", children=[
                html.Div(className="panel upload-panel", children=[
                    html.Div(className="upload-dropzone", children=[
                        html.H3("Crear nueva celda"),
                        html.Div("＋", className="upload-cloud"),
                        html.P("Primero crea la celda. Después carga el blanco y la adición de cobre en sus bloques."),
                        html.Button("＋ Nueva celda", id="new-cell-btn", n_clicks=0, className="upload-button", style={"border": "none", "cursor": "pointer"}),
                    ]),
                    html.Div(id="upload-status", className="upload-status"),
                ]),

                html.Div(className="panel filters-panel", children=[
                    html.Div([html.H3("Datos de la celda")], className="panel-title-row"),
                    html.Label("Material del electrodo"),
                    dcc.Dropdown(id="cell-material", options=["PAN", "PAN-Ag", "Grafito", "Otro"], placeholder="Seleccionar material", clearable=False, className="sensa-dropdown"),
                    html.Label("Electrolito de soporte"),
                    dcc.Dropdown(id="support-electrolyte", options=["KCl", "NaCl", "KNO3", "Otro"], placeholder="Seleccionar electrolito", className="sensa-dropdown"),
                    html.Div(className="filter-two", children=[
                        html.Div([html.Label("Concentración"), dcc.Input(id="electrolyte-concentration", type="number", min=0, step="any", placeholder="Valor real", className="date-input")]),
                        html.Div([html.Label("Unidad"), dcc.Dropdown(id="electrolyte-unit", options=["M", "mM"], placeholder="Unidad", className="sensa-dropdown")]),
                    ]),
                    html.Label("Volumen inicial de electrolito (mL)"),
                    dcc.Input(id="initial-volume-ml", type="number", min=0, step="any", placeholder="Volumen real", className="date-input", style={"width": "100%"}),
                    html.Label("Solución de Cu añadida"),
                    html.Div(className="filter-two", children=[
                        html.Div([html.Label("Concentración (ppm)"), dcc.Input(id="copper-stock-ppm", type="number", min=0, step="any", placeholder="ppm reales", className="date-input")]),
                        html.Div([html.Label("Volumen (mL)"), dcc.Input(id="copper-added-ml", type="number", min=0, step="any", placeholder="Volumen real", className="date-input")]),
                    ]),
                    html.Div(id="composition-summary", style={"margin": "10px 0", "padding": "10px", "background": "#F0F7FF", "borderRadius": "8px", "fontSize": "12px", "color": BLUE}),
                    html.Label("Observaciones"),
                    dcc.Textarea(id="cell-observations", placeholder="Observaciones de la preparación o del ensayo", style={"width": "100%", "minHeight": "85px", "boxSizing": "border-box", "border": "1px solid #CBD5E1", "borderRadius": "8px", "padding": "10px"}),
                    html.Div("El investigador y la fecha se registran automáticamente.", className="metric-note", style={"marginTop": "10px"}),
                ]),
            ]),

            # Main column
            html.Section(className="main-column", children=[
                html.Div(className="panel cell-panel", children=[
                    html.Div(className="cell-head", children=[
                        html.Div(className="cell-title-wrap", children=[
                            html.Img(src="/assets/icono.png", className="cell-icon"),
                            html.H2(id="cell-title", children="SIN CELDA CARGADA"),
                        ]),
                        html.Div(className="cell-badges", children=[
                            html.Span("Protocolo incompleto 0/7", id="completion-badge", className="incomplete-badge"),
                            html.Span("Sin ppm", id="concentration-badge", className="ppm-chip"),
                            html.Span("⋮", className="cell-menu"),
                        ]),
                    ]),

                    html.Div(className="protocol-grid", children=[
                        html.Div(className="protocol-block", children=[
                            html.Div([html.H3("Blanco"), html.Span(id="blank-count", children="0/2")], className="protocol-heading"),
                            html.Div(id="blank-protocol", className="protocol-cards", children=[protocol_card("CA Blanco", False), protocol_card("CV Blanco", False)]),
                            dcc.Upload(id="upload-blank", multiple=True, children=html.Button("Cargar CA + CV del blanco", className="secondary-action", style={"width": "100%", "marginTop": "10px"})),
                            html.Div(id="blank-upload-status", className="upload-status"),
                        ]),
                        html.Div(className="protocol-block copper", children=[
                            html.Div([html.H3("Adición de Cu²⁺"), html.Span(id="copper-count", children="0/5")], className="protocol-heading"),
                            html.Div(id="copper-protocol", className="protocol-cards copper-cards", children=[
                                protocol_card("CA Cu²⁺", False), protocol_card("CV Cu²⁺", False), protocol_card("LSV 1", False), protocol_card("LSV 2", False), protocol_card("LSV 3", False)
                            ]),
                            html.Div(style={"display": "flex", "gap": "8px", "alignItems": "center", "marginTop": "10px"}, children=[
                                dcc.Input(id="cell-concentration", type="number", min=0, step="any", placeholder="Cu final", readOnly=True, style={"width": "125px", "padding": "10px", "background": "#F8FAFC", "border": "1px solid #CBD5E1", "borderRadius": "8px"}),
                                html.Span("ppm finales", style={"color": MUTED}),
                                dcc.Upload(id="upload-copper", multiple=True, children=html.Button("Cargar CA + CV + 3 LSV", className="secondary-action")),
                            ]),
                            html.Div(id="copper-upload-status", className="upload-status"),
                        ]),
                    ]),
                ]),

                html.Div(className="visual-title", children="Visualización y comparación"),
                html.Div(className="charts-grid", children=[
                    chart_card("CA Blanco vs CA Cu²⁺", "ca-chart", EMPTY_CA),
                    chart_card("LSV1, LSV2, LSV3 con Cu²⁺", "lsv-chart", EMPTY_LSV),
                    chart_card("CV Blanco vs CV Cu²⁺", "cv-chart", EMPTY_CV),
                ]),

                html.Div(className="panel ai-panel", children=[
                    html.Div(className="ai-copy", children=[
                        html.Div("✧", className="ai-icon"),
                        html.Div([html.H3("Informe electroquímico automático"), html.P("Analiza tus datos con IA y genera informes profesionales con interpretaciones, comparaciones y conclusiones.")]),
                    ]),
                    html.Button("✧  Analizar con IA", id="analyze-btn", className="primary-action", n_clicks=0),
                    html.Button("▧  Generar informe PDF", id="pdf-btn", className="secondary-action", n_clicks=0),
                ]),
                dcc.Loading(html.Div(id="analysis-result", className="analysis-result"), type="circle"),
                html.Div(className="panel", style={"padding": "18px", "marginTop": "14px", "display": "flex", "alignItems": "center", "justifyContent": "space-between", "gap": "16px"}, children=[
                    html.Div([html.H3("Finalizar celda", style={"margin": "0 0 5px"}), html.Div("Requiere protocolo 7/7 e informe PDF generado.", className="metric-note")]),
                    html.Div(style={"display": "flex", "gap": "10px"}, children=[
                        html.Button("Descartar borrador", id="discard-draft-btn", n_clicks=0, className="secondary-action"),
                        html.Button("Guardar celda", id="save-cell-btn", n_clicks=0, className="primary-action"),
                    ]),
                ]),
                html.Div(id="save-cell-status", className="upload-status"),
            ]),

            # Right column
            html.Aside(className="right-column", children=[
                html.Div(className="panel recent-panel", children=[
                    html.H3("Celdas guardadas"),
                    dcc.Dropdown(id="saved-researcher-filter", placeholder="Todos los investigadores", clearable=True, className="sensa-dropdown"),
                    dcc.Dropdown(id="saved-concentration-filter", placeholder="Todas las concentraciones", clearable=True, className="sensa-dropdown", style={"marginTop": "7px"}),
                    dcc.Dropdown(id="saved-material-filter", placeholder="Todos los materiales", clearable=True, className="sensa-dropdown", style={"marginTop": "7px"}),
                    dcc.DatePickerRange(id="saved-date-filter", display_format="DD/MM/YYYY", start_date_placeholder_text="Desde", end_date_placeholder_text="Hasta", style={"marginTop": "7px", "marginBottom": "10px"}),
                    html.Div(id="recent-cells", children=html.Div("Aún no hay celdas registradas.", className="metric-note")),
                ]),
                html.Div(className="panel concentration-panel", children=[
                    html.H3("Concentraciones"),
                    html.Div(id="concentration-summary", children=html.Div("Sin datos reales.", className="metric-note")),
                ]),
            ]),
        ]),
    ]),

    html.Main(id="home-page", className="page", style={"display": "none"}, children=[
        html.Section(className="page-heading", children=[html.H1("SENSA Cells"), html.P("Plataforma para registrar, comparar e interpretar la evolución de celdas electroquímicas.")]),
        html.Div(id="home-summary", className="metrics-grid"),
        html.Div(className="panel", style={"padding": "28px", "marginTop": "18px"}, children=[
            html.H2("Flujo de trabajo"),
            html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(4, 1fr)", "gap": "14px"}, children=[
                html.Div([html.H3("1. Crear"), html.P("Registra investigador, material y concentración.")], className="metric-card"),
                html.Div([html.H3("2. Cargar"), html.P("CA + CV del blanco y CA + CV + 3 LSV con cobre.")], className="metric-card"),
                html.Div([html.H3("3. Analizar"), html.P("Visualiza, superpone y genera el informe técnico.")], className="metric-card"),
                html.Div([html.H3("4. Guardar"), html.P("Conserva la celda completa para futuras comparaciones.")], className="metric-card"),
            ]),
        ]),
    ]),

    html.Main(id="comparison-page", className="page", style={"display": "none"}, children=[
        html.Section(className="page-heading", children=[html.H1("Comparación y evolución electroquímica"), html.P("Selecciona una celda guardada para revisar CA, CV, las tres LSV y su repetibilidad.")]),
        html.Div(className="panel", style={"padding": "18px"}, children=[
            dcc.Dropdown(id="comparison-cell-select", placeholder="Seleccionar celda guardada", clearable=False, className="sensa-dropdown"),
            html.H2(id="comparison-title", style={"textAlign": "center", "margin": "22px 0 8px"}),
            html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(2, minmax(0, 1fr))", "gap": "16px"}, children=[
                chart_card("CA antes y después de añadir Cu", "comparison-ca", EMPTY_CA),
                chart_card("CV antes y después de añadir Cu", "comparison-cv", EMPTY_CV),
                chart_card("Tres LSV después de añadir Cu", "comparison-lsv", EMPTY_LSV),
                chart_card("Promedio y repetibilidad de LSV", "comparison-lsv-mean", make_lsv_mean_figure([])),
            ]),
        ]),
    ]),

    html.Main(id="analysis-page", className="page", style={"display": "none"}, children=[
        html.Section(className="page-heading", children=[html.H1("Biblioteca de análisis"), html.P("Consulta el análisis y el informe asociado a cada celda guardada.")]),
        html.Div(className="panel", style={"padding": "20px"}, children=[
            dcc.Dropdown(id="analysis-cell-select", placeholder="Seleccionar celda guardada", clearable=False, className="sensa-dropdown"),
            html.Div(id="saved-analysis-content", style={"marginTop": "18px"}),
        ]),
    ]),

    html.Main(id="ml-page", className="page", style={"display": "none"}, children=[
        html.Section(className="page-heading", children=[html.H1("Machine Learning"), html.P("Preparación de variables y predicción de concentración de cobre.")]),
        html.Div(className="panel", style={"padding": "30px", "textAlign": "center"}, children=[html.H2("Módulo preparado para la siguiente fase"), html.P("Las celdas guardadas alimentarán el conjunto de datos con picos, áreas, ruido, relación señal/ruido y repetibilidad.")]),
    ]),
])


# -----------------------------------------------------------------------------
# Callbacks
# -----------------------------------------------------------------------------
@app.callback(
    Output("active-page-store", "data"),
    Output("home-page", "style"), Output("cells-page", "style"), Output("comparison-page", "style"), Output("analysis-page", "style"), Output("ml-page", "style"),
    Output("nav-home", "className"), Output("nav-cells", "className"), Output("nav-comparison", "className"), Output("nav-analysis", "className"), Output("nav-ml", "className"),
    Input("nav-home", "n_clicks"), Input("nav-cells", "n_clicks"), Input("nav-comparison", "n_clicks"), Input("nav-analysis", "n_clicks"), Input("nav-ml", "n_clicks"),
    prevent_initial_call=True,
)
def navigate_pages(*clicks):
    mapping = {"nav-home": "home", "nav-cells": "cells", "nav-comparison": "comparison", "nav-analysis": "analysis", "nav-ml": "ml"}
    active = mapping.get(ctx.triggered_id, "cells")
    pages = ["home", "cells", "comparison", "analysis", "ml"]
    styles = [{"display": "block"} if page == active else {"display": "none"} for page in pages]
    classes = ["nav-item active" if page == active else "nav-item" for page in pages]
    return active, *styles, *classes


@app.callback(
    Output("comparison-cell-select", "options"), Output("comparison-cell-select", "value"),
    Output("analysis-cell-select", "options"), Output("analysis-cell-select", "value"),
    Output("home-summary", "children"),
    Input("database-store", "data"),
)
def update_secondary_pages(database):
    database = database or empty_database()
    cells = database.get("cells", [])
    options = [{"label": f"{cell.get('id')} - {cell.get('concentration_ppm', 'Sin')} ppm - {cell.get('researcher', 'Sin investigador')}", "value": cell.get("id")} for cell in reversed(cells)]
    value = options[0]["value"] if options else None
    complete = sum(cell.get("complete_count") == 7 for cell in cells)
    concentrations = len({cell.get("concentration_ppm") for cell in cells if cell.get("concentration_ppm") is not None})
    summary = [
        metric_card("⌁", "Celdas guardadas", str(len(cells)), "Registros definitivos"),
        metric_card("✣", "Protocolos completos", str(complete), "Protocolo 7/7", "green-note"),
        metric_card("≋", "Concentraciones", str(concentrations), "Niveles estudiados"),
        metric_card("▧", "Informes", str(database.get("reports_generated", 0)), "Informes generados"),
    ]
    return options, value, options, value, summary


@app.callback(
    Output("comparison-title", "children"),
    Output("comparison-ca", "figure"), Output("comparison-cv", "figure"), Output("comparison-lsv", "figure"), Output("comparison-lsv-mean", "figure"),
    Input("comparison-cell-select", "value"), State("database-store", "data"),
)
def render_saved_comparison(cell_id, database):
    cell = next((item for item in (database or {}).get("cells", []) if item.get("id") == cell_id), None)
    if not cell:
        return "Selecciona una celda guardada", EMPTY_CA, EMPTY_CV, EMPTY_LSV, make_lsv_mean_figure([])
    records = cell.get("records", [])
    title = f"Evolución de {cell_id} - Cu: {float(cell.get('concentration_ppm', 0)):g} ppm"
    return title, make_ca_figure(records), make_cv_figure(records), make_lsv_figure(records), make_lsv_mean_figure(records)


@app.callback(
    Output("saved-analysis-content", "children"),
    Input("analysis-cell-select", "value"), State("database-store", "data"),
)
def render_saved_analysis(cell_id, database):
    cell = next((item for item in (database or {}).get("cells", []) if item.get("id") == cell_id), None)
    if not cell:
        return html.Div("No hay análisis guardados.", className="metric-note")
    return html.Div([
        html.Div([html.Span(cell.get("researcher", "Sin investigador")), html.Span(f"{cell.get('concentration_ppm')} ppm", className="ppm-chip")], className="panel-title-row"),
        dcc.Markdown(local_analysis(cell.get("records", []), cell.get("composition"), cell.get("researcher"))[0]),
        html.Div(f"Informe asociado: {cell.get('report_filename', 'No disponible')}", className="metric-note"),
    ], className="analysis-box")


@app.callback(
    Output("researcher-panel", "style"),
    Input("profile-button", "n_clicks"),
    State("researcher-panel", "style"),
    prevent_initial_call=True,
)
def toggle_researcher_panel(n_clicks, style):
    style = dict(style or {})
    style["display"] = "block" if style.get("display") == "none" else "none"
    return style


@app.callback(
    Output("cell-concentration", "value"),
    Output("composition-summary", "children"),
    Input("support-electrolyte", "value"),
    Input("electrolyte-concentration", "value"),
    Input("electrolyte-unit", "value"),
    Input("initial-volume-ml", "value"),
    Input("copper-stock-ppm", "value"),
    Input("copper-added-ml", "value"),
)
def calculate_cell_composition(electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added):
    values = (electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added)
    if any(value is None for value in values) or float(initial_volume) + float(copper_added) <= 0:
        return None, "Completa todas las cantidades para calcular la composición final."
    final_volume = float(initial_volume) + float(copper_added)
    final_copper = float(copper_stock) * float(copper_added) / final_volume
    text = (
        f"Composición: {float(initial_volume):g} mL de {electrolyte} "
        f"{float(electrolyte_concentration):g} {electrolyte_unit} + "
        f"{float(copper_added):g} mL de Cu a {float(copper_stock):g} ppm. "
        f"Volumen final: {final_volume:g} mL | Cu final estimado: {final_copper:.3g} ppm."
    )
    return round(final_copper, 6), text
@app.callback(
    Output("database-store", "data", allow_duplicate=True),
    Input("reload-database-on-start", "n_intervals"),
    prevent_initial_call=True,
)
def reload_database_on_start(n_intervals):
    if not n_intervals:
        return no_update

    return load_database()

@app.callback(
    Output("database-store", "data", allow_duplicate=True),
    Output("researcher-dropdown", "options"),
    Output("researcher-dropdown", "value"),
    Output("researcher-status", "children"),
    Input("add-researcher-btn", "n_clicks"),
    State("new-researcher-name", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)

def add_researcher(n_clicks, name, database):
    clean_name = " ".join(str(name or "").split())
    database = database or load_database()
    if not clean_name:
        return no_update, no_update, no_update, "Escribe el nombre y apellido."
    researchers = database.setdefault("researchers", [])
    existing = next((item for item in researchers if item.casefold() == clean_name.casefold()), None)
    selected = existing or clean_name
    if not existing:
        researchers.append(clean_name)
        researchers.sort(key=str.casefold)
    database["current_researcher"] = selected
    save_database(database)
    options = [{"label": item, "value": item} for item in researchers]
    return database, options, selected, f"✓ {selected} es el investigador activo."


@app.callback(
    Output("database-store", "data", allow_duplicate=True),
    Input("researcher-dropdown", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def select_researcher(researcher, database):
    if not researcher:
        return no_update
    database = database or load_database()
    if researcher not in database.get("researchers", []):
        return no_update
    database["current_researcher"] = researcher
    save_database(database)
    return database


@app.callback(
    Output("records-store", "data", allow_duplicate=True),
    Output("current-cell-store", "data", allow_duplicate=True),
    Output("analysis-store", "data", allow_duplicate=True),
    Output("analysis-result", "children", allow_duplicate=True),
    Output("cell-material", "value", allow_duplicate=True),
    Output("cell-concentration", "value", allow_duplicate=True),
    Output("cell-observations", "value", allow_duplicate=True),
    Output("support-electrolyte", "value", allow_duplicate=True),
    Output("electrolyte-concentration", "value", allow_duplicate=True),
    Output("electrolyte-unit", "value", allow_duplicate=True),
    Output("initial-volume-ml", "value", allow_duplicate=True),
    Output("copper-stock-ppm", "value", allow_duplicate=True),
    Output("copper-added-ml", "value", allow_duplicate=True),
    Output("draft-mode-store", "data", allow_duplicate=True),
    Output("report-ready-store", "data", allow_duplicate=True),
    Input({"type": "select-cell", "index": ALL}, "n_clicks"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def select_cell(n_clicks, database):
    triggered = ctx.triggered_id
    if not isinstance(triggered, dict) or not any(n_clicks or []):
        return (no_update,) * 15
    cell_id = triggered.get("index")
    database = database or load_database()
    selected = next((cell for cell in database.get("cells", []) if cell.get("id") == cell_id), None)
    if not selected:
        return (no_update,) * 15
    composition = selected.get("composition", {})
    return (
        selected.get("records", []), cell_id, selected.get("analysis", ""), "",
        selected.get("material"), selected.get("concentration_ppm"), selected.get("observations", ""),
        composition.get("electrolyte"), composition.get("electrolyte_concentration"), composition.get("electrolyte_unit"),
        composition.get("initial_volume_ml"), composition.get("copper_stock_ppm"), composition.get("copper_added_ml"),
        False, bool(selected.get("report_generated")),
    )


@app.callback(
    Output("cell-zip-download", "data"),
    Input({"type": "download-cell", "index": ALL}, "n_clicks"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def download_saved_cell(n_clicks, database):
    triggered = ctx.triggered_id
    if not isinstance(triggered, dict) or not any(n_clicks or []):
        return no_update
    cell_id = triggered.get("index")
    cell = next((item for item in (database or {}).get("cells", []) if item.get("id") == cell_id), None)
    if not cell:
        return no_update
    client = get_supabase()
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        if client and cell.get("storage_files"):
            for item in cell.get("storage_files", []):
                zf.writestr(str(item.get("name")), storage_download(client, str(item.get("path"))))
        else:
            for record in cell.get("records", []):
                if record.get("source_b64"):
                    zf.writestr(Path(str(record.get("filename"))).name, base64.b64decode(record["source_b64"]))
        clean_manifest = json.loads(json.dumps(cell))
        for record in clean_manifest.get("records", []):
            record.pop("source_b64", None)
        zf.writestr("manifest.json", json.dumps(clean_manifest, ensure_ascii=False, indent=2).encode("utf-8"))
    return dcc.send_bytes(archive.getvalue(), f"{cell_id}.zip")


@app.callback(
    Output("database-store", "data", allow_duplicate=True),
    Output("records-store", "data", allow_duplicate=True),
    Output("current-cell-store", "data", allow_duplicate=True),
    Input({"type": "delete-cell", "index": ALL}, "submit_n_clicks"),
    State("database-store", "data"),
    State("current-cell-store", "data"),
    prevent_initial_call=True,
)
def delete_cell(submit_clicks, database, current_cell):
    triggered = ctx.triggered_id
    if not isinstance(triggered, dict) or not any(submit_clicks or []):
        return no_update, no_update, no_update
    cell_id = triggered.get("index")
    database = database or load_database()
    selected = next((cell for cell in database.get("cells", []) if cell.get("id") == cell_id), None)
    client = get_supabase()
    if selected and client:
        try:
            paths = [str(item.get("path")) for item in selected.get("storage_files", []) if item.get("path")]
            route = str(selected.get("storage_route") or "").strip("/")
            if route:
                paths.append(f"{route}/manifest.json")
            if paths:
                client.storage.from_(SUPABASE_BUCKET).remove(paths)
            row_id = selected.get("supabase_row_id")
            query = client.table(SUPABASE_TABLE).delete()
            (query.eq("id", row_id) if row_id else query.eq("nombre", cell_id)).execute()
        except Exception:
            return no_update, no_update, no_update
    original_count = len(database.get("cells", []))
    database["cells"] = [cell for cell in database.get("cells", []) if cell.get("id") != cell_id]
    if len(database["cells"]) == original_count:
        return no_update, no_update, no_update
    save_database(database)
    if current_cell == cell_id:
        latest = database["cells"][-1] if database["cells"] else None
        if latest:
            return database, latest.get("records", []), latest.get("id")
        return database, [], None
    return database, no_update, no_update


@app.callback(
    Output("records-store", "data", allow_duplicate=True),
    Output("upload-status", "children"),
    Output("current-cell-store", "data", allow_duplicate=True),
    Output("draft-mode-store", "data", allow_duplicate=True),
    Output("report-ready-store", "data", allow_duplicate=True),
    Output("cell-material", "value", allow_duplicate=True),
    Output("cell-observations", "value", allow_duplicate=True),
    Output("support-electrolyte", "value", allow_duplicate=True),
    Output("electrolyte-concentration", "value", allow_duplicate=True),
    Output("electrolyte-unit", "value", allow_duplicate=True),
    Output("initial-volume-ml", "value", allow_duplicate=True),
    Output("copper-stock-ppm", "value", allow_duplicate=True),
    Output("copper-added-ml", "value", allow_duplicate=True),
    Output("analysis-store", "data", allow_duplicate=True),
    Output("analysis-result", "children", allow_duplicate=True),
    Input("new-cell-btn", "n_clicks"),
    State("database-store", "data"),
    State("draft-mode-store", "data"),
    prevent_initial_call=True,
)
def new_cell(n_clicks, database, draft_mode):
    database = database or load_database()
    if draft_mode:
        return (no_update, html.Div("Ya existe un borrador activo. Guárdalo o descártalo antes de crear otra celda.", className="upload-error"), *([no_update] * 13))
    researcher = database.get("current_researcher")
    if not researcher:
        return (no_update, html.Div("Selecciona o registra un investigador antes de crear una celda.", className="upload-error"), *([no_update] * 13))
    cell_id = next_cell_id(database)
    return (
        [], html.Div([html.Strong(f"✓ Borrador {cell_id} creado."), html.Br(), "Completa los datos reales y carga los dos bloques de archivos."], className="upload-ok"),
        cell_id, True, False,
        None, "", None, None, None, None, None, None, None, "",
    )


@app.callback(
    Output("records-store", "data", allow_duplicate=True),
    Output("report-ready-store", "data", allow_duplicate=True),
    Output("blank-upload-status", "children"),
    Input("upload-blank", "contents"),
    State("upload-blank", "filename"),
    State("current-cell-store", "data"),
    State("records-store", "data"),
    State("draft-mode-store", "data"),
    prevent_initial_call=True,
)
def load_blank(contents_list, filenames, cell_id, current_records, draft_mode):
    if not cell_id or not draft_mode:
        return no_update, no_update, html.Div("Primero crea una nueva celda.", className="upload-error")
    records, errors = parse_upload(contents_list, filenames)
    techniques = sorted(record.get("technique") for record in records)
    if errors or len(records) != 2 or techniques != ["CA", "CV"]:
        detail = [html.Div(error) for error in errors]
        return no_update, no_update, html.Div([html.Strong("El blanco requiere exactamente 2 CSV: un CA y un CV."), *detail], className="upload-error")
    for record in records:
        record["stage"] = "blanco"
        record["concentration_ppm"] = 0.0
        record["rep"] = None
    merged = [record for record in (current_records or []) if record.get("stage") != "blanco"] + records
    return merged, False, html.Div("✓ Blanco cargado en el borrador: CA + CV (2/2).", className="upload-ok")


@app.callback(
    Output("records-store", "data", allow_duplicate=True),
    Output("report-ready-store", "data", allow_duplicate=True),
    Output("copper-upload-status", "children"),
    Input("upload-copper", "contents"),
    State("upload-copper", "filename"),
    State("cell-concentration", "value"),
    State("current-cell-store", "data"),
    State("records-store", "data"),
    State("draft-mode-store", "data"),
    prevent_initial_call=True,
)
def load_copper(contents_list, filenames, concentration, cell_id, current_records, draft_mode):
    if not cell_id or not draft_mode:
        return no_update, no_update, html.Div("Primero crea una nueva celda.", className="upload-error")
    if concentration is None or float(concentration) < 0:
        return no_update, no_update, html.Div("Ingresa la concentración de cobre en ppm.", className="upload-error")
    records, errors = parse_upload(contents_list, filenames)
    counts = {technique: sum(record.get("technique") == technique for record in records) for technique in ("CA", "CV", "LSV")}
    if errors or len(records) != 5 or counts != {"CA": 1, "CV": 1, "LSV": 3}:
        detail = [html.Div(error) for error in errors]
        return no_update, no_update, html.Div([html.Strong("La adición de cobre requiere exactamente 5 CSV: un CA, un CV y tres LSV."), *detail], className="upload-error")
    lsv_records = [record for record in records if record.get("technique") == "LSV"]
    for index, record in enumerate(lsv_records, start=1):
        record["rep"] = index
    for record in records:
        record["stage"] = "cobre"
        record["concentration_ppm"] = float(concentration)
    merged = [record for record in (current_records or []) if record.get("stage") != "cobre"] + records
    return merged, False, html.Div("✓ Adición de cobre cargada en el borrador: CA + CV + 3 LSV (5/5).", className="upload-ok")


@app.callback(
    Output("metric-cells", "children"),
    Output("metric-complete", "children"),
    Output("metric-concentrations", "children"),
    Output("metric-concentration-note", "children"),
    Output("metric-reports", "children"),
    Output("recent-cells", "children"),
    Output("concentration-summary", "children"),
    Output("profile-initials", "children"),
    Output("profile-name", "children"),
    Output("profile-role", "children"),
    Output("saved-researcher-filter", "options"),
    Output("saved-concentration-filter", "options"),
    Output("saved-material-filter", "options"),
    Input("database-store", "data"),
    Input("saved-researcher-filter", "value"),
    Input("saved-concentration-filter", "value"),
    Input("saved-material-filter", "value"),
    Input("saved-date-filter", "start_date"),
    Input("saved-date-filter", "end_date"),
)
def render_database_summary(database, researcher_filter, concentration_filter, material_filter, start_date, end_date):
    database = database or empty_database()
    cells = database.get("cells", [])
    complete = sum(1 for cell in cells if cell.get("complete_count") == 7)
    concentrations = sorted({float(cell["concentration_ppm"]) for cell in cells if cell.get("concentration_ppm") is not None})
    concentration_note = ", ".join(f"{value:g} ppm" for value in concentrations) if concentrations else "Sin datos"

    filtered_cells = []
    for cell in cells:
        created_date = str(cell.get("created_at", ""))[:10]
        if researcher_filter and cell.get("researcher") != researcher_filter:
            continue
        if concentration_filter is not None and cell.get("concentration_ppm") != concentration_filter:
            continue
        if material_filter and cell.get("material") != material_filter:
            continue
        if start_date and created_date < str(start_date)[:10]:
            continue
        if end_date and created_date > str(end_date)[:10]:
            continue
        filtered_cells.append(cell)

    recent_children = []
    for cell in reversed(filtered_cells):
        concentration = cell.get("concentration_ppm")
        ppm = f"{float(concentration):g} ppm" if concentration is not None else "Sin ppm"
        created = str(cell.get("created_at", ""))
        try:
            when = datetime.fromisoformat(created).strftime("%d/%m/%Y, %H:%M")
        except ValueError:
            when = "Fecha no disponible"
        is_complete = cell.get("complete_count") == 7
        recent_children.append(recent_cell(
            cell.get("id", "CELDA"), ppm, when,
            "Completada" if is_complete else f"Incompleta {cell.get('complete_count', 0)}/7",
            cell.get("researcher") or "Sin asignar",
            deletable=True,
        ))
    if not recent_children:
        recent_children = [html.Div("No hay celdas guardadas con estos filtros.", className="metric-note")]

    counts: dict[str, int] = {}
    for cell in cells:
        value = cell.get("concentration_ppm")
        label = "Sin ppm" if value is None else f"{float(value):g} ppm"
        counts[label] = counts.get(label, 0) + 1
    maximum = max(counts.values(), default=1)
    concentration_children = [
        html.Div(className="concentration-row", children=[
            html.Div(label, className="concentration-label"),
            html.Div(className="bar-track", children=html.Div(className=f"bar-fill bar-{idx % 5}", style={"width": f"{count / maximum * 100:.0f}%"})),
            html.Div(str(count), className="concentration-count"),
        ])
        for idx, (label, count) in enumerate(sorted(counts.items()))
    ] or [html.Div("Sin datos reales.", className="metric-note")]

    researcher = database.get("current_researcher")
    initials = "--"
    if researcher:
        initials = "".join(part[0] for part in researcher.split()[:2]).upper()
    researcher_options = sorted({cell.get("researcher") for cell in cells if cell.get("researcher")})
    concentration_options = [{"label": f"{float(value):g} ppm", "value": value} for value in concentrations]
    material_options = sorted({cell.get("material") for cell in cells if cell.get("material")})
    return (
        len(cells), complete, len(concentrations), concentration_note,
        database.get("reports_generated", 0), recent_children, concentration_children,
        initials, researcher or "Seleccionar investigador", "Investigador SENSA" if researcher else "SENSA",
        researcher_options, concentration_options, material_options,
    )


@app.callback(
    Output("cell-title", "children"),
    Output("completion-badge", "children"),
    Output("completion-badge", "className"),
    Output("concentration-badge", "children"),
    Output("blank-count", "children"),
    Output("copper-count", "children"),
    Output("blank-protocol", "children"),
    Output("copper-protocol", "children"),
    Output("ca-chart", "figure"),
    Output("lsv-chart", "figure"),
    Output("cv-chart", "figure"),
    Input("records-store", "data"),
    Input("current-cell-store", "data"),
)
def render_records(records, current_cell):
    records = records or []
    states = protocol_state(records)
    complete_count = sum(ok for _, _, ok in states)
    blank_states = [(label, ok) for label, stage, ok in states if stage == "blanco"]
    copper_states = [(label, ok) for label, stage, ok in states if stage == "cobre"]
    concentration = concentration_from_records(records)
    conc_text = f"{concentration:g} ppm" if concentration is not None else "Sin ppm"
    badge_text = f"Protocolo {'completo' if complete_count == 7 else 'incompleto'} {complete_count}/7"
    badge_class = "complete-badge" if complete_count == 7 else "incomplete-badge"

    title = current_cell or "SIN CELDA CARGADA"
    return (
        title,
        badge_text,
        badge_class,
        conc_text,
        f"{sum(ok for _, ok in blank_states)}/2",
        f"{sum(ok for _, ok in copper_states)}/5",
        [protocol_card(label, ok) for label, ok in blank_states],
        [protocol_card(label, ok) for label, ok in copper_states],
        make_ca_figure(records),
        make_lsv_figure(records),
        make_cv_figure(records),
    )


def build_composition(electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added, final_copper=None):
    final_volume = float(initial_volume or 0) + float(copper_added or 0)
    calculated_copper = float(copper_stock or 0) * float(copper_added or 0) / final_volume if final_volume > 0 else None
    return {
        "electrolyte": electrolyte,
        "electrolyte_concentration": electrolyte_concentration,
        "electrolyte_unit": electrolyte_unit,
        "initial_volume_ml": initial_volume,
        "copper_stock_ppm": copper_stock,
        "copper_added_ml": copper_added,
        "final_volume_ml": round(final_volume, 6),
        "final_copper_ppm": final_copper if final_copper is not None else calculated_copper,
        "measurement_units": {"current": "uA", "vset": "V", "time": "s", "volume": "mL", "copper_concentration": "ppm"},
    }


def local_analysis(records, composition=None, researcher=None):
    metrics = calculate_metrics(records)
    blank_ca = get_record(records, "CA", "blanco")
    cu_ca = get_record(records, "CA", "cobre")
    blank_cv = get_record(records, "CV", "blanco")
    cu_cv = get_record(records, "CV", "cobre")
    lsv_reps = [get_record(records, "LSV", "cobre", i) for i in (1, 2, 3)]
    completed = sum(1 for _, _, ok in protocol_state(records) if ok)
    period = assay_period(records)
    lines = [
        "## Informe electroquímico SENSA", "",
        f"**Investigador responsable:** {researcher or 'No registrado'}  ",
        f"**Fecha de realización del ensayo:** {period['date']}  ",
        f"**Hora del ensayo:** {period['time']}", "",
        "### 1. Resumen ejecutivo", "",
    ]
    lines.append(f"Se analizaron **{len(records)} archivos** y **{completed}/7** etapas del protocolo. La interpretación de CV y LSV se limita a la región electroquímica de interés **Vset = 0.2–0.6 V**; los valores fuera de ese intervalo no intervienen en los máximos ni en la repetibilidad reportada.")
    lines.extend(["", "### 2. Unidades de medida y composición de la celda", ""])
    lines.append("Las señales se analizan en **corriente filtrada (uA)**; CV y LSV contra **Vset (V)** dentro de **0.2–0.6 V**; y CA contra **tiempo (s)**.")
    if composition:
        lines.append(
            f"La celda contiene **{composition.get('initial_volume_ml', 'N/D')} mL** de **{composition.get('electrolyte', 'N/D')} "
            f"{composition.get('electrolyte_concentration', 'N/D')} {composition.get('electrolyte_unit', '')}**, más "
            f"**{composition.get('copper_added_ml', 'N/D')} mL** de solución de Cu a **{composition.get('copper_stock_ppm', 'N/D')} ppm**. "
            f"El volumen final es **{composition.get('final_volume_ml', 'N/D')} mL** y la concentración final estimada de Cu es "
            f"**{composition.get('final_copper_ppm', 'N/D')} ppm**."
        )
    else:
        lines.append("No se registraron cantidades estructuradas para esta celda; no deben inferirse valores ausentes.")
    lines.extend(["", "### 3. Cronoamperometría antes y después de añadir cobre", ""])
    if blank_ca and cu_ca:
        b = record_signal(blank_ca)
        c = record_signal(cu_ca)
        ratio = np.nanmean(np.abs(c)) / max(np.nanmean(np.abs(b)), 1e-12)
        decay = (c[0] - c[-1]) / max(abs(c[0]), 1e-12) * 100
        lines.append(f"El CA blanco inicia en **{b[0]:.3g} uA** y finaliza en **{b[-1]:.3g} uA**. Después de añadir cobre, la señal inicia en **{c[0]:.3g} uA**, finaliza en **{c[-1]:.3g} uA** y su magnitud media es **{ratio:.2f} veces** la del blanco. El cambio temporal de la señal con cobre es **{decay:.1f}%**.")
        lines.append("Una corriente mayor después de la adición evidencia una modificación de la respuesta interfacial. La disminución temporal puede involucrar carga de doble capa, transporte de masa o consumo de especie electroactiva; por sí sola no identifica selectivamente cobre.")
    else:
        lines.append("No están disponibles ambos CA; no es posible cuantificar la evolución antes/después.")

    lines.extend(["", "### 4. Evolución de las voltametrías cíclicas", ""])
    if blank_cv and cu_cv:
        bx, by = signal_in_roi(blank_cv)
        cx, cy = signal_in_roi(cu_cv)
        if len(by) and len(cy):
            bi, ci = int(np.nanargmax(by)), int(np.nanargmax(cy))
            lines.append(f"En la región **0.2–0.6 V**, el máximo anódico del blanco es **{by[bi]:.3g} uA** a **Vset = {bx[bi]:.3f} V**; con cobre es **{cy[ci]:.3g} uA** a **Vset = {cx[ci]:.3f} V**. La diferencia entre máximos es **{cy[ci]-by[bi]:.3g} uA**.")
            blank_feature = analyze_roi_feature(blank_cv)
            copper_feature = analyze_roi_feature(cu_cv)
            lines.append(describe_roi_feature("CV blanco", blank_feature))
            lines.append(describe_roi_feature("CV con cobre", copper_feature))
            lines.append(f"El CV con cobre aporta **{len(cy)} puntos** dentro de la región de interés y contiene **{cu_cv.get('cycle_count', 1)} ciclo(s)** completos en el archivo.")
            if copper_feature.get("tipo") == "pico":
                lines.append("Un pico localizado es compatible con un proceso farádico en esa zona de potencial. Solo puede asociarse a la adición de cobre si supera al blanco y se reproduce en las LSV; su presencia aislada no demuestra selectividad química.")
            elif copper_feature.get("tipo") == "meseta":
                lines.append("Una meseta indica que la corriente cambia poco en un intervalo de potencial y puede relacionarse con limitación por transporte de masa, estabilización superficial o saturación instrumental. Debe verificarse que no aparezca con forma semejante en el blanco.")
            else:
                lines.append("Al no observarse un pico o meseta definidos, no corresponde atribuir un evento electroquímico específico dentro de esta región.")
        else:
            lines.append("Los CV no contienen puntos válidos dentro de la región de interés **0.2–0.6 V**.")
    else:
        lines.append("No están disponibles ambos CV para la comparación del blanco con la adición de cobre.")

    lines.extend(["", "### 5. Tres LSV después de añadir cobre", ""])
    valid_lsv = [r for r in lsv_reps if r]
    peak_currents = []
    peak_potentials = []
    for index, record in enumerate(valid_lsv, start=1):
        x, y = signal_in_roi(record)
        if not len(y):
            lines.append(f"- LSV {index}: sin puntos válidos en **0.2–0.6 V**.")
            continue
        peak = int(np.nanargmax(y))
        peak_currents.append(float(y[peak])); peak_potentials.append(float(x[peak]))
        lines.append(f"- LSV {index}: máximo en **0.2–0.6 V** de **{y[peak]:.3g} uA** a **Vset = {x[peak]:.3f} V**; corriente a **0.6 V** de aproximadamente **{y[-1]:.3g} uA**.")
        lines.append("  " + describe_roi_feature(f"Forma LSV {index}", analyze_roi_feature(record)))
    if valid_lsv:
        lines.append(f"Las LSV se mantienen en el sentido ascendente definido por el protocolo: **{valid_lsv[0].get('vset_start', np.nan):.3f} V → {valid_lsv[0].get('vset_end', np.nan):.3f} V**.")
    if len(peak_currents) >= 2:
        cv = np.std(peak_currents) / max(abs(np.mean(peak_currents)), 1e-12) * 100
        potential_span = max(peak_potentials) - min(peak_potentials)
        lines.extend(["", "### 6. Promedio y repetibilidad de las LSV", ""])
        lines.append(f"La corriente máxima promedio es **{np.mean(peak_currents):.3g} uA**, con coeficiente de variación de **{cv:.2f}%**. Los valores Vset de máximo abarcan **{potential_span:.3f} V**. Una posición estable con amplitud decreciente puede corresponder a consumo progresivo del material disponible durante barridos consecutivos.")
        feature_types = [analyze_roi_feature(record).get("tipo") for record in valid_lsv]
        dominant = max(feature_types, key=feature_types.count) if feature_types else "sin datos suficientes"
        lines.append(f"La forma predominante entre las réplicas es **{dominant}**. Su repetición en las tres LSV aporta más evidencia que una forma observada en un único barrido; aun así, debe compararse con el blanco para atribuirla a la adición de cobre.")

    return "\n".join(lines), metrics


@app.callback(
    Output("analysis-store", "data"),
    Output("analysis-result", "children"),
    Input("analyze-btn", "n_clicks"),
    State("records-store", "data"),
    State("support-electrolyte", "value"), State("electrolyte-concentration", "value"), State("electrolyte-unit", "value"),
    State("initial-volume-ml", "value"), State("copper-stock-ppm", "value"), State("copper-added-ml", "value"),
    State("cell-concentration", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def analyze_with_ai(n_clicks, records, electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added, final_copper, database):
    records = records or []
    researcher = (database or {}).get("current_researcher")
    period = assay_period(records)
    composition = build_composition(electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added, final_copper)
    fallback_text, metrics = local_analysis(records, composition, researcher)
    gemini_key = os.getenv("GEMINI_API_KEY")
    openai_key = os.getenv("OPENAI_API_KEY")

    if (not gemini_key and not openai_key) or OpenAI is None:
        return fallback_text, html.Div([
            html.Div("Análisis local", className="analysis-kicker"),
            dcc.Markdown(fallback_text),
        ], className="analysis-box")

    try:
        protocol = protocol_state(records)
        rasgo_roi = [
            {
                "archivo": record.get("filename"),
                "técnica": record.get("technique"),
                "etapa": record.get("stage"),
                "réplica": record.get("rep"),
                **analyze_roi_feature(record),
            }
            for record in records
            if record.get("technique") in ("CV", "LSV")
        ]
        prompt = f"""
Eres el módulo de interpretación electroquímica de SENSA. Tu función es redactar en español un análisis técnico, cuantitativo, prudente y trazable de una celda electroquímica a partir de siete ensayos realizados en secuencia.

No inventes datos, mecanismos electroquímicos, unidades, materiales, concentraciones, volúmenes, fechas, horas, nombres ni condiciones experimentales.

Distingue siempre entre:
- Resultado observado o calculado: información presente en los archivos o en las métricas proporcionadas por SENSA.
- Interpretación: explicación respaldada directamente por los resultados observados.
- Hipótesis: explicación electroquímica posible que todavía requiere confirmación experimental.

Utiliza expresiones prudentes como “compatible con”, “podría relacionarse con”, “sugiere” o “requiere comparación adicional”. No afirmes que una señal corresponde al cobre si no existe evidencia suficiente frente al blanco.

El objetivo principal no es solamente enumerar métricas o clasificar el resultado. Debes explicar, en orden cronológico, el proceso electroquímico que se observa en la celda desde el blanco hasta la última LSV. Relaciona los ensayos entre sí y describe qué fenómeno domina la respuesta, qué cambia después de añadir Cu²⁺ y cómo evoluciona la interfaz electrodo-solución durante los barridos consecutivos.

Cuando interpretes el proceso electroquímico:
- Explica si la respuesta observada es compatible con corriente de fondo, polarización resistiva, carga o reorganización de la doble capa, transferencia de masa, reacción faradaica, acondicionamiento superficial, adsorción, pasivación, agotamiento local de especies o deriva instrumental.
- No presentes estas posibilidades como hechos si los datos no permiten diferenciarlas. Identifica por separado la observación confirmada, la interpretación compatible y la hipótesis que requiere validación.
- Distingue una perturbación transitoria por adición o mezcla de un cambio sostenido de la corriente.
- Indica si existe evidencia reproducible compatible con reducción, deposición, oxidación o redisolución de cobre. Si no existe, dilo expresamente y explica cuál es el proceso que sí domina la medición.
- Si la corriente cambia progresivamente entre LSV consecutivas, interpreta la tendencia temporal y no te limites a reportar su porcentaje de variación.
- No concluyas únicamente “resultado no concluyente”. Aun cuando no pueda confirmarse el cobre, explica qué proceso electroquímico sí se observa en la celda y por qué no puede atribuirse específicamente al Cu²⁺.

## Información proporcionada por SENSA

Estado y secuencia del protocolo:
{protocol}

Métricas calculadas:
{metrics}

Métricas de picos y mesetas dentro de la región de interés:
{rasgo_roi}

Composición y cantidades registradas:
{composition}

Investigador responsable:
{researcher or 'No registrado'}

Fecha real del ensayo:
{period['date']}

Hora inicial y final del ensayo:
{period['time']}

## Protocolo experimental esperado

La secuencia esperada es:
1. CA del blanco.
2. CV del blanco.
3. CA después de añadir Cu²⁺.
4. CV después de añadir Cu²⁺.
5. LSV 1.
6. LSV 2.
7. LSV 3.

Verifica el orden real utilizando la fecha, la hora, el modo electroquímico y el identificador de cada archivo.

Si falta un ensayo, está duplicado, aparece fuera de orden o su modo no coincide con el protocolo esperado, indícalo expresamente. No reconstruyas ni supongas archivos ausentes.

Interpreta los ensayos como una secuencia temporal de una misma celda, pero no asumas que pertenecen a la misma celda si el protocolo o los metadatos indican lo contrario.

## Reglas obligatorias sobre unidades

Utiliza las siguientes unidades:
- Corriente filtrada: µA.
- Potencial seteado Vset: V.
- Tiempo de CA: s.
- Volumen: mL.
- Concentración de cobre: ppm.
- Concentración del electrolito: mol/L o M, según se encuentre registrada.
- Prominencia de un pico: µA.
- Ancho de un pico: V.
- Extensión de una meseta: V.
- Pendiente de la línea base: µA/V.
- Área electroquímica corregida: µA·V.
- Desviación estándar de corriente: µA.
- Coeficiente de variación: %.
- Relación señal/ruido: adimensional.

Escribe la unidad junto a cada resultado cuantitativo, incluso cuando varios valores consecutivos correspondan a la misma magnitud.

Ninguna corriente, potencial, tiempo, volumen, concentración, área, pendiente, prominencia, ancho o variación puede aparecer sin su unidad correspondiente.

No conviertas unidades si la conversión no puede realizarse de manera inequívoca.

## Reglas sobre composición y concentración

Identifica por separado:
- Volumen inicial del electrolito en mL.
- Concentración inicial del electrolito en mol/L o M.
- Volumen de solución de Cu²⁺ añadido en mL.
- Concentración de la solución de Cu²⁺ añadida en ppm.
- Volumen final de la celda en mL.
- Concentración final estimada de Cu²⁺ en ppm.

No confundas la concentración de la solución añadida con la concentración final de la celda.

Calcula la concentración final únicamente cuando estén disponibles el volumen inicial, el volumen añadido y la concentración de la solución añadida:

C_final = (C_añadida × V_añadido) / (V_inicial + V_añadido)

Si antes de la adición se retiró una cantidad de solución, utiliza el volumen final real registrado. Si no existe información suficiente, escribe “concentración final no calculable con los datos disponibles” y no realices el cálculo.

No supongas que el valor indicado en el nombre del archivo representa automáticamente la concentración final de la celda.

## Región electroquímica obligatoria

Para todos los CV y LSV, analiza exclusivamente los puntos cuyo Vset se encuentre entre +0,2 V y +0,6 V, incluyendo ambos límites.

No utilices puntos exteriores a esta región para:
- Identificar máximos o mínimos.
- Detectar picos o mesetas.
- Calcular amplitudes.
- Calcular prominencias.
- Calcular anchos.
- Calcular áreas electroquímicas.
- Calcular corrientes medias.
- Evaluar repetibilidad.
- Comparar el blanco con la condición posterior a la adición.
- Formular la interpretación electroquímica.

Si un CV o una LSV no cubre completamente el intervalo de +0,2 V a +0,6 V, indícalo expresamente y limita el análisis al tramo realmente disponible. No extrapoles datos.

La CA se analiza respecto al tiempo completo en s y no debe recortarse utilizando Vset.

## Clasificación obligatoria de los rasgos

Dentro de +0,2 V a +0,6 V, clasifica cada CV y cada LSV como una de estas opciones:
- Pico.
- Meseta.
- Respuesta sin rasgo definido.
- Datos insuficientes para clasificar.

Utiliza exclusivamente las métricas presentes en rasgo_roi.

Para un pico, reporta:
- Potencial central en V.
- Corriente del pico en µA.
- Prominencia en µA.
- Ancho en V.
- Área corregida en µA·V, si está disponible.
- Relación señal/ruido, si está disponible.

Para una meseta, reporta:
- Potencial inicial en V.
- Potencial final en V.
- Extensión en V.
- Corriente media en µA.
- Desviación estándar en µA, si está disponible.
- Pendiente de la meseta en µA/V, si está disponible.

Para una respuesta sin rasgo definido, explica cuantitativamente por qué no cumple los criterios de pico o meseta, utilizando solamente las métricas proporcionadas.

No transformes una elevación ancha en un “pico” si las métricas la clasifican como meseta. No clasifiques una pendiente creciente como meseta si no existe una zona de estabilización.

## Tratamiento de la corriente filtrada

Prioriza la corriente filtrada indicada por SENSA.

Antes de utilizarla, verifica que:
- Contenga valores numéricos válidos.
- Cambie a lo largo del ensayo cuando la corriente original también cambia.
- No sea una columna artificialmente constante.
- No presente datos faltantes o saltos incompatibles con la señal original.

Si la corriente filtrada final aparece constante o inválida por un posible error de almacenamiento, indícalo en el apartado 6 y utiliza la corriente filtrada del ESP32 solamente si está disponible y correctamente identificada.

No mezcles corriente sin filtrar y corriente filtrada dentro de una misma comparación sin advertirlo.

## Análisis de las CA

Compara la CA del blanco y la CA posterior a la adición de Cu²⁺ utilizando:
- Potencial aplicado en V.
- Duración total en s.
- Corriente inicial en µA.
- Corriente final en µA.
- Corriente estabilizada en µA.
- Cambio absoluto de corriente en µA.
- Cambio relativo en %.
- Tiempo de estabilización en s.
- Desviación estándar en la zona estable en µA.
- Presencia de transitorios, deriva o ruido.

Distingue entre una variación inicial transitoria y un cambio sostenido de la corriente.

Una corriente inicial elevada después de la adición no debe interpretarse automáticamente como detección de cobre si posteriormente disminuye o no se mantiene.

## Análisis de los CV

Para el CV del blanco y el CV posterior a la adición:
- Analiza cada ciclo por separado.
- Distingue la rama de ida y la rama de regreso.
- No combines ramas con direcciones opuestas para calcular un único pico.
- Compara corriente, forma, pico, meseta e histéresis dentro de +0,2 V a +0,6 V.
- Determina si un rasgo ya existía en el blanco.
- Determina si el rasgo aparece, aumenta, disminuye o se desplaza después de la adición.
- Informa el desplazamiento de potencial en V y la diferencia de corriente en µA cuando estas métricas estén disponibles.

No atribuyas un rasgo al Cu²⁺ si aparece con forma y posición semejantes en el CV del blanco.

## Análisis y promedio de las tres LSV

Analiza individualmente LSV 1, LSV 2 y LSV 3 dentro de +0,2 V a +0,6 V.

Para cada LSV reporta, según las métricas disponibles:
- Clasificación del rasgo.
- Potencial central o intervalo en V.
- Corriente característica en µA.
- Prominencia en µA.
- Ancho en V.
- Área corregida en µA·V.
- Pendiente de línea base en µA/V.
- Ruido en µA.
- Relación señal/ruido.
- Corriente máxima y mínima en µA.

Después evalúa su repetibilidad utilizando:
- Promedio punto a punto de las tres LSV en µA.
- Desviación estándar en µA.
- Coeficiente de variación en %.
- Correlación entre curvas, si está disponible.
- Desplazamiento del rasgo entre repeticiones en V.
- Variación de amplitud entre repeticiones en µA.
- Tendencia temporal de aumento o disminución de la señal.

Si las curvas no poseen los mismos valores de Vset, utiliza únicamente el promedio interpolado proporcionado por SENSA. No inventes ni describas un procedimiento de interpolación que no esté registrado en las métricas.

## Comparación con el blanco

Antes de relacionar un pico o una meseta con Cu²⁺, verifica:
- Si el rasgo está presente en el blanco.
- Si aparece en un potencial semejante en V.
- Si su corriente cambia después de añadir Cu²⁺.
- Si la diferencia supera el ruido y la variabilidad experimental.
- Si se reproduce en LSV 1, LSV 2 y LSV 3.
- Si existe una relación coherente con la concentración.

La existencia de un pico o una meseta reproducible no demuestra por sí sola que corresponda al cobre.

Clasifica la evidencia final como una de estas opciones:
- Señal compatible con respuesta de Cu²⁺.
- Señal electroquímica reproducible, pero no específica de Cu²⁺.
- Respuesta dominada por el blanco.
- Ensayo afectado por deriva, ruido, fuga o adquisición.
- Resultado no concluyente.

## Datos ausentes

Cuando un dato obligatorio no esté disponible, escribe “No registrado” o “No calculable con los datos disponibles”, según corresponda.

No reemplaces información ausente con valores aproximados, típicos, esperados o tomados de otros ensayos.

## Formato obligatorio de la respuesta

Antes del apartado 1, muestra exactamente:

Investigador responsable: {researcher or 'No registrado'}
Fecha del ensayo: {period['date']}
Hora inicial del ensayo: {period['start_time']}
Hora final del ensayo: {period['end_time']}

Las horas inicial y final deben obtenerse de la secuencia real de los siete ensayos. No utilices la hora de generación del informe.

Después entrega exactamente seis apartados numerados. No agregues apartados 7, 8 ni 9. No incluyas anexos, recomendaciones adicionales ni conclusiones fuera del apartado 6.

### 1. Resumen ejecutivo breve

Resume:
- Cumplimiento o incumplimiento del protocolo.
- Comportamiento general de la celda.
- Cambio principal después de añadir Cu²⁺.
- Rasgo predominante dentro de +0,2 V a +0,6 V.
- Nivel de evidencia respecto al cobre.

No excedas dos párrafos.

### 2. Unidades de medida y composición completa de la celda

Presenta:
- Todas las unidades utilizadas.
- Componentes y materiales registrados.
- Volumen inicial en mL.
- Volumen añadido en mL.
- Volumen final en mL.
- Concentración del electrolito en mol/L o M.
- Concentración de la solución de Cu²⁺ añadida en ppm.
- Concentración final estimada de Cu²⁺ en ppm.
- Electrodos registrados y función de cada uno, únicamente si esta información está disponible.
- Secuencia cronológica de los siete ensayos con su hora real.

### 3. Análisis del blanco

Analiza:
- CA del blanco respecto al tiempo en s.
- Estabilidad, transitorio, deriva y ruido en µA.
- CV del blanco dentro de +0,2 V a +0,6 V.
- Cada ciclo y cada rama del CV por separado.
- Clasificación del rasgo del CV blanco como pico, meseta, respuesta sin rasgo definido o datos insuficientes.
- Métricas cuantitativas del rasgo.

Este apartado establece la referencia electroquímica para las comparaciones posteriores.

### 4. Efecto después de añadir Cu²⁺ y proceso electroquímico observado

Analiza:
- CA después de la adición respecto al tiempo en s.
- Diferencia frente a la CA del blanco en µA y %.
- Si el cambio es transitorio o sostenido.
- CV posterior a la adición dentro de +0,2 V a +0,6 V.
- Cada ciclo y cada rama del CV por separado.
- Clasificación del rasgo.
- Diferencia de potencial en V y corriente en µA frente al blanco.
- Compatibilidad o incompatibilidad con una respuesta asociada al Cu²⁺.

Después de las métricas, incluye una explicación narrativa que conecte la CA del blanco, la CV del blanco, la CA con Cu²⁺, la CV con Cu²⁺ y la evolución posterior de las LSV. Explica el estado inicial de la celda, la respuesta inmediata a la adición, si el cambio fue transitorio o persistente y qué proceso electroquímico es compatible con los datos.

No confundas un cambio producido por mezcla, carga capacitiva o acondicionamiento con una detección confirmada.

### 5. Repetibilidad de LSV 1, LSV 2 y LSV 3

Incluye:
- Resultado individual de cada LSV.
- Clasificación de cada rasgo.
- Potencial, corriente, prominencia, ancho, extensión o corriente media, según corresponda.
- Promedio de las tres LSV.
- Variabilidad y repetibilidad.
- Desplazamiento del rasgo entre repeticiones.
- Evolución temporal de la señal.
- Comparación del promedio LSV con el blanco.
- Evaluación prudente de la relación entre la señal y el Cu²⁺.

### 6. Señales que requieren revisión

Enumera únicamente problemas sustentados por los datos, por ejemplo:
- Archivos faltantes, duplicados o fuera de orden.
- Columnas filtradas constantes o inválidas.
- Datos faltantes o pérdida de adquisición.
- Ruido elevado.
- Deriva.
- Histéresis excesiva.
- Baja repetibilidad.
- Desplazamiento importante del rasgo.
- Fuga de solución.
- Región de +0,2 V a +0,6 V incompleta.
- Concentración final no calculable.
- Diferencias entre los metadatos y los comentarios.
- Evidencia insuficiente para atribuir el rasgo al Cu²⁺.

Finaliza este apartado con una sola conclusión trazable, seleccionando una de las cinco categorías de evidencia establecidas.

Antes de indicar esa categoría, responde explícitamente en un párrafo continuo: qué proceso electroquímico se observó en la celda, qué cambió después de añadir Cu²⁺, cómo evolucionó la respuesta del electrodo durante las tres LSV, si se observa evidencia de reducción, deposición, oxidación o redisolución de cobre y qué fenómeno domina finalmente la medición.

No escribas contenido después del apartado 6.
"""
        # Prompt dinámico: interpreta la historia real de la celda según los
        # archivos disponibles, sin forzar una secuencia fija de siete ensayos.
        available_files = []
        full_range_metrics = []
        for record in records:
            filename = str(record.get("filename", ""))
            match = re.search(r"(20\d{6})[_-](\d{6})", filename)
            assay_time = match.group(2) if match else None
            if assay_time:
                assay_time = f"{assay_time[0:2]}:{assay_time[2:4]}:{assay_time[4:6]}"
            signal = record_signal(record)
            x_values = np.asarray(record.get("x", []), dtype=float)
            available_files.append({
                "archivo": filename,
                "hora": assay_time or "No identificada",
                "técnica": record.get("technique"),
                "condición_inferida": record.get("stage"),
                "réplica": record.get("rep"),
                "puntos": int(len(signal)),
                "columna_x": record.get("x_column"),
                "columna_corriente": record.get("y_column"),
                "ventana_filtro": record.get("filter_window"),
                "Vset_inicial_V": record.get("vset_start"),
                "Vset_final_V": record.get("vset_end"),
                "Vset_mínimo_V": record.get("vset_min"),
                "Vset_máximo_V": record.get("vset_max"),
            })
            if len(signal):
                full_range_metrics.append({
                    "archivo": filename,
                    "técnica": record.get("technique"),
                    "condición_inferida": record.get("stage"),
                    "puntos": int(len(signal)),
                    "x_mínimo": float(np.nanmin(x_values)) if len(x_values) else None,
                    "x_máximo": float(np.nanmax(x_values)) if len(x_values) else None,
                    "corriente_mínima_uA": float(np.nanmin(signal)),
                    "corriente_máxima_uA": float(np.nanmax(signal)),
                    "corriente_media_uA": float(np.nanmean(signal)),
                    "corriente_inicial_uA": float(signal[0]),
                    "corriente_final_uA": float(signal[-1]),
                })

        prompt = f"""
Eres el módulo de interpretación electroquímica de SENSA. Redacta en español un informe técnico que reconstruya la evolución real de una celda electroquímica a partir de los archivos que están disponibles. El estilo debe ser narrativo, cuantitativo, prudente y semejante al de un electroquímico que compara la historia completa de la celda.

No fuerces un protocolo fijo ni inventes ensayos ausentes. Analiza todos y únicamente los archivos listados. Ordénalos cronológicamente por la hora del nombre del archivo. Si existe un solo ensayo de una técnica, analízalo individualmente; si existen varios, compáralos en secuencia. Genera solamente las secciones que correspondan a las técnicas y condiciones realmente disponibles.

## Datos proporcionados por SENSA

Archivos disponibles en la celda:
{available_files}

Métricas de cada archivo en todo su intervalo adquirido:
{full_range_metrics}

Métricas calculadas por SENSA en la región de interés de +0,2 V a +0,6 V:
{metrics}

Clasificación de picos, mesetas y otros rasgos en +0,2 V a +0,6 V:
{rasgo_roi}

Composición y cantidades registradas:
{composition}

Investigador responsable: {researcher or 'No registrado'}
Fecha real del ensayo: {period['date']}
Hora inicial: {period['start_time']}
Hora final: {period['end_time']}

## Objetivo principal

Explica qué proceso electroquímico se observa en la celda y cómo cambia su estado desde el primer archivo hasta el último. No te limites a enumerar mínimos, máximos o promedios. Conecta los resultados entre sí y explica el significado electroquímico de los cambios.

Distingue siempre:
- Observación confirmada: cambio directamente visible o calculado.
- Interpretación compatible: explicación coherente con los datos, pero no demostrada.
- Hipótesis: mecanismo que requiere un ensayo adicional para confirmarse.

Utiliza expresiones como “compatible con”, “podría relacionarse con”, “sugiere” y “no permite confirmar”. No atribuyas automáticamente una señal al cobre.

## Reglas de interpretación

1. Reconstruye la secuencia cronológica real e identifica CA, CV y LSV, condición inferida, hora y número de repetición cuando esté disponible.
2. En CA, analiza transitorio, decaimiento, estabilidad, deriva y corriente final. Compara todas las CA disponibles y decide si el estado del blanco o de la celda cambió con el tiempo.
3. En CV, compara forma, corriente de fondo, ramas, histéresis y crecimiento de corriente. Si una respuesta ya aparece en el blanco, no la asignes exclusivamente al cobre.
4. En LSV, analiza forma, zona electroactiva, máximo o meseta, potencial, amplitud y evolución entre repeticiones. Una tendencia ordenada entre barridos debe interpretarse como evolución sistemática, no como simple ruido.
5. Si existen varias LSV, evalúa su superposición, repetibilidad, señal promedio, dispersión, desplazamiento del máximo y tendencia de amplitud.
6. Considera como explicaciones posibles la polarización, carga o reorganización de la doble capa, memoria electroquímica, modificación superficial, adsorción, pasivación, transferencia de masa, agotamiento o redistribución de especie electroactiva y deriva instrumental. Indica cuáles son compatibles y cuáles no pueden distinguirse con los datos.
7. Solo describe una señal como compatible con cobre cuando aparece después de la adición, supera razonablemente el fondo, conserva forma o potencial entre repeticiones y es coherente con reducción, deposición, oxidación o redisolución. Aun así, aclara si falta un blanco LSV equivalente.
8. Si no se confirma cobre, explica qué proceso sí domina la respuesta. Nunca termines únicamente con “resultado no concluyente”.
9. Usa las métricas del intervalo completo para describir la evolución global. Usa la región +0,2 V a +0,6 V para clasificar y comparar el rasgo analítico. Si el archivo no cubre esa región, indícalo.
10. No inventes materiales, electrodos, mecanismos, unidades, volúmenes ni concentraciones. Cuando falte un dato, escribe “No registrado” o “No calculable con los datos disponibles”.

## Unidades y redacción

- Corriente: µA.
- Vset: V.
- Tiempo de CA: s.
- Volumen: mL.
- Concentración: ppm o la unidad registrada.
- Redondea corrientes a 1 o 2 decimales, potenciales a 3 decimales y porcentajes a 1 decimal.
- Integra las cifras dentro de la interpretación y evita listas extensas de números sin explicación.

## Estructura dinámica obligatoria

Inicia con:

Investigador responsable: {researcher or 'No registrado'}
Fecha del ensayo: {period['date']}
Hora inicial del ensayo: {period['start_time']}
Hora final del ensayo: {period['end_time']}

Después utiliza, únicamente cuando sean aplicables, estos apartados:

### 1. Resumen ejecutivo
Resume la evolución de la celda, el proceso dominante, la calidad de la evidencia y la posible participación del cobre.

### 2. Secuencia experimental y composición
Enumera cronológicamente los archivos realmente disponibles y describe solamente la composición registrada. Calcula la concentración final únicamente si existen todos los datos necesarios.

### 3. Evolución de las cronoamperometrías
Inclúyelo solo si existe al menos una CA. Compara todas las CA disponibles y explica qué revela su evolución sobre el estado de la interfaz.

### 4. Evolución de las voltametrías cíclicas
Inclúyelo solo si existe al menos una CV. Compara todos los CV disponibles y distingue la respuesta que ya pertenece al blanco de cualquier cambio posterior.

### 5. LSV, promedio y repetibilidad
Inclúyelo solo si existe al menos una LSV. Si hay varias, analiza cada una y luego su tendencia conjunta. Identifica la región candidata para cuantificación y explica si falta un blanco LSV equivalente.

### 6. Interpretación del proceso electroquímico
Narra en orden qué ocurrió en la celda. Explica el proceso dominante, la evolución de la superficie o interfaz, la posible memoria electroquímica y si existe evidencia compatible con reducción, deposición, oxidación o redisolución de cobre.

### 7. Limitaciones y señales que requieren revisión
Incluye únicamente limitaciones demostradas por los datos: blanco inestable, filtro inadecuado, deriva, ruido, falta de réplica, región incompleta, ausencia de blanco equivalente o concentración no calculable.

### 8. Conclusión general
Responde expresamente: qué proceso electroquímico se observó, qué cambió durante la secuencia, si la señal es reproducible, si puede atribuirse al cobre y cuál es el siguiente control experimental indispensable.

No agregues secciones correspondientes a técnicas inexistentes. No menciones archivos que no estén en la lista. No escribas recomendaciones genéricas: cada conclusión debe estar vinculada a una observación o métrica proporcionada.
"""
        if gemini_key:
            # Gemini ofrece una interfaz compatible con el SDK de OpenAI.
            client = OpenAI(
                api_key=gemini_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            )
            response = client.chat.completions.create(
                model=os.getenv("GEMINI_MODEL", "gemini-3.6-flash"),
                messages=[{"role": "user", "content": prompt}],
            )
            text = response.choices[0].message.content
            provider_label = "Análisis con Gemini"
        else:
            client = OpenAI(api_key=openai_key)
            response = client.responses.create(
                model=os.getenv("OPENAI_MODEL", "gpt-5.6"),
                input=prompt,
            )
            text = response.output_text
            provider_label = "Análisis con OpenAI"
        return text, html.Div([
            html.Div(provider_label, className="analysis-kicker"),
            dcc.Markdown(text),
        ], className="analysis-box")
    except Exception as exc:
        error_text = fallback_text + f"\n\n> La llamada de IA no pudo completarse: {exc}"
        return error_text, html.Div([html.Div("Análisis local (IA no disponible)", className="analysis-kicker"), dcc.Markdown(error_text)], className="analysis-box")


@app.callback(
    Output("database-store", "data", allow_duplicate=True),
    Output("draft-mode-store", "data", allow_duplicate=True),
    Output("save-cell-status", "children"),
    Input("save-cell-btn", "n_clicks"),
    State("draft-mode-store", "data"),
    State("report-ready-store", "data"),
    State("records-store", "data"),
    State("current-cell-store", "data"),
    State("database-store", "data"),
    State("cell-material", "value"),
    State("cell-observations", "value"),
    State("cell-concentration", "value"),
    State("analysis-store", "data"),
    State("support-electrolyte", "value"), State("electrolyte-concentration", "value"), State("electrolyte-unit", "value"),
    State("initial-volume-ml", "value"), State("copper-stock-ppm", "value"), State("copper-added-ml", "value"),
    State("pdf-content-store", "data"),
    prevent_initial_call=True,
)
def save_current_cell(n_clicks, draft_mode, report_ready, records, cell_id, database, material, observations, concentration, analysis, electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added, pdf_b64):
    if not draft_mode:
        return no_update, no_update, html.Div("Crea una nueva celda antes de guardar.", className="upload-error")
    complete_count = sum(ok for _, _, ok in protocol_state(records or []))
    missing = []
    if complete_count != 7:
        missing.append(f"protocolo completo 7/7 (actual: {complete_count}/7)")
    if not report_ready:
        missing.append("informe PDF generado")
    if not material:
        missing.append("material del electrodo")
    if concentration is None:
        missing.append("concentración")
    if any(value is None for value in (electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added)):
        missing.append("cantidades completas de la celda")
    if missing:
        return no_update, no_update, html.Div("Falta: " + ", ".join(missing) + ".", className="upload-error")
    database = database or load_database()
    researcher = database.get("current_researcher")
    if not researcher:
        return no_update, no_update, html.Div("Selecciona el investigador responsable.", className="upload-error")
    composition = build_composition(electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume, copper_stock, copper_added, concentration)
    cell = {
        "id": cell_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "concentration_ppm": float(concentration),
        "complete_count": 7,
        "status": "Guardada",
        "researcher": researcher,
        "material": material,
        "composition": composition,
        "observations": " ".join(str(observations or "").split()),
        "records": records or [],
        "analysis": analysis or local_analysis(records or [], composition, researcher)[0],
        "report_generated": True,
        "report_filename": f"SENSA_Informe_{cell_id}.pdf",
    }
    try:
        if any(item.get("id") == cell_id for item in database.get("cells", [])):
            raise ValueError("La celda ya fue guardada.")
        stored_cell = save_cell_to_supabase(cell, pdf_b64)
        database = save_completed_cell(database, stored_cell)
    except Exception as exc:
        return no_update, no_update, html.Div(str(exc), className="upload-error")
    database["reports_generated"] = int(database.get("reports_generated", 0)) + 1
    save_database(database)
    return database, False, html.Div(f"✓ {cell_id} guardada definitivamente y añadida a Celdas guardadas.", className="upload-ok")


@app.callback(
    Output("records-store", "data", allow_duplicate=True),
    Output("current-cell-store", "data", allow_duplicate=True),
    Output("draft-mode-store", "data", allow_duplicate=True),
    Output("report-ready-store", "data", allow_duplicate=True),
    Output("save-cell-status", "children", allow_duplicate=True),
    Input("discard-draft-btn", "n_clicks"),
    State("draft-mode-store", "data"),
    prevent_initial_call=True,
)
def discard_draft(n_clicks, draft_mode):
    if not draft_mode:
        return no_update, no_update, no_update, no_update, html.Div("No hay un borrador activo.", className="upload-error")
    return [], None, False, False, html.Div("Borrador descartado. No se guardó ningún dato.", className="upload-ok")


@app.callback(
    Output("pdf-download", "data"),
    Output("report-ready-store", "data", allow_duplicate=True),
    Output("pdf-content-store", "data"),
    Input("pdf-btn", "n_clicks"),
    State("records-store", "data"),
    State("analysis-store", "data"),
    State("ca-chart", "figure"),
    State("lsv-chart", "figure"),
    State("cv-chart", "figure"),
    State("current-cell-store", "data"),
    State("support-electrolyte", "value"),
    State("electrolyte-concentration", "value"),
    State("electrolyte-unit", "value"),
    State("initial-volume-ml", "value"),
    State("copper-stock-ppm", "value"),
    State("copper-added-ml", "value"),
    State("cell-concentration", "value"),
    State("database-store", "data"),
    prevent_initial_call=True,
)
def make_pdf(n_clicks, records, analysis_text, ca_fig, lsv_fig, cv_fig, current_cell,
             electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume,
             copper_stock, copper_added, final_copper, database):
    if not n_clicks:
        return no_update, no_update, no_update
    if not records:
        return no_update, no_update, no_update

    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
    from reportlab.lib.units import cm
    from reportlab.platypus import Image, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle

    buffer = io.BytesIO()
    doc = SimpleDocTemplate(buffer, pagesize=A4, rightMargin=1.5*cm, leftMargin=1.5*cm, topMargin=1.2*cm, bottomMargin=1.2*cm)
    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle(name="SensaTitle", parent=styles["Title"], fontSize=20, textColor=colors.HexColor(BLUE), alignment=TA_CENTER, spaceAfter=10))
    styles.add(ParagraphStyle(name="Section", parent=styles["Heading2"], fontSize=12, textColor=colors.HexColor(BLUE), spaceBefore=10, spaceAfter=6))
    styles.add(ParagraphStyle(name="BodySmall", parent=styles["BodyText"], fontSize=9, leading=12, textColor=colors.HexColor("#334155")))

    story = []
    composition = build_composition(
        electrolyte, electrolyte_concentration, electrolyte_unit, initial_volume,
        copper_stock, copper_added, final_copper,
    )
    saved_cell = next((cell for cell in (database or {}).get("cells", []) if cell.get("id") == current_cell), None)
    researcher = (saved_cell or {}).get("researcher") or (database or {}).get("current_researcher") or "No registrado"
    period = assay_period(records)
    def display_number(value, decimals=4):
        try:
            return f"{float(value):.{decimals}g}"
        except (TypeError, ValueError):
            return "N/D"

    def pdf_markup(value):
        """Texto seguro para Paragraph y notación Cu2+ sin glifos cuadrados."""
        from xml.sax.saxutils import escape

        text = escape(str(value))
        # Helvetica no siempre contiene el glifo Unicode de superíndice +.
        # ReportLab sí puede representar la carga con la etiqueta <super>.
        text = text.replace("Cu²⁺", "Cu<super>2+</super>")
        text = text.replace("Cu²+", "Cu<super>2+</super>")
        return text

    def pdf_paragraph(value, style=None):
        return Paragraph(pdf_markup(value), style or styles["BodySmall"])

    logo_path = ASSETS / "sensa_logo_header.png"
    if logo_path.exists():
        story.append(Image(str(logo_path), width=5.2*cm, height=3.15*cm))
    story.append(Paragraph("Informe electroquímico SENSA Cells", styles["SensaTitle"]))
    conc = composition.get("final_copper_ppm")

    header_data = [
        [
            Paragraph("<b>Celda</b>", styles["BodySmall"]),
            Paragraph("<b>Cu<super>2+</super> final</b>", styles["BodySmall"]),
            Paragraph("<b>Investigador</b>", styles["BodySmall"]),
            Paragraph("<b>Fecha y hora del ensayo</b>", styles["BodySmall"]),
        ],
        [
            pdf_paragraph(current_cell or "Sin identificar"),
            Paragraph(
                f"{display_number(conc) if conc is not None else 'No detectada'} ppm",
                styles["BodySmall"],
            ),
            pdf_paragraph(researcher),
            pdf_paragraph(f"{period['date']} | {period['time']}"),
        ],
    ]
    header_table = Table(header_data, colWidths=[3.0*cm, 3.0*cm, 4.0*cm, 5.0*cm])
    header_table.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#EAF2FB")),
        ("TEXTCOLOR", (0,0), (-1,0), colors.HexColor(BLUE)),
        ("BOX", (0,0), (-1,-1), 0.6, colors.HexColor(BORDER)),
        ("INNERGRID", (0,0), (-1,-1), 0.35, colors.HexColor(BORDER)),
        ("VALIGN", (0,0), (-1,-1), "MIDDLE"),
        ("LEFTPADDING", (0,0), (-1,-1), 7),
        ("RIGHTPADDING", (0,0), (-1,-1), 7),
        ("TOPPADDING", (0,0), (-1,-1), 6),
        ("BOTTOMPADDING", (0,0), (-1,-1), 6),
    ]))
    story.append(header_table)
    story.append(Spacer(1, 8))
    story.append(Paragraph("Unidades y composición de la celda", styles["Section"]))
    composition_data = [
        ["Magnitud / componente", "Cantidad y unidad"],
        ["Señales analizadas", "Corriente filtrada: uA | Vset: V | Tiempo CA: s"],
        ["Electrolito soporte", f"{composition.get('electrolyte') or 'N/D'} — {display_number(composition.get('electrolyte_concentration'))} {composition.get('electrolyte_unit') or ''}"],
        ["Volumen inicial", f"{display_number(composition.get('initial_volume_ml'))} mL"],
        ["Solución de cobre añadida", f"{display_number(composition.get('copper_added_ml'))} mL a {display_number(composition.get('copper_stock_ppm'))} ppm"],
        ["Volumen final", f"{display_number(composition.get('final_volume_ml'))} mL"],
        ["Cobre final estimado", f"{display_number(composition.get('final_copper_ppm'))} ppm"],
    ]
    composition_table = Table(composition_data, colWidths=[5.2*cm, 9.8*cm])
    composition_table.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,0), colors.HexColor(BLUE)),
        ("TEXTCOLOR", (0,0), (-1,0), colors.white),
        ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"),
        ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor(BORDER)),
        ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
        ("FONTSIZE", (0,0), (-1,-1), 8.5),
        ("TOPPADDING", (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
    ]))
    story.append(composition_table)
    story.append(Spacer(1, 8))
    objective_scope_data = [
        [
            Paragraph("<b>Objetivo</b>", styles["BodySmall"]),
            pdf_paragraph(
                "Documentar la evolución de la celda desde el blanco hasta la respuesta "
                "después de añadir cobre, comparando CA, CV y tres LSV consecutivas con métricas trazables."
            ),
        ],
        [
            Paragraph("<b>Alcance</b>", styles["BodySmall"]),
            pdf_paragraph(
                "El informe diferencia resultados observados de interpretaciones electroquímicas. "
                "Para CV y LSV, las métricas analíticas se calculan en Vset = 0.2–0.6 V; "
                "la CA se analiza contra tiempo."
            ),
        ],
    ]
    objective_scope_table = Table(objective_scope_data, colWidths=[3.2*cm, 11.8*cm])
    objective_scope_table.setStyle(TableStyle([
        ("BOX", (0,0), (-1,-1), 0.6, colors.HexColor(BORDER)),
        ("INNERGRID", (0,0), (-1,-1), 0.35, colors.HexColor(BORDER)),
        ("BACKGROUND", (0,0), (0,-1), colors.HexColor("#F1F5F9")),
        ("VALIGN", (0,0), (-1,-1), "TOP"),
        ("LEFTPADDING", (0,0), (-1,-1), 7),
        ("RIGHTPADDING", (0,0), (-1,-1), 7),
        ("TOPPADDING", (0,0), (-1,-1), 6),
        ("BOTTOMPADDING", (0,0), (-1,-1), 6),
    ]))
    story.append(objective_scope_table)

    states = protocol_state(records or [])
    data = [["Archivo", "Técnica", "Condición", "Puntos", "Filtro"]]
    for record in records or []:
        window = record.get("filter_window") or smooth_signal(np.asarray(record.get("y", []), float), record.get("technique"))[1]
        data.append([record.get("filename", ""), record.get("technique", ""), "Blanco" if record.get("stage") == "blanco" else "Con Cu", str(len(record.get("y", []))), f"Ventana {window}"])
    table = Table(data, colWidths=[7.1*cm, 1.7*cm, 2.5*cm, 1.4*cm, 2.8*cm])
    table.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,0), colors.HexColor(BLUE)),
        ("TEXTCOLOR", (0,0), (-1,0), colors.white),
        ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"),
        ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor(BORDER)),
        ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, colors.HexColor("#F8FAFC")]),
        ("FONTSIZE", (0,0), (-1,-1), 8.5),
        ("BOTTOMPADDING", (0,0), (-1,-1), 5),
        ("TOPPADDING", (0,0), (-1,-1), 5),
    ]))
    story.append(Paragraph("1. Secuencia experimental y archivos", styles["Section"]))
    story.append(table)
    story.append(Spacer(1, 10))
    story.append(Paragraph("Control del protocolo", styles["Section"]))
    protocol_data = [["Etapa", "Estado"]] + [
        [pdf_paragraph(label), pdf_paragraph("Completa" if ok else "Faltante")]
        for label, _, ok in states
    ]
    protocol_table = Table(protocol_data, colWidths=[11*cm, 4*cm])
    protocol_table.setStyle(TableStyle([("BACKGROUND", (0,0), (-1,0), colors.HexColor(BLUE)), ("TEXTCOLOR", (0,0), (-1,0), colors.white), ("GRID", (0,0), (-1,-1), .4, colors.HexColor(BORDER)), ("FONTSIZE", (0,0), (-1,-1), 8.5)]))
    story.append(protocol_table)

    # Las gráficas del PDF se generan con Matplotlib.
