#!/usr/bin/env python3
"""
log_to_xlsx.py - turn a myact_ctl CSV log into an Excel workbook for performance monitoring.

Usage:
    python3 log_to_xlsx.py myact_log_20260101_120000.csv            # -> same name, .xlsx
    python3 log_to_xlsx.py run.csv report.xlsx

Needs:  pip install openpyxl

Workbook layout
    Summary  : one row per motor (formulas that read the axis sheets), plus notes
    Axis0..N : the raw samples of each motor, 3 extra error columns (formulas) and 3 charts
               (position cmd vs actual, velocity cmd vs actual, torque cmd vs actual)
"""
import csv
import os
import sys
from collections import OrderedDict

try:
    from openpyxl import Workbook
    from openpyxl.chart import Reference, ScatterChart, Series
    from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
    from openpyxl.utils import get_column_letter
except ImportError:
    sys.exit("openpyxl is missing. Install it with:  pip install openpyxl")

FONT = "Arial"

# column order in the CSV written by myact_ctl (A..S in each axis sheet)
COLS = [
    "time_s", "axis", "phase", "statusword", "error_code", "mode_display",
    "act_pos", "cmd_pos", "act_vel", "cmd_vel", "act_torque_pm", "cmd_torque_pm",
    "pos_deg", "cmd_pos_deg", "vel_rpm", "cmd_vel_rpm", "torque_pct", "cmd_torque_pct",
    "comm_bad",
]
TEXT_COLS = {"phase", "statusword", "error_code"}
INT_COLS = {"axis", "mode_display", "act_pos", "cmd_pos", "act_vel", "cmd_vel",
            "act_torque_pm", "cmd_torque_pm", "comm_bad"}
COL = {name: get_column_letter(i + 1) for i, name in enumerate(COLS)}   # name -> letter
# extra formula columns after the 19 logged ones
EXTRA = ["pos_err_deg", "vel_err_rpm", "torque_err_pct"]
for i, name in enumerate(EXTRA):
    COL[name] = get_column_letter(len(COLS) + 1 + i)

NUM_FMT = {
    "time_s": "0.000", "pos_deg": "0.000", "cmd_pos_deg": "0.000", "vel_rpm": "0.00",
    "cmd_vel_rpm": "0.00", "torque_pct": "0.0", "cmd_torque_pct": "0.0",
    "pos_err_deg": "0.000", "vel_err_rpm": "0.00", "torque_err_pct": "0.0",
}

HEAD_FILL = PatternFill("solid", fgColor="1F3864")
HEAD_FONT = Font(name=FONT, bold=True, color="FFFFFF")
BASE_FONT = Font(name=FONT)
BOLD_FONT = Font(name=FONT, bold=True)
THIN = Side(style="thin", color="BFBFBF")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def read_log(path):
    """Return OrderedDict axis -> list of row dicts (values converted to numbers)."""
    axes = OrderedDict()
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        missing = [c for c in COLS if c not in (reader.fieldnames or [])]
        if missing:
            sys.exit("This does not look like a myact_ctl log, missing columns: " + ", ".join(missing))
        for raw in reader:
            row = {}
            try:
                for c in COLS:
                    v = raw[c]
                    if c in TEXT_COLS:
                        row[c] = v
                    elif c in INT_COLS:
                        row[c] = int(float(v))
                    else:
                        row[c] = float(v)
            except (TypeError, ValueError):
                continue          # skip a truncated last line (e.g. after a crash)
            axes.setdefault(row["axis"], []).append(row)
    return axes


def style_header(ws, ncols):
    for c in range(1, ncols + 1):
        cell = ws.cell(row=1, column=c)
        cell.font = HEAD_FONT
        cell.fill = HEAD_FILL
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        cell.border = BORDER
    ws.row_dimensions[1].height = 32


def add_chart(ws, title, ytitle, cols, anchor, last):
    ch = ScatterChart()
    ch.title = title
    ch.style = 2
    ch.height = 9
    ch.width = 24
    ch.x_axis.title = "time (s)"
    ch.y_axis.title = ytitle
    ch.roundedCorners = False
    ch.legend.position = "b"
    ch.legend.overlay = False
    ch.title.overlay = False
    ch.x_axis.title.overlay = False
    ch.y_axis.title.overlay = False
    ch.x_axis.delete = False
    ch.y_axis.delete = False
    ch.x_axis.number_format = "0"
    xref = Reference(ws, min_col=1, min_row=2, max_row=last)
    for name in cols:
        idx = COLS.index(name) + 1
        yref = Reference(ws, min_col=idx, min_row=1, max_row=last)   # row 1 = series name
        s = Series(yref, xref, title_from_data=True)
        s.marker.symbol = "none"
        s.smooth = False
        s.graphicalProperties.line.width = 15000
        ch.series.append(s)
    ws.add_chart(ch, anchor)


def build_axis_sheet(wb, axis, rows):
    ws = wb.create_sheet(f"Axis{axis}")
    headers = COLS + EXTRA
    ws.append(headers)
    style_header(ws, len(headers))

    for r, row in enumerate(rows, start=2):
        ws.append([row[c] for c in COLS] + [
            f"={COL['cmd_pos_deg']}{r}-{COL['pos_deg']}{r}",
            f"={COL['cmd_vel_rpm']}{r}-{COL['vel_rpm']}{r}",
            f"={COL['cmd_torque_pct']}{r}-{COL['torque_pct']}{r}",
        ])
    last = len(rows) + 1

    # formats and fonts (loop only over used cells)
    for c, name in enumerate(headers, start=1):
        fmt = NUM_FMT.get(name)
        width = 12
        if name in ("phase",):
            width = 11
        if name in ("statusword", "error_code"):
            width = 12
        for r in range(2, last + 1):
            cell = ws.cell(row=r, column=c)
            cell.font = BASE_FONT
            if fmt:
                cell.number_format = fmt
        ws.column_dimensions[get_column_letter(c)].width = width

    ws.freeze_panes = "B2"
    ws.auto_filter.ref = f"A1:{get_column_letter(len(headers))}{last}"

    chart_col = get_column_letter(len(headers) + 2)
    add_chart(ws, f"Axis {axis}: position", "degrees", ["pos_deg", "cmd_pos_deg"], f"{chart_col}2", last)
    add_chart(ws, f"Axis {axis}: velocity", "rpm", ["vel_rpm", "cmd_vel_rpm"], f"{chart_col}21", last)
    add_chart(ws, f"Axis {axis}: torque", "% of rated", ["torque_pct", "cmd_torque_pct"], f"{chart_col}40", last)
    return ws, last


def mode_info(rows):
    """Highest mode_display seen: 8 CSP, 9 CSV, 10 CST, 0 = never enabled (monitor only)."""
    m = max((r["mode_display"] for r in rows), default=0)
    return {8: ("CSP position", "pos_err_deg", "deg"),
            9: ("CSV velocity", "vel_err_rpm", "rpm"),
            10: ("CST torque", "torque_err_pct", "% rated")}.get(m, ("monitor / none", None, ""))


def build_summary(wb, src_name, axes, sheets):
    ws = wb["Summary"]
    ws["A1"] = "MyActuator EtherCAT run summary"
    ws["A1"].font = Font(name=FONT, bold=True, size=14)
    ws["A2"] = "Source log:"
    ws["A2"].font = BOLD_FONT
    ws["B2"] = src_name
    ws["B2"].font = BASE_FONT
    ws["A3"] = ("All values below are formulas reading the Axis sheets. "
                "'RUN' statistics only use samples where the motor was enabled and running.")
    ws["A3"].font = Font(name=FONT, italic=True, color="595959")

    heads = ["Axis", "Mode (from drive)", "Samples logged", "Logged duration (s)", "Samples in RUN",
             "Position min (deg)", "Position max (deg)", "Max |velocity| in RUN (rpm)",
             "Mean torque in RUN (%)", "Max |torque| in RUN (%)", "Tracking error RMS in RUN",
             "Error unit", "Fault samples (error_code != 0)", "Max comm_bad"]
    hr = 5
    for c, h in enumerate(heads, start=1):
        cell = ws.cell(row=hr, column=c, value=h)
        cell.font = HEAD_FONT
        cell.fill = HEAD_FILL
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        cell.border = BORDER
        ws.column_dimensions[get_column_letter(c)].width = 17
    ws.row_dimensions[hr].height = 48
    ws.column_dimensions["A"].width = 12
    ws.column_dimensions["B"].width = 20

    r = hr + 1
    for axis, rows in axes.items():
        sh, last = sheets[axis]
        n = f"Axis{axis}"

        def rng(name):
            return f"{n}!${COL[name]}$2:${COL[name]}${last}"

        run = f'{rng("phase")},"RUN"'
        mode_name, err_col, err_unit = mode_info(rows)
        ws.cell(row=r, column=1, value=axis)
        ws.cell(row=r, column=2, value=(
            f'=IF(MAX({rng("mode_display")})=8,"CSP position",'
            f'IF(MAX({rng("mode_display")})=9,"CSV velocity",'
            f'IF(MAX({rng("mode_display")})=10,"CST torque","monitor / none")))'))
        ws.cell(row=r, column=3, value=f'=COUNT({rng("time_s")})')
        ws.cell(row=r, column=4, value=f'=MAX({rng("time_s")})-MIN({rng("time_s")})')
        ws.cell(row=r, column=5, value=f'=COUNTIF({run})')
        ws.cell(row=r, column=6, value=f'=MIN({rng("pos_deg")})')
        ws.cell(row=r, column=7, value=f'=MAX({rng("pos_deg")})')
        ws.cell(row=r, column=8, value=(
            f'=IF(E{r}=0,"n/a",SUMPRODUCT(MAX(({rng("phase")}="RUN")*ABS({rng("vel_rpm")}))))'))
        ws.cell(row=r, column=9, value=f'=IFERROR(AVERAGEIFS({rng("torque_pct")},{run}),"n/a")')
        ws.cell(row=r, column=10, value=(
            f'=IF(E{r}=0,"n/a",SUMPRODUCT(MAX(({rng("phase")}="RUN")*ABS({rng("torque_pct")}))))'))
        if err_col:
            ws.cell(row=r, column=11, value=(
                f'=IF(E{r}=0,"n/a",SQRT(SUMPRODUCT(({rng("phase")}="RUN")*{rng(err_col)}^2)/E{r}))'))
            ws.cell(row=r, column=12, value=err_unit)
        else:
            ws.cell(row=r, column=11, value="n/a")
            ws.cell(row=r, column=12, value="-")
        ws.cell(row=r, column=13, value=f'=COUNTIF({rng("error_code")},"<>0x0000")')
        ws.cell(row=r, column=14, value=f'=MAX({rng("comm_bad")})')

        for c in range(1, len(heads) + 1):
            cell = ws.cell(row=r, column=c)
            cell.font = BASE_FONT
            cell.border = BORDER
            cell.alignment = Alignment(horizontal="center")
        for c, fmt in ((4, "0.00"), (6, "0.00"), (7, "0.00"), (8, "0.00"), (9, "0.0"),
                       (10, "0.0"), (11, "0.000")):
            ws.cell(row=r, column=c).number_format = fmt
        r += 1

    notes = [
        "Notes",
        "- Mode is read from the drive's mode_display (0x6061): 8 = CSP, 9 = CSV, 10 = CST.",
        "- Tracking error = command minus actual, in the unit of the motor's own mode "
        "(deg for CSP, rpm for CSV, % of rated torque for CST). It is computed in columns T-V of each Axis sheet.",
        "- Degrees and rpm use the --cpr value given to myact_ctl (default 131072 counts/rev). "
        "If that value is wrong for a motor, its deg/rpm columns are wrong; the raw count columns are not.",
        "- Torque is in % of the motor's rated torque as reported by the drive (0x6077 / 10).",
        "- The log starts once all motors are in EtherCAT OP, so the first rows are the start of the enable sequence.",
        "- Statistics are over the whole logged time unless the heading says RUN.",
    ]
    r += 1
    for i, t in enumerate(notes):
        ws.cell(row=r + i, column=1, value=t).font = BOLD_FONT if i == 0 else BASE_FONT
    ws.freeze_panes = "A6"


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + ".xlsx"
    axes = read_log(src)
    if not axes:
        sys.exit("No data rows found in " + src)

    wb = Workbook()
    wb.active.title = "Summary"
    sheets = {a: build_axis_sheet(wb, a, rows) for a, rows in axes.items()}
    build_summary(wb, os.path.basename(src), axes, sheets)
    wb.save(dst)
    total = sum(len(v) for v in axes.values())
    print(f"wrote {dst}: {len(axes)} motor(s), {total} samples")


if __name__ == "__main__":
    main()
