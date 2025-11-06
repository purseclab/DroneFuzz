#!/usr/bin/env python3
import os, sys, glob, argparse, pickle, yaml, logging
import numpy as np
from tqdm import tqdm

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.append(HERE)

SCRIPTS_DIR = os.path.join(HERE, "scripts")
if os.path.isdir(SCRIPTS_DIR) and SCRIPTS_DIR not in sys.path:
    sys.path.append(SCRIPTS_DIR)

from plot_servo_values import parse_ardupilot_bin
from types import SimpleNamespace

from minimal_poc import FuzzConfig, _ssl_finetune_head, FuzzState

logging.basicConfig(level=logging.INFO, format="%(message)s")
log = logging.getLogger("offline")

def bin_to_series(servo_data):
    """Convert parse_ardupilot_bin arrays to your {servo1_raw..servo4_raw} dict list."""
    C1, C2, C3, C4 = (servo_data["C1"], servo_data["C2"], servo_data["C3"], servo_data["C4"])
    N = len(servo_data["timestamp"])
    out = []
    for i in range(N):
        out.append({
            "servo1_raw": float(C1[i]),
            "servo2_raw": float(C2[i]),
            "servo3_raw": float(C3[i]),
            "servo4_raw": float(C4[i]),
        })
    return out

def add_to_fuzz_queue(self, msgs, score: float):
    item = SimpleNamespace(priority=float(score), msgs=list(msgs))
    self.fuzzer_queue.append(item)
    # Keep highest-priority first to mimic your online behavior
    self.fuzzer_queue.sort(key=lambda x: x.priority, reverse=True)

def _prepare_logs_for_dtw_plot(off, src_bin_path: str, baseline_bin_path: str = None):
    """
    Ensure fuzzer_temp_dir/logs/ has what save_diff_img() expects.
    - baseline:  {off.fuzz_enum_mode:08d}.BIN
    - anomaly:   {log_id:08d}.BIN (and write LASTLOG.TXT=log_id)
    """
    import shutil, random
    logs_dir = os.path.join(off.fuzzer_temp_dir, "logs")
    os.makedirs(logs_dir, exist_ok=True)

    # pick an integer id for this anomaly file
    log_id = random.randint(1, 99999999)

    # copy the anomaly BIN
    anom_dst = os.path.join(logs_dir, f"{log_id:08d}.BIN")
    shutil.copy2(src_bin_path, anom_dst)

    # choose a baseline: given path, or first golden, or fall back to the same file
    if baseline_bin_path and os.path.exists(baseline_bin_path):
        base_src = baseline_bin_path
    elif off.golden_rc_vals:
        # If you have raw golden series only, you may not have a BIN; just reuse the same BIN.
        base_src = src_bin_path
    else:
        base_src = src_bin_path

    base_dst = os.path.join(logs_dir, f"{off.fuzz_enum_mode:08d}.BIN")
    if not os.path.exists(base_dst):
        shutil.copy2(base_src, base_dst)

    # write LASTLOG.TXT with the anomaly id
    with open(os.path.join(logs_dir, "LASTLOG.TXT"), "w") as f:
        f.write(str(log_id))

    # nothing to return; DTW will read the files on its own

class _NoopCoverage:
    def update(self): pass
    def archive_data(self, filename=None): pass

class OfflineHarness:
    """
    Minimal shell that sets the attributes your existing private/oracle
    methods use, then CALLS them. No code duplication.
    """
    def __init__(self, out_dir, base_cfg: dict):
        self.config = dict(base_cfg or {})
        self.fuzzer_temp_dir = out_dir
        os.makedirs(self.fuzzer_temp_dir, exist_ok=True)
        self.fuzzer_temp_input_dir = os.path.join(self.fuzzer_temp_dir, "input")
        os.makedirs(self.fuzzer_temp_input_dir, exist_ok=True)
        
        def _ssl_finetune_head_method(model_ae, device):
            return _ssl_finetune_head(self, model_ae, device)

        self._ssl_finetune_head = _ssl_finetune_head_method

        self.script_dir = os.path.join(HERE, "scripts")  # used by save_diff_img in your DTW path
        self.golden_rc_vals = []
        self.fuzzer_stats = {
            "dtw_threshold": [],
            "simulations_completed": 0,
            "potential_crashes": 0,
            "messages_sent": 0,
            "last_mission_time": 0.0,
        }
        self.fuzzer_state = None
        self.fuzzer_stats["bugs_dtw"] = 0
        self.fuzzer_stats["bugs_lstm"] = 0

        self.fuzz_enum_mode = 0
        self.last_scores = {"dtw": float("nan"), "z_dtw": float("nan"),
                            "lstm_err": float("nan"), "p_anom": float("nan")}
        self.last_dtw_distance = float("nan")
        self.fuzzer_queue = []            # simple list of items with .priority
        self.fuzz_msgs = []               # messages written to mkstemp file
        self.fuzzer_state = FuzzState.Init
        self.add_to_fuzz_queue = add_to_fuzz_queue.__get__(self, OfflineHarness)

        # thresholds populated by _sigma_calc_dtw/_sigma_calc_lstm
        self.min_fuzz_threshold = None
        self.max_fuzz_threshold = None
        self.dtw_mean = None
        self.dtw_std = None
        self.lstm_err_min = None
        self.lstm_err_max = None
        self.lstm_err_mean = None
        self.lstm_err_std = None

        self.lstm_window = int(self.config.get("lstm_window", 128))
        self.lstm_stride = int(self.config.get("lstm_stride", 64))
        self.lstm_hidden = int(self.config.get("lstm_hidden", 32))

        # SSL knobs (needed by oracle_lstm)
        self.ssl_buffer = []
        self.ssl_max_buf = int(self.config.get("ssl_max_buf", 2048))
        self.ssl_min_train = int(self.config.get("ssl_min_train", 256))
        self.ssl_train_every = int(self.config.get("ssl_train_every", 3))
        self.ssl_head_lr = float(self.config.get("ssl_head_lr", 1e-4))
        self.ssl_head_batch = int(self.config.get("ssl_head_batch", 128))
        self.ssl_head_epochs = int(self.config.get("ssl_head_epochs", 3))
        self.ssl_normal_z = float(self.config.get("ssl_normal_z", 0.2))
        self.ssl_anom_z = float(self.config.get("ssl_anom_z", 2.0))
        self.ssl_w_normal = float(self.config.get("ssl_w_normal", 0.5))
        self.ssl_w_anom = float(self.config.get("ssl_w_anom", 1.0))
        self.ssl_head_path = os.path.join(self.fuzzer_temp_dir, "lstm_head.pt")
        self.ssl_last_train_sims = -1

        self.coverage_class = _NoopCoverage()

        self._series_to_matrix  = FuzzConfig._series_to_matrix.__get__(self, OfflineHarness)
        self._build_windows     = FuzzConfig._build_windows.__get__(self, OfflineHarness)
        self._normalize         = FuzzConfig._normalize.__get__(self, OfflineHarness)
        self._ssl_push          = FuzzConfig._ssl_push.__get__(self, OfflineHarness)

        self._sigma_calc_dtw    = FuzzConfig._sigma_calc_dtw.__get__(self, OfflineHarness)
        self._sigma_calc_lstm   = FuzzConfig._sigma_calc_lstm.__get__(self, OfflineHarness)
        self.calculate_dtw      = FuzzConfig.calculate_dtw.__get__(self, OfflineHarness)

        # oracles
        self.oracle_dtw         = FuzzConfig.oracle_dtw.__get__(self, OfflineHarness)
        self.oracle_lstm        = FuzzConfig.oracle_lstm.__get__(self, OfflineHarness)

    def persist_rc_pickle_and_yaml(self):
        pickle_file = os.path.join(os.getcwd(), "rcou_vals.pkl")
        with open(pickle_file, "wb") as f:
            f.write(pickle.dumps(self.golden_rc_vals))
        mod_config_file = os.path.join(os.getcwd(), "cal_config.yaml")
        cfg = dict(self.config)
        cfg["calibration_threshold"] = [
            float(self.min_fuzz_threshold),
            float(self.max_fuzz_threshold),
        ]
        with open(mod_config_file, "w") as f:
            yaml.dump(cfg, f)

def load_goldens_from_dir(logs_dir, rc_log_filter=True):
    bins = sorted(glob.glob(os.path.join(logs_dir, "*.BIN")))
    if not bins:
        raise FileNotFoundError(f"No BINs in {logs_dir}")
    goldens = []
    for p in bins:
        try:
            d = parse_ardupilot_bin(p, rc_log_filter=rc_log_filter, channels=["C1","C2","C3","C4"])
            goldens.append(bin_to_series(d))
        except Exception as e:
            log.warning("Skipping %s: %s", p, e)
    if not goldens:
        raise RuntimeError("No usable BINs after parsing.")
    return goldens

def update_tqdm_postfix(off: OfflineHarness, pbar: tqdm):
    dtw_min_val = off.min_fuzz_threshold
    dtw_max_val = off.max_fuzz_threshold
    pbar.set_postfix({
        "sims": off.fuzzer_stats["simulations_completed"],
        "msgs": off.fuzzer_stats["messages_sent"],
        "dtw_min": "NaN" if dtw_min_val is None else f"{dtw_min_val:.6f}",
        "dtw_max": "NaN" if dtw_max_val is None else f"{dtw_max_val:.6f}",
        "bugs_dtw": off.fuzzer_stats.get("bugs_dtw", 0),
        "bugs_lstm": off.fuzzer_stats.get("bugs_lstm", 0),
    })

def main():
    ap = argparse.ArgumentParser("Offline: reuse minimal_poc to calibrate from BINs and score BINs")
    logs_dir = os.path.join(HERE, "calibration_logs")
    ap.add_argument("--out_dir",  required=True, help="Where minimal_poc artifacts go (lstm_ae.pt, *.npy, head, etc.)")
    ap.add_argument("--rc-log-filter", action="store_true", help='Respect "LOG RC"/"STOP RC"')
    ap.add_argument("--score_bin", help="Optional: score this BIN offline with both oracles")
    ap.add_argument("--lstm-window", type=int, default=128)
    ap.add_argument("--lstm-stride", type=int, default=64)
    ap.add_argument("--lstm-hidden", type=int, default=32)
    ap.add_argument("--lstm-epochs", type=int, default=100)
    ap.add_argument("--lstm-batch", type=int, default=64)
    ap.add_argument("--lstm-lr", type=float, default=1e-3)
    ap.add_argument("--score_dir", help="If set, score all BINs in this folder (offline, no SITL)")
    ap.add_argument("--scores_csv", default="scores.csv", help="CSV filename to write under out_dir")

    args = ap.parse_args()

    base_cfg = {
        "lstm_window": args.lstm_window,
        "lstm_stride": args.lstm_stride,
        "lstm_hidden": args.lstm_hidden,
        "lstm_epochs": args.lstm_epochs,
        "lstm_batch": args.lstm_batch,
        "lstm_lr": args.lstm_lr,
    }

    off = OfflineHarness(args.out_dir, base_cfg)

    # 1) collect goldens
    off.golden_rc_vals = load_goldens_from_dir(logs_dir, rc_log_filter=args.rc_log_filter)
    log.info("Loaded %d golden BINs from %s", len(off.golden_rc_vals), logs_dir)

    # 2) calibrate with exact routines
    off._sigma_calc_dtw()
    log.info("DTW thresholds: min=%.6f max=%.6f (mean=%.6f std=%.6f)",
             off.min_fuzz_threshold, off.max_fuzz_threshold, off.dtw_mean, off.dtw_std)
    off._sigma_calc_lstm()
    log.info("LSTM AE band: mean=%.6e std=%.6e  -> min=%.6e max=%.6e",
             off.lstm_err_mean, off.lstm_err_std, off.lstm_err_min, off.lstm_err_max)

    # 3) persist artifacts in the same places online code expects
    off.persist_rc_pickle_and_yaml()
    log.info("Wrote rcou_vals.pkl + cal_config.yaml in %s; model+norm in %s", os.getcwd(), args.out_dir)

    # 4) optional: score one BIN
    if args.score_bin:
        _prepare_logs_for_dtw_plot(off, args.score_bin)
        d = parse_ardupilot_bin(args.score_bin, rc_log_filter=args.rc_log_filter, channels=["C1","C2","C3","C4"])
        series = bin_to_series(d)
        off.rcou_vals = series
        off.fuzzer_stats["simulations_completed"] += 1
        # DTW first (sets weak labels/last_scores), then LSTM
        prev_crashes = off.fuzzer_stats["potential_crashes"]
        off.oracle_dtw()
        if off.fuzzer_stats["potential_crashes"] > prev_crashes:
            off.fuzzer_stats["bugs_dtw"] += 1
        prev_crashes = off.fuzzer_stats["potential_crashes"]
        off.oracle_lstm()
        if off.fuzzer_stats["potential_crashes"] > prev_crashes:
            off.fuzzer_stats["bugs_lstm"] += 1
        log.info("DTW dist=%.6f  z=%.2f", float(off.last_scores["dtw"]), float(off.last_scores["z_dtw"]))
        log.info("LSTM err=%.6e  p_anom=%s",
                float(off.last_scores["lstm_err"]),
                "nan" if np.isnan(off.last_scores["p_anom"]) else f"{off.last_scores['p_anom']:.3f}")

    # Default to scoring the calibration_logs folder if no target provided
    if not args.score_dir and not args.score_bin:
        log.info("No --score_dir/--score_bin provided; defaulting to --score_dir %s", logs_dir)
        args.score_dir = logs_dir

    # 5) optional: score an entire folder (offline)
    if args.score_dir:
        bin_paths = sorted(glob.glob(os.path.join(args.score_dir, "*.BIN")))
        if not bin_paths:
            log.error("No BINs found in %s", args.score_dir)
            return 1

        csv_path = os.path.join(args.out_dir, args.scores_csv)
        with open(csv_path, "w", newline="") as f:
            import csv
            w = csv.writer(f)
            w.writerow(["bin", "dtw", "z_dtw", "lstm_err", "p_anom", "flag_dtw", "flag_lstm"])

            pbar = tqdm(total=len(bin_paths), desc="Offline scoring")
            for i, p in enumerate(bin_paths, start=1):
                # “iteration” start
                log.info("Starting offline iteration %d on %s", i, os.path.basename(p))

                # load BIN → rcou_vals (no SITL)
                try:
                    _prepare_logs_for_dtw_plot(off, p)
                    d = parse_ardupilot_bin(p, rc_log_filter=args.rc_log_filter, channels=["C1","C2","C3","C4"])
                except Exception as e:
                    log.warning("Skipping %s: %s", p, e)
                    pbar.update(1)
                    continue

                off.rcou_vals = bin_to_series(d)
                off.fuzzer_stats["simulations_completed"] = i

                # run DTW then LSTM (LSTM uses DTW for weak labels)
                prev_crashes = off.fuzzer_stats["potential_crashes"]
                off.oracle_dtw()
                flagged_dtw = off.fuzzer_stats["potential_crashes"] > prev_crashes
                if flagged_dtw:
                    off.fuzzer_stats["bugs_dtw"] += 1

                prev_crashes = off.fuzzer_stats["potential_crashes"]
                off.oracle_lstm()
                flagged_lstm = off.fuzzer_stats["potential_crashes"] > prev_crashes
                if flagged_lstm:
                    off.fuzzer_stats["bugs_lstm"] += 1

                # write one row to CSV
                w.writerow([
                    os.path.basename(p),
                    f"{float(off.last_scores['dtw']):.6f}",
                    f"{float(off.last_scores['z_dtw']):.2f}",
                    f"{float(off.last_scores['lstm_err']):.6e}",
                    "" if np.isnan(off.last_scores['p_anom']) else f"{off.last_scores['p_anom']:.3f}",
                    int(flagged_dtw),
                    int(flagged_lstm),
                ])

                # “iteration” end visuals
                pbar.update(1)
                update_tqdm_postfix(off, pbar)

        log.info("Wrote offline scores CSV: %s", csv_path)
        # --- final summary ---
        total_bins = len(bin_paths)
        bugs_dtw  = off.fuzzer_stats.get("bugs_dtw", 0)
        bugs_lstm = off.fuzzer_stats.get("bugs_lstm", 0)
        both = bugs_dtw + bugs_lstm
        log.info("============================================================")
        log.info("Offline Scoring Summary")
        log.info("Total BINs processed: %d", total_bins)
        log.info("DTW flagged anomalies : %d", bugs_dtw)
        log.info("LSTM flagged anomalies: %d", bugs_lstm)
        log.info("Combined total (may overlap): %d", both)
        log.info("============================================================")

        pbar.close()
        print(f"[SUMMARY] DTW={bugs_dtw}, LSTM={bugs_lstm}, total={both}")




if __name__ == "__main__":
    sys.exit(main())
