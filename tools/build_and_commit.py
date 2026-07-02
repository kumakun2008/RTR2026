import subprocess
import sys
import datetime
import os

ENVIRONMENTS = [
    "RTR_Main_Board",
    "RTR_Pitot_Board",
    "RTR_Display_Board",
    "RTR_GPS_Board",
    "RTR_Speaker_Board",
    "RTR_Altimeter_Board",
    "RTR_Rudder_Board",
    "RTR_Sim_Bridge"
]

def run_command(command, cwd=None):
    print(f"Executing: {' '.join(command)}")
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, cwd=cwd, encoding='utf-8', errors='ignore')
    return result.returncode == 0, result.stdout, result.stderr

def main():
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pio_path = os.path.expanduser(r"~\.platformio\penv\Scripts\pio.exe")
    if not os.path.exists(pio_path):
        pio_path = "pio" # fallback to global pio

    print("=== Start Building All PlatformIO Environments ===")
    success = True
    results = {}

    for env in ENVIRONMENTS:
        print(f"\nBuilding environment: {env}...")
        cmd = [pio_path, "run", "-e", env]
        ok, stdout, stderr = run_command(cmd, cwd=project_dir)
        if ok:
            print(f"[SUCCESS] Build passed for {env}")
            results[env] = "SUCCESS"
        else:
            print(f"[FAILED] Build failed for {env}")
            print(stderr)
            results[env] = "FAILED"
            success = False

    print("\n=== Build Summary ===")
    for env, status in results.items():
        print(f"{env}: {status}")

    if not success:
        print("\n[ERROR] One or more environments failed to build. Aborting git commit/push.")
        sys.exit(1)

    print("\n=== Start Git Commit and Push ===")
    # Git stage
    ok, out, err = run_command(["git", "add", "."], cwd=project_dir)
    if not ok:
        print(f"Git add failed: {err}")
        sys.exit(1)

    # Git commit
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    commit_msg = f"Auto-commit: Successful build of all nodes on {timestamp}"
    ok, out, err = run_command(["git", "commit", "-m", commit_msg], cwd=project_dir)
    if not ok:
        if "nothing to commit" in out or "no changes added to commit" in out:
            print("No changes to commit.")
            sys.exit(0)
        print(f"Git commit failed: {err}")
        sys.exit(1)

    # Git push
    ok, out, err = run_command(["git", "push", "origin", "main"], cwd=project_dir)
    if not ok:
        print(f"Git push failed: {err}")
        sys.exit(1)

    print("\n[OK] Build, Commit, and Push completed successfully!")

if __name__ == "__main__":
    main()
