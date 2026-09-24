#!/usr/bin/env python3
"""Run one online SLAM replay under gdb and dump all thread stacks on a stall.

A stall is declared when stderr.log has not grown for --idle-seconds. The
watcher then interrupts the inferior (SIGINT), gdb prints
`thread apply all bt`, and the process is killed. Works with
kernel.yama.ptrace_scope=1 because gdb is the inferior's parent.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--idle-seconds', type=int, default=120)
    parser.add_argument('--max-seconds', type=int, default=3000)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    out = args.out_dir
    out.mkdir(parents=True)
    run = out / 'run'
    command = [*command, '--out-dir', str(run)]
    gdb = ['gdb', '-q', '-batch', '-ex', 'set pagination off', '-ex', 'handle SIGINT stop nopass',
           '-ex', 'handle SIGPIPE nostop noprint pass', '-ex', 'run',
           '-ex', 'thread apply all bt 40', '-ex', 'kill', '--args', *command]
    stderr_path = out / 'stderr.log'
    with (out / 'gdb_stdout.log').open('x') as stdout, stderr_path.open('x') as stderr:
        proc = subprocess.Popen(gdb, stdout=stdout, stderr=stderr, start_new_session=True)
        start = last_growth = time.monotonic()
        last_size = 0
        verdict = None
        while proc.poll() is None:
            time.sleep(5)
            size = stderr_path.stat().st_size
            now = time.monotonic()
            if size != last_size:
                last_size, last_growth = size, now
            if now - start > args.max_seconds:
                verdict = 'max_seconds'
            elif now - last_growth > args.idle_seconds:
                verdict = 'stall'
            if verdict:
                inferior = subprocess.run(['pgrep', '-P', str(proc.pid)], capture_output=True, text=True).stdout.split()
                for pid in inferior:
                    os.kill(int(pid), signal.SIGINT)
                try:
                    proc.wait(timeout=180)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                break
    (out / 'verdict.txt').write_text(f'{verdict or "exited"} exit={proc.returncode}\n')
    print(verdict or 'exited', proc.returncode, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
