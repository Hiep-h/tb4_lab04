"""Luu ban do bang nav2_map_server map_saver_cli (chay nen, khong lam dung chuong trinh goi)."""
import os
import subprocess
import threading
import time


class MapSaver:
    """Luu ban do bang nav2_map_server map_saver_cli (chay nen, khong lam dung giao dien)."""

    def __init__(self, map_dir):
        self.map_dir = os.path.expanduser(map_dir)
        self.busy = False
        self.last = 'chưa lưu'

    def save_async(self, done=None):
        if self.busy:
            return
        self.busy = True

        def work():
            try:
                os.makedirs(self.map_dir, exist_ok=True)
                prefix = os.path.join(self.map_dir, 'map')
                res = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix, '--ros-args', '-p',
                     'use_sim_time:=true'], capture_output=True, text=True, timeout=40)
                ok = res.returncode == 0 and os.path.exists(prefix + '.yaml') and os.path.exists(prefix + '.pgm')
                self.last = ('Đã lưu %s lúc %s' % (prefix + '.yaml', time.strftime('%H:%M:%S'))) if ok else \
                    'LỖI lưu map: ' + (res.stderr or res.stdout)[-120:].replace('\n', ' ')
            except Exception as exc:  # noqa: BLE001
                self.last = 'LỖI lưu map: %s' % exc
            finally:
                self.busy = False
                if done:
                    done(self.last)

        threading.Thread(target=work, daemon=True).start()

    def save_blocking(self):
        """Dung khi dong bang: cho den khi luu xong (toi da ~40 s)."""
        ev = threading.Event()
        self.save_async(lambda _msg: ev.set())
        ev.wait(45)
