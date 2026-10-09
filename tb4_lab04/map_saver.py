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
                err = ''
                ok = False
                # 1) map_saver_cli voi timeout dai (mac dinh chi 2 s, de that bai khi mo phong cham); thu toi 3 lan
                for _ in range(3):
                    res = subprocess.run(
                        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix, '--ros-args', '-p',
                         'use_sim_time:=true', '-p', 'save_map_timeout:=30.0'], capture_output=True, text=True, timeout=90)
                    ok = res.returncode == 0 and os.path.exists(prefix + '.yaml') and os.path.exists(prefix + '.pgm')
                    if ok:
                        break
                    err = (res.stderr or res.stdout)[-120:].replace('\n', ' ')
                # 2) du phong: dich vu luu ban do cua chinh slam_toolbox (khong phu thuoc /map subscription)
                if not ok:
                    res = subprocess.run(
                        ['ros2', 'service', 'call', '/slam_toolbox/save_map', 'slam_toolbox/srv/SaveMap',
                         '{name: {data: "%s"}}' % prefix], capture_output=True, text=True, timeout=60)
                    ok = os.path.exists(prefix + '.yaml') and os.path.exists(prefix + '.pgm') and 'result=0' in res.stdout.replace(' ', '')
                    err = err or (res.stderr or res.stdout)[-120:].replace('\n', ' ')
                self.last = ('Đã lưu %s lúc %s' % (prefix + '.yaml', time.strftime('%H:%M:%S'))) if ok else 'LỖI lưu map: ' + err
            except Exception as exc:  # noqa: BLE001
                self.last = 'LỖI lưu map: %s' % exc
            finally:
                self.busy = False
                if done:
                    done(self.last)

        threading.Thread(target=work, daemon=True).start()

    def save_blocking(self):
        """Dung khi dong bang: cho den khi luu xong (toi da vai phut)."""
        ev = threading.Event()
        self.save_async(lambda _msg: ev.set())
        ev.wait(200)
