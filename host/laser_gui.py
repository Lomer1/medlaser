import queue
import threading
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

# Ответы платы короткие, в логе показываем их смысл.
LOG_TEXT = {
    "err flow": "Ошибка: чиллер не работает",
    "evt stop flow": "Ошибка: чиллер остановился, серия прервана",
    "err not armed": "Ошибка: не включено «Разрешить пуск»",
    "err timing": "Ошибка: заряд и поджиг не укладываются в период на этой частоте",
    "err freq": "Ошибка: частота вне 1–10 Гц или серия ещё идёт",
    "err charge": "Ошибка: заряд вне 1–800 мс или серия ещё идёт",
    "err fire": "Ошибка: поджиг вне 1–20 мс или серия ещё идёт",
    "err busy": "Ошибка: серия идёт, сначала Stop",
    "err pol": "Ошибка: неверная полярность",
    "err unknown": "Ошибка: неизвестная команда",
    "evt stop timeout": "Ошибка: нет связи с компьютером, серия остановлена",
}


class App:
    def __init__(self, root):
        self.root = root
        root.title("Medlaser")
        self.ser = None
        self.rxq = queue.Queue()
        self.txq = []
        self.waiting = False
        self.stop_read = threading.Event()
        self.quiet = False
        self.ping_acc = 0

        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        self.port = ttk.Combobox(top, width=28, state="readonly")
        self.port.grid(row=0, column=0, padx=4)
        ttk.Button(top, text="Обновить", command=self.refresh).grid(row=0, column=1)
        ttk.Button(top, text="Открыть", command=self.connect).grid(row=0, column=2, padx=4)
        ttk.Button(top, text="Закрыть", command=self.disconnect).grid(row=0, column=3)

        form = ttk.Frame(root, padding=8)
        form.grid(row=1, column=0, sticky="ew")
        self.freq = tk.IntVar(value=1)
        self.level = tk.IntVar(value=0)
        self.charge = tk.IntVar(value=150)
        self.fire = tk.IntVar(value=1)
        self.count = tk.IntVar(value=1)
        self.armed = tk.IntVar(value=0)
        self.freq_box = self._row(form, 0, "Частота, Гц", self.freq, 1, 10)
        self.level_box = self._row(form, 1, "Мощность, 0…2000", self.level, 0, 2000)
        self.charge_box = self._row(form, 2, "Заряд конденсатора, мс", self.charge, 1, 800)
        self.fire_box = self._row(form, 3, "Поджиг, мс", self.fire, 1, 20)
        self.count_box = self._row(form, 4, "Импульсов, 0 = пока не Stop", self.count, 0, 100000)
        ttk.Checkbutton(form, text="Разрешить пуск", variable=self.armed).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(6, 0)
        )

        wires = (
            "PA0 — заряд конденсатора\n"
            "PA1 — поджиг\n"
            "PA4 — мощность\n"
            "PA5 — педаль\n"
            "PA6 — расход воды\n"
            "GND — земля гребёнки\n"
            "5V не подключать"
        )
        ttk.Label(form, text=wires, wraplength=520).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        self.flow = ttk.Label(form, text="Расход, л/мин: —")
        self.flow.grid(row=7, column=0, columnspan=2, sticky="w", pady=(8, 0))

        btns = ttk.Frame(root, padding=8)
        btns.grid(row=2, column=0, sticky="ew")
        ttk.Button(btns, text="Start", command=self.start).grid(row=0, column=0, padx=4)
        ttk.Button(btns, text="Stop", command=self.stop).grid(row=0, column=1, padx=4)

        self.log = tk.Text(root, height=14, width=78)
        self.log.grid(row=3, column=0, padx=8, pady=8)
        self.refresh()
        root.after(200, self.drain)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _row(self, parent, row, label, var, lo, hi, increment=1, fmt=None):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        box = ttk.Spinbox(
            parent, from_=lo, to=hi, increment=increment, textvariable=var, width=10
        )
        if fmt:
            box.configure(format=fmt)
        box.grid(row=row, column=1, sticky="w")
        box.configure(command=self._sync_timing)
        box.bind("<FocusOut>", lambda _event: self._sync_timing())
        box.bind("<Return>", lambda _event: self._sync_timing())
        return box

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.port["values"] = ports
        if ports and not self.port.get():
            self.port.set(ports[0])

    def connect(self):
        self.disconnect()
        name = self.port.get()
        if not name:
            self.write_log("Порт не выбран")
            return
        self.ser = serial.Serial(name, 115200, timeout=0.1)
        self.stop_read.clear()
        threading.Thread(target=self.reader, daemon=True).start()
        self.enqueue(["watch 1", "status"])
        self._sync_timing()

    def disconnect(self):
        self.stop_read.set()
        if self.ser is not None:
            try:
                self.ser.write(b"stop\n")
                self.ser.close()
            except serial.SerialException:
                pass
            self.ser = None
        self.txq.clear()
        self.waiting = False
        self.quiet = False
        self.ping_acc = 0

    def reader(self):
        buf = b""
        while not self.stop_read.is_set() and self.ser is not None:
            try:
                chunk = self.ser.read(64)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.rxq.put(line.decode("ascii", "replace").strip())

    def enqueue(self, cmds):
        self.txq.extend(cmds)
        self.pump()

    def pump(self):
        if self.waiting or not self.txq or self.ser is None:
            return
        cmd = self.txq.pop(0)
        self.quiet = cmd == "ping"
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
        except serial.SerialException as exc:
            self.write_log(str(exc))
            return
        self.waiting = True
        if not self.quiet:
            self.write_log("> " + cmd)

    def drain(self):
        try:
            while True:
                line = self.rxq.get_nowait()
                if line.startswith("flow "):
                    self.flow.configure(text="Расход, л/мин: " + line[5:])
                    continue
                if not (self.quiet and line == "ok"):
                    self.write_log(LOG_TEXT.get(line, line))
                self.waiting = False
                self.quiet = False
                self.pump()
        except queue.Empty:
            pass
        if self.ser is not None and not self.waiting:
            self.ping_acc += 100
            if self.ping_acc >= 400:
                self.ping_acc = 0
                self.enqueue(["ping"])
        self.root.after(100, self.drain)

    def write_log(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def _sync_timing(self):
        if self.ser is None or not hasattr(self, "freq_box"):
            return
        self.enqueue(
            [
                f"charge {self._box_int(self.charge_box, 1, 800)}",
                f"fire {self._box_int(self.fire_box, 1, 20)}",
                f"freq {self._box_int(self.freq_box, 1, 10)}",
                f"level {self._box_int(self.level_box, 0, 2000)}",
                f"count {self._box_int(self.count_box, 0, 100000)}",
            ]
        )

    def _box_int(self, box, lo, hi):
        try:
            value = int(float(box.get()))
        except (tk.TclError, ValueError):
            value = lo
        return min(hi, max(lo, value))

    def start(self):
        self.enqueue(
            [
                f"charge {self._box_int(self.charge_box, 1, 800)}",
                f"fire {self._box_int(self.fire_box, 1, 20)}",
                f"freq {self._box_int(self.freq_box, 1, 10)}",
                f"level {self._box_int(self.level_box, 0, 2000)}",
                f"count {self._box_int(self.count_box, 0, 100000)}",
                f"arm {1 if self.armed.get() else 0}",
                "run",
            ]
        )

    def stop(self):
        self.txq.clear()
        self.waiting = False
        if self.ser is not None:
            try:
                self.ser.write(b"stop\n")
            except serial.SerialException:
                return
            self.write_log("> stop")

    def on_close(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
