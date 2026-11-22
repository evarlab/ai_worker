"""Desktop buttons controlling the existing collector, without replacing Quest XR."""
import json
from pathlib import Path
import ssl
import tkinter as tk
from tkinter import ttk
from urllib.request import Request, urlopen
from urllib.error import HTTPError


def main():
    root = tk.Tk(); root.title('人間実演の記録 — Quest + ContactGlove'); root.geometry('720x450')
    frame = ttk.Frame(root, padding=20); frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='人間実演の記録', font=('', 22)).pack(anchor='w')
    ttk.Label(frame, text='QuestのEnter XRとグローブ配信を起動してから開始してください。').pack(anchor='w', pady=8)
    task = tk.StringVar(value='human_demo'); outcome = tk.StringVar(value='success')
    ttk.Label(frame, text='タスク名').pack(anchor='w'); ttk.Entry(frame, textvariable=task).pack(fill='x')
    status = tk.StringVar(value='接続確認中'); error = tk.StringVar(); path = tk.StringVar()
    ttk.Label(frame, textvariable=status, font=('', 16)).pack(anchor='w', pady=15)
    buttons = ttk.Frame(frame); buttons.pack(fill='x')
    start = ttk.Button(buttons, text='● 記録開始', command=lambda: action('start')); start.pack(side='left', padx=5, ipady=12)
    stop = ttk.Button(buttons, text='■ 記録停止', command=lambda: action('stop')); stop.pack(side='left', padx=5, ipady=12)
    ttk.Combobox(buttons, textvariable=outcome, values=['success','failure','unspecified'], state='readonly', width=13).pack(side='left', padx=5)
    ttk.Label(frame, textvariable=path, wraplength=670).pack(anchor='w', pady=12)
    ttk.Label(frame, textvariable=error, foreground='red', wraplength=670).pack(anchor='w')
    ttk.Label(frame, text='画面を閉じても記録サーバは継続します。停止ボタンで保存を確定します。', wraplength=670).pack(anchor='w', side='bottom')
    context = ssl.create_default_context(cafile=str(Path('~/.config/ai_worker/human-demo/cert.pem').expanduser()))
    def request(data=None):
        state = json.loads(Path('~/.local/state/ai_worker/human-demo-server.json').expanduser().read_text())
        req = Request(f'https://localhost:{state["port"]}'+('/control' if data else '/status'), data=json.dumps(data).encode() if data else None,
            headers={'Authorization':'Bearer '+state['token'], 'Content-Type':'application/json'})
        try:
            with urlopen(req,context=context,timeout=1.) as response: return json.load(response)
        except HTTPError as e: raise ValueError(e.read().decode()) from e
    def display(s):
        recording = s['recording']; ages = s['ages_s']
        ready = all(ages.get(k,1e9)<.5 for k in ['xr','glove_left','glove_right'])
        status.set(('● 記録中' if recording else '■ 停止中')+f'   VR {s.get("xr_rate_hz",0):.0f} Hz / '+('入力あり' if ready else '入力待ち'))
        path.set('保存先: '+str(s.get('directory') or '未開始'))
        start.configure(state='disabled' if recording else 'normal'); stop.configure(state='normal' if recording else 'disabled')
    def action(name):
        try:
            if name=='start' and not task.get().strip(): raise ValueError('タスク名を入力してください')
            result=request(dict(action=name,task=task.get().strip(),outcome=outcome.get())); display(result['status']); error.set('')
        except Exception as e: error.set(str(e))
    def poll():
        try: display(request())
        except Exception as e:
            status.set('サーバに接続できません'); error.set(str(e)); start.configure(state='disabled'); stop.configure(state='disabled')
        root.after(1000,poll)
    poll(); root.mainloop()

if __name__ == '__main__': main()
