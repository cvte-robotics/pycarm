import tkinter as tk
from tkinter import ttk, messagebox
import threading
import socket
import json
from zeroconf import Zeroconf, ServiceBrowser, ServiceStateChange

SERVICE_TYPE = "_carm._tcp.local."

class CarmScanApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CARM Device Scanner")
        self.root.geometry("600x400")

        self.zeroconf = None
        self.browser = None
        self.devices = {}  # {name: info}

        self.setup_ui()

    def setup_ui(self):
        # Scan Button
        top_frame = tk.Frame(self.root)
        top_frame.pack(fill=tk.X, pady=10, padx=10)
        
        self.scan_btn = tk.Button(top_frame, text="Scan Devices", command=self.start_scan)
        self.scan_btn.pack(side=tk.LEFT)

        # Device List
        list_frame = tk.Frame(self.root)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10)

        columns = ("name", "server", "ip", "port")
        self.tree = ttk.Treeview(list_frame, columns=columns, show="headings")
        self.tree.heading("name", text="Device Name")
        self.tree.heading("server", text="Server Host")
        self.tree.heading("ip", text="IP Address")
        self.tree.heading("port", text="Port")
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<<TreeviewSelect>>", self.on_select)

        # Rename Section
        btm_frame = tk.Frame(self.root)
        btm_frame.pack(fill=tk.X, pady=10, padx=10)

        tk.Label(btm_frame, text="New Name:").pack(side=tk.LEFT)
        self.name_entry = tk.Entry(btm_frame, width=20)
        self.name_entry.pack(side=tk.LEFT, padx=5)
        
        self.rename_btn = tk.Button(btm_frame, text="Rename Selected", command=self.rename_device, state=tk.DISABLED)
        self.rename_btn.pack(side=tk.LEFT)

    def start_scan(self):
        """启动 mDNS 浏览"""
        self.devices.clear()
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        if self.zeroconf is not None:
            self.zeroconf.close()
            
        self.zeroconf = Zeroconf()
        self.browser = ServiceBrowser(self.zeroconf, SERVICE_TYPE, handlers=[self.on_service_state_change])
        print(f"[mDNS] 开始监听 {SERVICE_TYPE} ...")

    def on_service_state_change(self, zeroconf, service_type, name, state_change):
        print(f"[mDNS] event={state_change} name={name}")
        if state_change is ServiceStateChange.Added:
            info = zeroconf.get_service_info(service_type, name)
            if info:
                print(f"[mDNS] raw info: {info}")
                print(f"[mDNS] addresses (raw): {info.addresses}")
                print(f"[mDNS] server: {info.server}, port: {info.port}")
                addresses = [socket.inet_ntoa(addr) for addr in info.addresses]
                print(f"[mDNS] parsed IPs: {addresses}")
                ip = ", ".join(addresses) if addresses else "Unknown"

                device_info = {
                    "name": name.replace(f".{SERVICE_TYPE}", ""),
                    "full_name": name,
                    "server": info.server,
                    "ip": ip,
                    "ips": addresses,
                    "port": info.port
                }
                self.devices[name] = device_info
                self.root.after(0, self.update_list)

        elif state_change is ServiceStateChange.Removed:
            if name in self.devices:
                del self.devices[name]
                self.root.after(0, self.update_list)

    def update_list(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for name, info in self.devices.items():
            self.tree.insert("", "end", iid=name,
                             values=(info["name"], info["server"], info["ip"], info["port"]))

    def on_select(self, event):
        selected = self.tree.selection()
        if selected:
            self.rename_btn.config(state=tk.NORMAL)
            item = self.devices[selected[0]]
            self.name_entry.delete(0, tk.END)
            self.name_entry.insert(0, item["name"])
        else:
            self.rename_btn.config(state=tk.DISABLED)

    def rename_device(self):
        selected = self.tree.selection()
        if not selected:
            return
            
        device = self.devices[selected[0]]
        new_name = self.name_entry.get().strip()
        if not new_name:
            messagebox.showerror("Error", "Name cannot be empty")
            return

        port = device["port"]
        ips = device.get("ips", [])

        def do_rename():
            for ip in ips:
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.settimeout(3.0)
                        s.connect((ip, port))
                        req = json.dumps({"action": "rename", "new_name": new_name})
                        s.sendall(req.encode('utf-8'))
                        resp = s.recv(1024).decode('utf-8')
                        resp_data = json.loads(resp)
                        if resp_data.get("status") == "success":
                            self.root.after(0, lambda: messagebox.showinfo("Success", f"Renamed to {new_name} successfully. Please rescan."))
                            return
                        else:
                            self.root.after(0, lambda m=resp_data.get("message", "Unknown error"): messagebox.showerror("Error", m))
                            return
                except Exception:
                    continue
            self.root.after(0, lambda: messagebox.showerror("Error", "Failed to rename: no reachable IP"))

        threading.Thread(target=do_rename, daemon=True).start()

    def on_close(self):
        if self.zeroconf is not None:
            self.zeroconf.close()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = CarmScanApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
