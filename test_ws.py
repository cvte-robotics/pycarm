import websocket
import threading

def on_message(ws, message):
    print(f"后台接收: {message}")

ws = websocket.WebSocketApp(
    "wss://echo.websocket.events",
    on_message=on_message
)

# 启动后台线程运行WebSocket
threading.Thread(target=ws.run_forever, daemon=True).start()

# 主线程发送消息
while True:
    msg = input("输入消息（输入 'exit' 退出）: ")
    if msg == "exit":
        break
    ws.send(msg)

ws.close()