import socket
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

telemetry_data = {}

def udp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 14554))
    while True:
        data, _ = sock.recvfrom(4096)
        try:
            msg = json.loads(data.decode("utf-8"))
            for k, v in msg.items():
                telemetry_data[k] = v
        except:
            pass

class TelemetryHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(telemetry_data).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html><head><title>Internal Telemetry</title></head>
            <body>
                <h1>Drone Telemetry</h1>
                <pre id="data"></pre>
                <script>
                    setInterval(() => {
                        fetch('/api/telemetry').then(r => r.json()).then(d => {
                            document.getElementById('data').textContent = JSON.stringify(d, null, 2);
                        });
                    }, 500);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())

def http_server():
    server = HTTPServer(('0.0.0.0', 5001), TelemetryHandler)
    server.serve_forever()

if __name__ == '__main__':
    t = threading.Thread(target=udp_server, daemon=True)
    t.start()
    http_server()
