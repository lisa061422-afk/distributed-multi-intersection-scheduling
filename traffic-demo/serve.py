"""
Quick dev server — run this, then open the demo in a browser.
Usage: python serve.py
Then open http://localhost:8080/warehouse_amr_demo_test.html
"""
import http.server
import socketserver
import os

PORT = 8080
os.chdir(os.path.dirname(os.path.abspath(__file__)))

Handler = http.server.SimpleHTTPRequestHandler
Handler.extensions_map.update({'.html': 'text/html', '.png': 'image/png'})

print(f"Serving at http://localhost:{PORT}/warehouse_amr_demo_test.html")
print("Press Ctrl+C to stop.")
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    httpd.serve_forever()
