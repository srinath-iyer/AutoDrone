"""This file is reponsible for the automation involved in publishing OTA updates to the ESP32 firmware."""

import subprocess
import os
import http.server
import ssl
import threading
import time
import socket

FIRMWARE_PATH = "AutoDrone/src/onboard/esp32/.pio/build/esp32dev/firmware.bin"
CERT_PATH = "cert.pem"
KEY_PATH = "key.pem"
PORT = 8080

def build_firmware():
    print("[*] Building firmware...")

    build_dir = "src/onboard/esp32"
    subprocess.run(["pio", "run"], cwd=build_dir, check=True)

def get_ip():
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    return local_ip

def start_https_server():
    print("[*] Starting HTTPS server on port", PORT)
    os.chdir("build")
    
    handler = http.server.SimpleHTTPRequestHandler
    httpd = http.server.HTTPServer(("0.0.0.0", PORT), handler)
    httpd.socket = ssl.wrap_socket(httpd.socket,
                                   certfile=CERT_PATH,
                                   keyfile=KEY_PATH,
                                   server_side=True)
    httpd.serve_forever()

def main():
    build_firmware()
    local_ip = get_ip()
    
    # Start HTTPS server in a thread so your terminal isn't blocked
    thread = threading.Thread(target=start_https_server, daemon=True)
    thread.start()
    
    print(f"[*] HTTPS server hosting {FIRMWARE_PATH} at https://{local_ip}:{PORT}/")
    print("[*] Trigger OTA update from ESP32 now.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[*] Shutting down.")

if __name__ == "__main__":
    main()
