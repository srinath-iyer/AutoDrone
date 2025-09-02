from cv_pipeline import CVPipeline
from dashboard_server import start_server
import threading
import time

pipeline = CVPipeline()
pipeline.start()

server_thread = threading.Thread(target=start_server, args=(pipeline,))
server_thread.daemon = True
server_thread.start()

while True:
    time.sleep(1)
