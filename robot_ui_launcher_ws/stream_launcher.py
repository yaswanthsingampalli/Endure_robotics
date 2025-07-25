#stream_launcher.py
#!/usr/bin/env python3
"""
Simple MJPEG streaming server with direct V4L2 access
"""
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
import signal
import sys
import os

class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            
            try:
                # Try different camera backends and settings
                camera = None
                
                # Method 1: Direct V4L2 access
                try:
                    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    print("Using CAP_V4L2 backend")
                except:
                    pass
                
                # Method 2: Fallback to default
                if camera is None or not camera.isOpened():
                    camera = cv2.VideoCapture(0)
                    print("Using default backend")
                
                # Set camera properties
                if camera.isOpened():
                    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    camera.set(cv2.CAP_PROP_FPS, 30)
                    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to minimize latency
                    print("Camera initialized successfully")
                else:
                    print("ERROR: Could not open camera")
                    self.send_error(500, "Could not open camera")
                    return
                
                frame_count = 0
                while True:
                    success, frame = camera.read()
                    if not success:
                        print(f"Failed to read frame {frame_count}")
                        # Try to reinitialize camera
                        camera.release()
                        camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
                        if not camera.isOpened():
                            camera = cv2.VideoCapture(0)
                        continue
                    
                    frame_count += 1
                    
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    if ret:
                        try:
                            self.wfile.write(b'--FRAME\r\n')
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(buffer)))
                            self.end_headers()
                            self.wfile.write(buffer.tobytes())
                            self.wfile.write(b'\r\n')
                        except:
                            # Client disconnected
                            break
                            
            except Exception as e:
                print(f"Client disconnected or error occurred: {e}")
            finally:
                if 'camera' in locals() and camera:
                    camera.release()
        else:
            self.send_error(404)
            self.end_headers()

def main():
    try:
        # Test camera access first
        print("Testing camera access...")
        test_camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not test_camera.isOpened():
            test_camera = cv2.VideoCapture(0)
        
        if test_camera.isOpened():
            print("✅ Camera is accessible")
            ret, frame = test_camera.read()
            if ret:
                print(f"✅ Camera read successful: {frame.shape}")
            test_camera.release()
        else:
            print("❌ ERROR: Camera is not accessible!")
            print("Check if another process is using the camera:")
            os.system("lsof /dev/video0 2>/dev/null || echo 'No processes found using /dev/video0'")
            return 1
        
        server = HTTPServer(('', 9999), VideoStreamHandler)
        print("✅ Camera streaming server started on port 9999")
        print("Access stream at: http://localhost:9999/video_feed")
        server.serve_forever()
        
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.socket.close()
        sys.exit(0)
    except Exception as e:
        print(f"Server error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    sys.exit(main())