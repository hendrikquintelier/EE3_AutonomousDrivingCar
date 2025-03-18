import socket
import sys
import time
import platform

UDP_IP = "0.0.0.0"
UDP_PORT = 1234
BUFFER_SIZE = 1024

def get_local_ip():
    try:
        # Get the hostname
        hostname = socket.gethostname()
        # Get the IP address
        ip_address = socket.gethostbyname(hostname)
        return ip_address
    except Exception as e:
        print(f"Error getting local IP: {e}")
        return None

def create_socket():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)  # Increase receive buffer
        sock.bind((UDP_IP, UDP_PORT))
        print(f"Listening for UDP packets on port {UDP_PORT}...")
        print(f"Local IP address: {get_local_ip()}")
        print(f"System: {platform.system()} {platform.release()}")
        return sock
    except Exception as e:
        print(f"Error setting up socket: {e}")
        return None

def main():
    sock = None
    last_message_time = time.time()
    message_count = 0
    
    while True:
        try:
            if sock is None:
                sock = create_socket()
                if sock is None:
                    print("Failed to create socket, retrying in 5 seconds...")
                    time.sleep(5)
                    continue

            print(f"Waiting for data on {UDP_IP}:{UDP_PORT}...")
            data, addr = sock.recvfrom(BUFFER_SIZE)
            current_time = time.time()
            
            # Calculate message rate
            if current_time - last_message_time >= 1.0:
                print(f"Message rate: {message_count} messages/second")
                message_count = 0
                last_message_time = current_time
            
            message_count += 1
            print(f"Received from {addr}: {data.decode('utf-8', errors='ignore')}")

        except KeyboardInterrupt:
            print("\nStopping server...")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            if sock:
                try:
                    sock.close()
                except:
                    pass
                sock = None
            print("Attempting to recreate socket...")
            time.sleep(1)
            continue

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        if sock:
            try:
                sock.close()
            except:
                pass
        print("Server stopped") 