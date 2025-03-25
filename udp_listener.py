import socket
import sys
import time
import platform
import subprocess

def get_network_info():
    try:
        # Run ipconfig and parse the output
        result = subprocess.run(['ipconfig'], capture_output=True, text=True)
        print("\nNetwork Interfaces:")
        print(result.stdout)
        
        # Get the WiFi IP address
        wifi_ip = None
        lines = result.stdout.split('\n')
        for i, line in enumerate(lines):
            if "Wireless LAN adapter Wi-Fi" in line:
                # Look for IPv4 Address in the next few lines
                for j in range(i, min(i + 10, len(lines))):
                    if "IPv4 Address" in lines[j]:
                        wifi_ip = lines[j].split(": ")[-1].strip()
                        break
                break
        
        if wifi_ip:
            print(f"\nFound WiFi IP: {wifi_ip}")
        else:
            print("\nWarning: Could not find WiFi IP address")
        
        return wifi_ip
    except Exception as e:
        print(f"Error getting network info: {e}")
        return None

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 1234
BUFFER_SIZE = 1024

def create_socket():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)  # Increase receive buffer
        sock.bind((UDP_IP, UDP_PORT))
        
        # Get network information
        wifi_ip = get_network_info()
        
        print(f"\nSocket Details:")
        print(f"Listening on all interfaces (0.0.0.0)")
        print(f"Port: {UDP_PORT}")
        print(f"WiFi IP: {wifi_ip}")
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

            print(f"\nWaiting for data on all interfaces (0.0.0.0):{UDP_PORT}...")
            data, addr = sock.recvfrom(BUFFER_SIZE)
            current_time = time.time()
            
            # Calculate message rate
            if current_time - last_message_time >= 1.0:
                print(f"Message rate: {message_count} messages/second")
                message_count = 0
                last_message_time = current_time
            
            message_count += 1
            print(f"Received from {addr[0]}:{addr[1]} on interface {addr[0]}")
            print(f"Data: {data.decode('utf-8', errors='ignore')}")

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