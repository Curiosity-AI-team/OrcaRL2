import socket

def get_local_ip():
    try:
        # Try to find an IPv4 address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except Exception as e:
        print(f"Error getting local IP: {e}")
        return None

# Example usage
local_ip = get_local_ip()
print("Local IP Address:", local_ip)