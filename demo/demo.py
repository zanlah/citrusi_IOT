import serial
import time
import socket

class ESP32Server:
    def __init__(self, port='COM8', baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )
        
    def send_command(self, cmd, wait_time=1):
        """Send AT command and return response"""
        cmd = cmd + '\r\n'
        self.ser.write(cmd.encode())
        time.sleep(wait_time)
        
        response = ''
        while self.ser.in_waiting:
            response += self.ser.read().decode()
            
        print(f"Sent: {cmd.strip()}")
        print(f"Received: {response}")
        return response
    
    def setup_ap_server(self):
        """Setup ESP32 as Access Point with Web Server"""
        # Reset module
        self.send_command("AT+RST")
        time.sleep(2)
        
        # Set mode to AP
        self.send_command("AT+CWMODE=2")
        
        # Configure AP settings
        self.send_command('AT+CWSAP="ESP32_Config","12345678",1,3')
        
        # Enable multiple connections
        self.send_command("AT+CIPMUX=1")
        
        # Start TCP server
        self.send_command("AT+CIPSERVER=1,80")
        
        # Get IP address
        self.send_command("AT+CIFSR")
        
    def send_webpage(self, connection_id):
        """Send HTML configuration page"""
        html = """<html><head><title>ESP32 WiFi Config</title></head><body><h1>ESP32 WiFi Configuration</h1><form method='get' action='setap'>SSID: <input type='text' name='ssid'><br><br>Password: <input type='text' name='pass'><br><br><input type='submit' value='Connect'></form></body></html>"""
        
        # Send HTTP response
        response = f"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {len(html)}\r\nConnection: close\r\n\r\n{html}"
        
        # Send in one go
        self.send_command(f"AT+CIPSEND={connection_id},{len(response)}")
        time.sleep(0.5)  # Wait for '>' prompt
        self.ser.write(response.encode())
        time.sleep(1)
        
        # Close this connection
        self.send_command(f"AT+CIPCLOSE={connection_id}")

    def connect_to_wifi(self, ssid, password):
        """Connect to WiFi and verify connection"""
        # Switch to AP+Station mode
        self.send_command("AT+CWMODE=3")
        time.sleep(1)
        
        # Connect to WiFi
        self.send_command(f'AT+CWJAP="{ssid}","{password}"')
        time.sleep(5)  # Give it time to connect
        
        # Check connection status and get IP
        response = self.send_command("AT+CIFSR")
        if "STAIP" in response:
            # Get the IP address
            ip_start = response.find("STAIP,\"") + 7
            ip_end = response.find("\"", ip_start)
            new_ip = response[ip_start:ip_end]
            
            # Send success response with IP
            success_msg = f"Connected! New IP: {new_ip}"
            self.send_command(f"AT+CIPSEND=0,{len(success_msg)}")
            time.sleep(0.5)
            self.send_command(success_msg)
        else:
            # Send error response
            error_msg = "Failed to connect to WiFi"
            self.send_command(f"AT+CIPSEND=0,{len(error_msg)}")
            time.sleep(0.5)
            self.send_command(error_msg)

    def handle_client_request(self, data):
        """Handle incoming HTTP requests"""
        if '+IPD' in data:
            # Extract connection ID
            start_idx = data.find('+IPD,') + 5
            end_idx = data.find(',', start_idx)
            conn_id = data[start_idx:end_idx]
            
            if 'GET /setap' in data:
                # Parse SSID and password from GET request
                try:
                    ssid_start = data.find('ssid=') + 5
                    ssid_end = data.find('&', ssid_start)
                    ssid = data[ssid_start:ssid_end]
                    
                    pass_start = data.find('pass=') + 5
                    pass_end = data.find(' ', pass_start)
                    password = data[pass_start:pass_end]
                    
                    self.connect_to_wifi(ssid, password)
                    

                    
                except Exception as e:
                    print(f"Error parsing request: {e}")
            
            else:
                # Send configuration webpage
                self.send_webpage(conn_id)
    
    def run(self):
        """Main loop to handle connections"""
        print("Starting ESP32 server...")
        self.setup_ap_server()
        
        print("\nServer running! Connect to ESP32_Config WiFi network")
        print("Then visit 192.168.4.1 in your web browser")
        
        try:
            while True:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode()
                    self.handle_client_request(data)
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nClosing server...")
        
        finally:
            self.ser.close()

if __name__ == "__main__":
    # Create and run server (adjust COM port as needed)
    server = ESP32Server(port='COM8')
    server.run()