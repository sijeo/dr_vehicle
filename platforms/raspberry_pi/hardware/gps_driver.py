"""
GPS driver for Ublox NEO6M using UART/Serial.
"""

import time
import threading
from typing import Optional, List
from queue import Queue, Empty

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not available, GPS driver will use simulation mode")

class GPSDriver:
    """
    Driver for Ublox NEO6M GPS module via UART.
    """
    
    def __init__(self, serial_port: str = "/dev/ttyAMA0", baud_rate: int = 9600):
        """
        Initialize GPS driver.
        
        Args:
            serial_port: Serial port device (e.g., "/dev/ttyAMA0")
            baud_rate: Baud rate (default 9600 for NEO6M)
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.initialized = False
        
        # Threading for continuous reading
        self.reader_thread = None
        self.running = False
        
        # Data buffer
        self.sentence_queue = Queue(maxsize=100)
        self.receive_buffer = ""
        
        # Simulation mode
        self.simulation_mode = not SERIAL_AVAILABLE
        self.sim_time_start = time.time()
        self.sim_lat_base = 37.7749  # San Francisco
        self.sim_lon_base = -122.4194
        
        # Statistics
        self.sentences_received = 0
        self.sentences_parsed = 0
        self.last_sentence_time = 0
    
    def initialize(self) -> bool:
        """
        Initialize the GPS connection.
        
        Returns:
            True if initialization successful
        """
        if self.simulation_mode:
            print("GPS: Running in simulation mode")
            self.initialized = True
            self._start_simulation()
            return True
        
        try:
            # Open serial connection
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            
            # Clear any existing data
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            
            # Start reader thread
            self.running = True
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            
            self.initialized = True
            print(f"GPS: Initialized on {self.serial_port} at {self.baud_rate} baud")
            
            # Wait a bit for first data
            time.sleep(2.0)
            
            return True
            
        except Exception as e:
            print(f"GPS: Initialization failed: {e}")
            return False
    
    def _start_simulation(self):
        """Start simulation mode."""
        self.running = True
        self.reader_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self.reader_thread.start()
    
    def _reader_loop(self):
        """Main reader loop for real GPS data."""
        while self.running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    # Read available data
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    
                    # Decode to string
                    text = data.decode('ascii', errors='ignore')
                    self.receive_buffer += text
                    
                    # Extract complete sentences
                    self._extract_sentences()
                
                else:
                    time.sleep(0.01)  # Small delay when no data
                    
            except Exception as e:
                print(f"GPS: Reader error: {e}")
                time.sleep(0.1)
    
    def _simulation_loop(self):
        """Simulation loop generating fake NMEA sentences."""
        sequence = 0
        
        while self.running:
            current_time = time.time() - self.sim_time_start
            
            # Simulate slow movement in a circle
            radius = 0.001  # About 100m radius in degrees
            angular_velocity = 0.1  # rad/s
            
            lat = self.sim_lat_base + radius * 0.5 * np.sin(angular_velocity * current_time)
            lon = self.sim_lon_base + radius * np.cos(angular_velocity * current_time)
            
            # Generate time stamp
            utc_time = time.gmtime()
            time_str = f"{utc_time.tm_hour:02d}{utc_time.tm_min:02d}{utc_time.tm_sec:02d}.00"
            date_str = f"{utc_time.tm_mday:02d}{utc_time.tm_mon:02d}{utc_time.tm_year-2000:02d}"
            
            # Convert lat/lon to NMEA format
            lat_deg = int(abs(lat))
            lat_min = (abs(lat) - lat_deg) * 60
            lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
            lat_dir = "N" if lat >= 0 else "S"
            
            lon_deg = int(abs(lon))
            lon_min = (abs(lon) - lon_deg) * 60
            lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
            lon_dir = "E" if lon >= 0 else "W"
            
            # Generate GGA sentence
            gga_data = f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},1,08,1.0,545.4,M,46.9,M,,"
            gga_checksum = self._calculate_checksum(gga_data)
            gga_sentence = f"${gga_data}*{gga_checksum:02X}\r\n"
            
            # Generate RMC sentence
            speed_knots = 2.0 + np.sin(current_time * 0.2)  # Varying speed
            course = (angular_velocity * current_time * 180 / np.pi) % 360  # Course in degrees
            
            rmc_data = f"GPRMC,{time_str},A,{lat_str},{lat_dir},{lon_str},{lon_dir},{speed_knots:.1f},{course:.1f},{date_str},,,"
            rmc_checksum = self._calculate_checksum(rmc_data)
            rmc_sentence = f"${rmc_data}*{rmc_checksum:02X}\r\n"
            
            # Add sentences to queue
            try:
                self.sentence_queue.put_nowait(gga_sentence.strip())
                self.sentence_queue.put_nowait(rmc_sentence.strip())
                self.sentences_received += 2
            except:
                pass  # Queue full
            
            # GPS typically updates at 1 Hz
            time.sleep(1.0)
    
    def _extract_sentences(self):
        """Extract complete NMEA sentences from receive buffer."""
        while '\n' in self.receive_buffer:
            line_end = self.receive_buffer.find('\n')
            sentence = self.receive_buffer[:line_end].strip()
            self.receive_buffer = self.receive_buffer[line_end + 1:]
            
            if sentence and sentence.startswith('$'):
                try:
                    self.sentence_queue.put_nowait(sentence)
                    self.sentences_received += 1
                    self.last_sentence_time = time.time()
                except:
                    pass  # Queue full
    
    def _calculate_checksum(self, sentence: str) -> int:
        """Calculate NMEA checksum."""
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return checksum
    
    def read_sentence(self, timeout: float = 1.0) -> Optional[str]:
        """
        Read next NMEA sentence.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            NMEA sentence string, or None if timeout/error
        """
        if not self.initialized:
            return None
        
        try:
            sentence = self.sentence_queue.get(timeout=timeout)
            self.sentences_parsed += 1
            return sentence
            
        except Empty:
            return None
        except Exception as e:
            print(f"GPS: Read error: {e}")
            return None
    
    def read_sentences(self, max_sentences: int = 10) -> List[str]:
        """
        Read multiple NMEA sentences.
        
        Args:
            max_sentences: Maximum number of sentences to read
            
        Returns:
            List of NMEA sentence strings
        """
        sentences = []
        
        for _ in range(max_sentences):
            sentence = self.read_sentence(timeout=0.1)
            if sentence:
                sentences.append(sentence)
            else:
                break
        
        return sentences
    
    def flush_buffer(self):
        """Clear all buffered sentences."""
        while not self.sentence_queue.empty():
            try:
                self.sentence_queue.get_nowait()
            except Empty:
                break
    
    def is_data_available(self) -> bool:
        """Check if data is available to read."""
        return not self.sentence_queue.empty()
    
    def get_data_age(self) -> float:
        """
        Get age of last received data.
        
        Returns:
            Age in seconds since last sentence received
        """
        if self.last_sentence_time == 0:
            return float('inf')
        
        return time.time() - self.last_sentence_time
    
    def send_command(self, command: str) -> bool:
        """
        Send UBX command to GPS (for configuration).
        
        Args:
            command: UBX command string
            
        Returns:
            True if sent successfully
        """
        if not self.initialized or self.simulation_mode:
            return False
        
        try:
            self.serial_conn.write(command.encode('ascii'))
            self.serial_conn.flush()
            return True
            
        except Exception as e:
            print(f"GPS: Command send error: {e}")
            return False
    
    def get_statistics(self) -> dict:
        """Get driver statistics."""
        return {
            'initialized': self.initialized,
            'simulation_mode': self.simulation_mode,
            'sentences_received': self.sentences_received,
            'sentences_parsed': self.sentences_parsed,
            'queue_size': self.sentence_queue.qsize(),
            'data_age': self.get_data_age(),
            'serial_port': self.serial_port,
            'baud_rate': self.baud_rate
        }
    
    def cleanup(self):
        """Cleanup resources."""
        self.running = False
        
        # Wait for reader thread to finish
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=2.0)
        
        # Close serial connection
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
            self.serial_conn = None
        
        # Clear queue
        self.flush_buffer()
        
        self.initialized = False
        print("GPS: Cleanup completed")

# Import numpy for simulation if available
try:
    import numpy as np
except ImportError:
    # Simple fallback for sine function if numpy not available
    import math
    class np:
        @staticmethod
        def sin(x):
            return math.sin(x)
        
        @staticmethod
        def cos(x):
            return math.cos(x)