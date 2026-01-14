#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OV7725 USB Camera Viewer with GUI
Receive RGB565 image data from STM32 via USB CDC and display

Dependencies:
    pip install pyserial numpy opencv-python pillow
"""

import sys
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import cv2
from PIL import Image, ImageTk
import serial
import serial.tools.list_ports
import struct

# Frame header definition (must match STM32 settings)
FRAME_HEADER_SYNC1 = 0xAA
FRAME_HEADER_SYNC2 = 0x55
FRAME_HEADER_END1 = 0xA5
FRAME_HEADER_END2 = 0x5A
FRAME_HEADER_SIZE = 10  # 同步字(2B) + 帧号(2B) + 总包数(2B) + 每包大小(2B) + 结束字(2B)

# Packet header definition
PACKET_HEADER_SIZE = 4

# Default image parameters
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
BYTES_PER_PIXEL = 2  # RGB565


class OV7725ViewerGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("OV7725 USB Camera Viewer")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        self.serial = None
        self.running = False
        self.image_width = DEFAULT_WIDTH
        self.image_height = DEFAULT_HEIGHT
        self.frame_size = self.image_width * self.image_height * BYTES_PER_PIXEL
        self.frame_buffer = bytearray(self.frame_size)
        self.frame_count = 0
        self.stm32_frame_idx = 0
        self.fps = 0
        self.last_fps_time = time.time()
        self.fps_count = 0
        self.receive_thread = None
        self.sync_errors = 0
        
        self.setup_ui()
        self.refresh_ports()
        
    def setup_ui(self):
        """Setup the GUI components"""
        # Top frame for controls
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Port selection
        ttk.Label(control_frame, text="Serial Port:").pack(side=tk.LEFT, padx=5)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        # Refresh button
        self.refresh_btn = ttk.Button(control_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=5)
        
        # Connect button
        self.connect_btn = ttk.Button(control_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        # Save button
        self.save_btn = ttk.Button(control_frame, text="Save Frame", command=self.save_frame, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=5)
        
        # Status frame
        status_frame = ttk.Frame(self.root, padding="5")
        status_frame.pack(fill=tk.X)
        
        self.status_label = ttk.Label(status_frame, text="Status: Disconnected")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.fps_label = ttk.Label(status_frame, text="FPS: 0")
        self.fps_label.pack(side=tk.RIGHT, padx=5)
        
        self.frame_label = ttk.Label(status_frame, text="GUI Frame: 0 (STM32 Frame: 0)")
        self.frame_label.pack(side=tk.RIGHT, padx=5)
        
        self.sync_label = ttk.Label(status_frame, text="Sync Errors: 0")
        self.sync_label.pack(side=tk.RIGHT, padx=5)
        
        self.resolution_label = ttk.Label(status_frame, text=f"Resolution: {DEFAULT_WIDTH}x{DEFAULT_HEIGHT}")
        self.resolution_label.pack(side=tk.RIGHT, padx=5)
        
        # Image display frame
        image_frame = ttk.Frame(self.root, padding="10")
        image_frame.pack(fill=tk.BOTH, expand=True)
        
        # Canvas for image display
        self.canvas = tk.Canvas(image_frame, bg="black", width=DEFAULT_WIDTH*2, height=DEFAULT_HEIGHT*2)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Create placeholder image
        self.photo = None
        self.current_image = None
        self.display_placeholder()
        
        # Bind window close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def display_placeholder(self):
        """Display a placeholder image"""
        placeholder = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        cv2.putText(placeholder, "No Image", (80, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
        self.update_image(placeholder)
        
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)
        else:
            self.port_var.set("No ports found")
            
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.running:
            self.disconnect()
        else:
            self.connect()
            
    def connect(self):
        """Connect to selected serial port"""
        port_str = self.port_var.get()
        if not port_str or "No ports" in port_str:
            messagebox.showerror("Error", "Please select a valid serial port")
            return
            
        # Extract port name (e.g., "COM3" from "COM3 - USB Serial Device")
        port = port_str.split(" - ")[0]
        
        print(f"[CONNECT] Attempting to connect to {port}...")
        
        # 更新UI状态
        self.status_label.config(text=f"Status: Connecting to {port}...")
        self.connect_btn.config(state=tk.DISABLED)
        self.root.update()  # 强制更新UI
        
        try:
            # USB CDC虚拟串口不需要特定波特率，但pyserial需要设置
            # 使用较短的超时时间避免阻塞GUI
            print(f"[CONNECT] Opening serial port...")
            self.serial = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=0.05,  # 50ms超时，更短以避免阻塞
                write_timeout=0.5,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False
            )
            print(f"[CONNECT] Serial port opened successfully!")
            print(f"[CONNECT] Port: {self.serial.port}, Baudrate: {self.serial.baudrate}")
            print(f"[CONNECT] Is open: {self.serial.is_open}")
            
            # 清空缓冲区（忽略错误）
            try:
                self.serial.reset_input_buffer()
                print("[CONNECT] Input buffer cleared")
            except Exception as e:
                print(f"[CONNECT] Failed to clear input buffer: {e}")
            try:
                self.serial.reset_output_buffer()
                print("[CONNECT] Output buffer cleared")
            except Exception as e:
                print(f"[CONNECT] Failed to clear output buffer: {e}")
                
            self.running = True
            self.sync_errors = 0
            self.connect_btn.config(text="Disconnect", state=tk.NORMAL)
            self.save_btn.config(state=tk.NORMAL)
            self.port_combo.config(state=tk.DISABLED)
            self.refresh_btn.config(state=tk.DISABLED)
            self.status_label.config(text=f"Status: Connected to {port}")
            
            # Start receive thread
            print("[CONNECT] Starting receive thread...")
            self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.receive_thread.start()
            print("[CONNECT] Receive thread started!")
            
        except serial.SerialException as e:
            print(f"[CONNECT] Serial exception: {e}")
            self.connect_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Status: Connection failed")
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
        except Exception as e:
            print(f"[CONNECT] Unexpected error: {e}")
            self.connect_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Status: Connection failed")
            messagebox.showerror("Connection Error", f"Unexpected error: {e}")
            
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None
        
        self.connect_btn.config(text="Connect")
        self.save_btn.config(state=tk.DISABLED)
        self.port_combo.config(state="readonly")
        self.refresh_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Status: Disconnected")
        
    def rgb565_to_rgb888(self, data):
        """Convert RGB565 data to RGB888 image
        Note: STM32 sends data in Big-Endian (high byte first, low byte second)
        """
        # Use big-endian format '>u2' to match STM32 byte order
        rgb565 = np.frombuffer(data, dtype='>u2')
        rgb565 = rgb565.reshape((self.image_height, self.image_width))
        
        # RGB565 format: RRRRR GGGGGG BBBBB (5-6-5 bits)
        r = ((rgb565 >> 11) & 0x1F) << 3  # 5-bit red -> 8-bit
        g = ((rgb565 >> 5) & 0x3F) << 2   # 6-bit green -> 8-bit
        b = (rgb565 & 0x1F) << 3          # 5-bit blue -> 8-bit
        
        rgb888 = np.stack([r, g, b], axis=-1).astype(np.uint8)
        return rgb888
    
    def find_frame_header(self):
        """Search for frame header in serial stream
        Header format (10 bytes): 
            [0xAA, 0x55, frame_idx_L, frame_idx_H, 
             total_packets_L, total_packets_H, packet_size_L, packet_size_H, 0xA5, 0x5A]
        Returns: (frame_idx, total_packets, packet_size) or None if not found
        """
        max_bytes_to_search = 4096  # 最多搜索4KB数据
        bytes_searched = 0
        no_data_count = 0
        max_no_data = 100  # 最多等待100次（约5秒）
        raw_bytes = []  # 用于调试打印
        
        while self.running and bytes_searched < max_bytes_to_search:
            # 尝试读取一个字节（使用超时）
            try:
                byte1 = self.serial.read(1)
            except Exception as e:
                time.sleep(0.01)
                continue
                
            if len(byte1) == 0:
                no_data_count += 1
                if no_data_count >= max_no_data:
                    if raw_bytes:
                        print(f"[RX] Received {len(raw_bytes)} bytes before timeout: {' '.join(f'{b:02X}' for b in raw_bytes[:50])}...")
                    return None  # 超时，没有数据
                continue
            
            no_data_count = 0  # 重置计数器
            bytes_searched += 1
            raw_bytes.append(byte1[0])
            
            # 每收到100字节打印一次
            if len(raw_bytes) % 100 == 0:
                print(f"[RX] Received {len(raw_bytes)} bytes so far...")
                
            if byte1[0] == FRAME_HEADER_SYNC1:
                byte2 = self.serial.read(1)
                if len(byte2) == 0:
                    continue
                bytes_searched += 1
                raw_bytes.append(byte2[0])
                    
                if byte2[0] == FRAME_HEADER_SYNC2:
                    print(f"[RX] Found sync bytes 0xAA 0x55!")
                    # Found sync, read rest of header
                    header_rest = self.serial.read(FRAME_HEADER_SIZE - 2)
                    if len(header_rest) == FRAME_HEADER_SIZE - 2:
                        bytes_searched += len(header_rest)
                        raw_bytes.extend(header_rest)
                        print(f"[RX] Header: {' '.join(f'{b:02X}' for b in raw_bytes[-10:])}")
                        # Verify end markers (at positions 6 and 7)
                        if header_rest[6] == FRAME_HEADER_END1 and header_rest[7] == FRAME_HEADER_END2:
                            # Parse header
                            frame_idx = struct.unpack('<H', header_rest[0:2])[0]
                            total_packets = struct.unpack('<H', header_rest[2:4])[0]
                            packet_size = struct.unpack('<H', header_rest[4:6])[0]
                            print(f"[RX] Valid header: frame={frame_idx}, packets={total_packets}, size={packet_size}")
                            return (frame_idx, total_packets, packet_size)
                        else:
                            print(f"[RX] Invalid end markers: {header_rest[6]:02X} {header_rest[7]:02X}")
        
        if raw_bytes:
            print(f"[RX] Searched {bytes_searched} bytes, first 50: {' '.join(f'{b:02X}' for b in raw_bytes[:50])}")
        return None
    
    def receive_data_packet(self, expected_idx, expected_size):
        """Receive a single data packet with header validation
        STM32发送格式：先发送包头(4字节)，再发送数据（分两次CDC_Transmit_FS调用）
        Packet header: [packet_idx_L, packet_idx_H, data_len_L, data_len_H]
        Then data follows separately
        Returns: (data, actual_idx) or (None, -1) on error
        """
        # Read packet header (4 bytes) with retry
        pkt_header = b''
        header_timeout = 1.0
        header_start = time.time()
        
        while len(pkt_header) < PACKET_HEADER_SIZE and (time.time() - header_start) < header_timeout:
            remaining = PACKET_HEADER_SIZE - len(pkt_header)
            chunk = self.serial.read(remaining)
            if chunk:
                pkt_header += chunk
            else:
                time.sleep(0.001)
        
        if len(pkt_header) != PACKET_HEADER_SIZE:
            print(f"[RX] Packet header read failed, got {len(pkt_header)} bytes")
            return (None, -1)
        
        packet_idx = struct.unpack('<H', pkt_header[0:2])[0]
        data_len = struct.unpack('<H', pkt_header[2:4])[0]
        
        # Validate packet index
        if packet_idx != expected_idx:
            print(f"[RX] Packet index mismatch: expected {expected_idx}, got {packet_idx}")
            return (None, packet_idx)
        
        # Validate data length
        if data_len > expected_size or data_len == 0:
            print(f"[RX] Invalid data length: {data_len}, expected max {expected_size}")
            return (None, -1)
        
        # Read packet data (sent separately by STM32 via second CDC_Transmit_FS call)
        data = b''
        remaining = data_len
        read_timeout = 2.0
        start_time = time.time()
        
        while remaining > 0 and (time.time() - start_time) < read_timeout:
            chunk = self.serial.read(min(remaining, 2048))
            if len(chunk) > 0:
                data += chunk
                remaining -= len(chunk)
            else:
                time.sleep(0.001)
        
        if len(data) != data_len:
            print(f"[RX] Packet {packet_idx} data incomplete: got {len(data)}/{data_len} bytes")
            return (None, -1)
        
        return (data, packet_idx)
        
    def receive_loop(self):
        """Background thread for receiving image data with packet validation"""
        print("[RX] Receive thread started, waiting for data...")
        bytes_received_total = 0
        
        while self.running:
            try:
                # 打印接收缓冲区状态
                try:
                    waiting = self.serial.in_waiting
                    if waiting > 0:
                        print(f"[RX] Buffer has {waiting} bytes waiting")
                except:
                    pass
                
                # Search for frame header
                header_info = self.find_frame_header()
                if header_info is None:
                    if bytes_received_total == 0:
                        print("[RX] No frame header found, waiting...")
                    self.sync_errors += 1
                    continue
                
                print(f"[RX] Frame header found!")
                
                frame_idx, total_packets, packet_size = header_info
                self.stm32_frame_idx = frame_idx
                
                # Receive all packets with validation
                received = 0
                frame_valid = True
                start_time = time.time()
                timeout = 5.0
                
                for pkt_idx in range(total_packets):
                    if time.time() - start_time > timeout:
                        print(f"Frame {frame_idx}: Timeout at packet {pkt_idx}/{total_packets}")
                        self.sync_errors += 1
                        frame_valid = False
                        break
                    
                    # Receive packet with index validation
                    data, actual_idx = self.receive_data_packet(pkt_idx, packet_size)
                    
                    if data is None:
                        if actual_idx == -1:
                            print(f"Frame {frame_idx}: Packet {pkt_idx} read error")
                        else:
                            print(f"Frame {frame_idx}: Packet index mismatch (expected {pkt_idx}, got {actual_idx})")
                        self.sync_errors += 1
                        frame_valid = False
                        break
                    
                    # Copy data to frame buffer
                    data_len = len(data)
                    if received + data_len <= self.frame_size:
                        self.frame_buffer[received:received + data_len] = data
                        received += data_len
                    else:
                        print(f"Frame {frame_idx}: Buffer overflow at packet {pkt_idx}")
                        frame_valid = False
                        break
                
                # Check if frame is complete and valid
                if frame_valid and received == self.frame_size:
                    self.frame_count += 1
                    self.fps_count += 1
                    
                    # Calculate FPS
                    current_time = time.time()
                    if current_time - self.last_fps_time >= 1.0:
                        self.fps = self.fps_count
                        self.fps_count = 0
                        self.last_fps_time = current_time
                    
                    # Convert and display
                    image = self.rgb565_to_rgb888(bytes(self.frame_buffer))
                    self.current_image = image
                    
                    # Update UI in main thread
                    self.root.after(0, lambda: self.update_display(image))
                elif frame_valid and received != self.frame_size:
                    print(f"Frame {frame_idx}: Incomplete data ({received}/{self.frame_size} bytes)")
                    self.sync_errors += 1
                    
            except PermissionError as e:
                # USB CDC虚拟串口不支持某些Windows串口命令，忽略此错误
                if self.running and "ClearCommError" not in str(e):
                    print(f"Permission error: {e}")
                time.sleep(0.01)
            except serial.SerialException as e:
                if self.running:
                    print(f"Serial error: {e}")
                    # 尝试重新同步
                    try:
                        self.serial.reset_input_buffer()
                    except:
                        pass
                    time.sleep(0.1)
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                    time.sleep(0.1)
                    
    def update_display(self, image):
        """Update the image display and labels"""
        self.update_image(image)
        self.frame_label.config(text=f"GUI Frame: {self.frame_count} (STM32 Frame: {self.stm32_frame_idx})")
        self.fps_label.config(text=f"FPS: {self.fps}")
        self.sync_label.config(text=f"Sync Errors: {self.sync_errors}")
        
    def update_image(self, image):
        """Update the canvas with new image"""
        # Resize image to fit canvas
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        if canvas_width > 1 and canvas_height > 1:
            # Calculate scale to fit while maintaining aspect ratio
            scale_w = canvas_width / self.image_width
            scale_h = canvas_height / self.image_height
            scale = min(scale_w, scale_h)
            
            new_width = int(self.image_width * scale)
            new_height = int(self.image_height * scale)
            
            # Resize using PIL
            pil_image = Image.fromarray(image)
            pil_image = pil_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
            
            self.photo = ImageTk.PhotoImage(pil_image)
            
            # Center image on canvas
            x = (canvas_width - new_width) // 2
            y = (canvas_height - new_height) // 2
            
            self.canvas.delete("all")
            self.canvas.create_image(x, y, anchor=tk.NW, image=self.photo)
            
    def save_frame(self):
        """Save current frame to file"""
        if self.current_image is not None:
            filename = f"frame_{self.stm32_frame_idx}_{int(time.time())}.png"
            cv2.imwrite(filename, cv2.cvtColor(self.current_image, cv2.COLOR_RGB2BGR))
            messagebox.showinfo("Saved", f"Frame saved as {filename}")
        else:
            messagebox.showwarning("Warning", "No image to save")
            
    def on_closing(self):
        """Handle window close event"""
        self.disconnect()
        self.root.destroy()
        
    def run(self):
        """Start the GUI main loop"""
        self.root.mainloop()


def main():
    app = OV7725ViewerGUI()
    app.run()


if __name__ == "__main__":
    main()
