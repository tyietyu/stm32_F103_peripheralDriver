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
FRAME_HEADER_SIZE = 8

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
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )
            self.running = True
            self.sync_errors = 0
            self.connect_btn.config(text="Disconnect")
            self.save_btn.config(state=tk.NORMAL)
            self.port_combo.config(state=tk.DISABLED)
            self.refresh_btn.config(state=tk.DISABLED)
            self.status_label.config(text=f"Status: Connected to {port}")
            
            # Start receive thread
            self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.receive_thread.start()
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            
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
        Header format: [0xAA, 0x55, frame_idx_L, frame_idx_H, width_L, height_L, 0xA5, 0x5A]
        Returns: (frame_idx, width, height) or None if not found
        """
        timeout = 5.0
        start_time = time.time()
        
        while self.running and (time.time() - start_time < timeout):
            # Read one byte at a time to find sync
            byte1 = self.serial.read(1)
            if len(byte1) == 0:
                continue
                
            if byte1[0] == FRAME_HEADER_SYNC1:
                byte2 = self.serial.read(1)
                if len(byte2) == 0:
                    continue
                    
                if byte2[0] == FRAME_HEADER_SYNC2:
                    # Found sync, read rest of header
                    header_rest = self.serial.read(FRAME_HEADER_SIZE - 2)
                    if len(header_rest) == FRAME_HEADER_SIZE - 2:
                        # Verify end markers
                        if header_rest[4] == FRAME_HEADER_END1 and header_rest[5] == FRAME_HEADER_END2:
                            # Parse header (little-endian for frame_idx, single byte for width/height)
                            frame_idx = struct.unpack('<H', header_rest[0:2])[0]
                            width = header_rest[2]   # Low 8 bits only
                            height = header_rest[3]  # Low 8 bits only
                            return (frame_idx, width, height)
        return None
        
    def receive_loop(self):
        """Background thread for receiving image data"""
        while self.running:
            try:
                # Search for frame header
                header_info = self.find_frame_header()
                if header_info is None:
                    self.sync_errors += 1
                    continue
                
                frame_idx, width, height = header_info
                
                # Update image dimensions if changed
                if width != self.image_width or height != self.image_height:
                    self.image_width = width
                    self.image_height = height
                    self.frame_size = width * height * BYTES_PER_PIXEL
                    self.frame_buffer = bytearray(self.frame_size)
                    self.root.after(0, lambda w=width, h=height: self.resolution_label.config(text=f"Resolution: {w}x{h}"))
                
                self.stm32_frame_idx = frame_idx
                
                # Receive frame data
                received = 0
                start_time = time.time()
                timeout = 5.0
                
                while received < self.frame_size and self.running:
                    if time.time() - start_time > timeout:
                        self.sync_errors += 1
                        break
                        
                    available = self.serial.in_waiting
                    if available > 0:
                        to_read = min(available, self.frame_size - received)
                        data = self.serial.read(to_read)
                        self.frame_buffer[received:received + len(data)] = data
                        received += len(data)
                    else:
                        time.sleep(0.001)
                
                if received == self.frame_size:
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
