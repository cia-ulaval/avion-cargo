import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import logging
import threading

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")


class MainView:
    # Main GUI window using Tkinter
    # Shows camera feed, vehicle status, and marker info
    
    def __init__(self, controller):
        self.controller = controller
        self.root = None
        self.running = False
        
        # GUI components
        self.video_label = None
        self.status_text = None
        self.markers_text = None
        self.stats_text = None
        
        # Current data
        self.current_frame = None
        self.current_markers = []
        self.current_vehicle_status = None
        self.current_statistics = {}
        
        # Thread safety
        self._update_lock = threading.Lock()
        
        logging.info("MainView initialized")
    
    def initialize(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Avion Cargo - Precision Landing Control")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
        
        self._create_widgets()
        self.controller.subscribe_observer(self)
        
        self.running = True
        logging.info("GUI initialized and ready")
    
    def _create_widgets(self):
        # Create all GUI elements
        
        main_container = ttk.Frame(self.root, padding="10")
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Setup grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_container.columnconfigure(0, weight=3)
        main_container.columnconfigure(1, weight=1)
        main_container.rowconfigure(0, weight=1)
        
        # Left panel - Video feed
        video_frame = ttk.LabelFrame(main_container, text="Camera Feed", padding="5")
        video_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        self.video_label = ttk.Label(video_frame, text="Waiting for camera...", 
                                     background="black", foreground="white")
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # Right panel - Status and info
        right_panel = ttk.Frame(main_container)
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        right_panel.rowconfigure(0, weight=1)
        right_panel.rowconfigure(1, weight=1)
        right_panel.rowconfigure(2, weight=1)
        
        # Vehicle status
        status_frame = ttk.LabelFrame(right_panel, text="Vehicle Status", padding="5")
        status_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        self.status_text = tk.Text(status_frame, height=10, width=40, 
                                   font=("Courier", 10), state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        
        # Markers detected
        markers_frame = ttk.LabelFrame(right_panel, text="Detected Markers", padding="5")
        markers_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        self.markers_text = tk.Text(markers_frame, height=10, width=40,
                                    font=("Courier", 9), state=tk.DISABLED)
        self.markers_text.pack(fill=tk.BOTH, expand=True)
        
        # Statistics
        stats_frame = ttk.LabelFrame(right_panel, text="Statistics", padding="5")
        stats_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.stats_text = tk.Text(stats_frame, height=8, width=40,
                                  font=("Courier", 9), state=tk.DISABLED)
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        
        # Control buttons
        button_frame = ttk.Frame(right_panel)
        button_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(5, 0))
        
        ttk.Button(button_frame, text="Start", command=self._on_start).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Stop", command=self._on_stop).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Quit", command=self._on_closing).pack(side=tk.LEFT, padx=2)
    
    def update(self, data):
        # Called by controller when new data is available (observer pattern)
        with self._update_lock:
            self.current_frame = data.get('frame')
            self.current_markers = data.get('markers', [])
            self.current_vehicle_status = data.get('vehicle_status')
            self.current_statistics = data.get('statistics', {})
        
        # Schedule GUI update in main thread
        if self.root is not None:
            self.root.after(0, self._update_gui)
    
    def _update_gui(self):
        # Update all GUI elements with latest data
        with self._update_lock:
            frame = self.current_frame
            markers = self.current_markers
            status = self.current_vehicle_status
            stats = self.current_statistics
        
        if frame is not None:
            self._display_frame(frame, markers)
        
        if status is not None:
            self._update_vehicle_status(status)
        
        self._update_markers_display(markers)
        self._update_statistics(stats)
    
    def _display_frame(self, frame, markers=None):
        # Display camera frame with detected markers drawn on it
        try:
            frame_display = frame.copy()
            
            # Draw markers if any detected
            if markers and len(markers) > 0:
                for marker in markers:
                    if 'corners' in marker and marker['corners'] is not None:
                        # Get marker corners and reshape to 2D array
                        corners = marker['corners'].reshape(4, 2).astype(int)
                        
                        # Draw green box around marker
                        cv2.polylines(frame_display, [corners], True, (0, 255, 0), 2)
                        
                        # Draw red dots at corners
                        for corner in corners:
                            cv2.circle(frame_display, tuple(corner), 4, (0, 0, 255), -1)
                        
                        # Calculate center point
                        center_x = int(corners[:, 0].mean())
                        center_y = int(corners[:, 1].mean())
                        
                        # Add label with ID and distance
                        marker_id = marker.get('id', '?')
                        distance = marker.get('distance', 0)
                        label = f"ID:{marker_id} D:{distance:.2f}m"
                        
                        # Draw background for text
                        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(frame_display, 
                                    (center_x - 5, center_y - text_h - 10),
                                    (center_x + text_w + 5, center_y - 5),
                                    (0, 255, 0), -1)
                        
                        # Draw text
                        cv2.putText(frame_display, label, (center_x, center_y - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                        
                        # Draw crosshair at center
                        cv2.drawMarker(frame_display, (center_x, center_y),
                                     (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            
            # Convert BGR to RGB for display
            frame_rgb = cv2.cvtColor(frame_display, cv2.COLOR_BGR2RGB)
            
            # Resize to fit window
            display_width = 800
            h, w = frame_rgb.shape[:2]
            display_height = int(h * (display_width / w))
            frame_resized = cv2.resize(frame_rgb, (display_width, display_height))
            
            # Convert to Tkinter image
            image = Image.fromarray(frame_resized)
            photo = ImageTk.PhotoImage(image=image)
            
            self.video_label.configure(image=photo, text="")
            self.video_label.image = photo  # Keep reference to avoid garbage collection
            
        except Exception as e:
            logging.error(f"Error displaying frame: {e}", exc_info=True)
    
    def _update_vehicle_status(self, status):
        # Update vehicle status text display
        self.status_text.configure(state=tk.NORMAL)
        self.status_text.delete(1.0, tk.END)
        
        if status:
            status_str = f"""
Connected: {'Yes' if status['connected'] else 'No'}
Mode: {status['mode']}
Armed: {'Yes' if status['armed'] else 'No'}
Battery: {status['battery']:.1f}V
GPS Sats: {status['gps']['satellites']}
GPS Fix: {status['gps']['fix_type']}
Latitude: {status['location']['lat']:.6f}
Longitude: {status['location']['lon']:.6f}
Altitude: {status['location']['alt']:.1f}m
Heartbeat: {status['last_heartbeat']:.1f}s ago
"""
            self.status_text.insert(1.0, status_str)
        else:
            self.status_text.insert(1.0, "No vehicle data")
        
        self.status_text.configure(state=tk.DISABLED)
    
    def _update_markers_display(self, markers):
        # Update detected markers text
        self.markers_text.configure(state=tk.NORMAL)
        self.markers_text.delete(1.0, tk.END)
        
        if markers and len(markers) > 0:
            markers_str = f"Found {len(markers)} marker(s):\n\n"
            for i, marker in enumerate(markers):
                markers_str += f"Marker {i+1}:\n"
                markers_str += f"  ID: {marker['id']}\n"
                markers_str += f"  Distance: {marker['distance']:.2f}m\n"
                markers_str += f"  Angle X: {marker['angle_x']:.3f}rad\n"
                markers_str += f"  Angle Y: {marker['angle_y']:.3f}rad\n\n"
            self.markers_text.insert(1.0, markers_str)
        else:
            self.markers_text.insert(1.0, "No markers detected")
        
        self.markers_text.configure(state=tk.DISABLED)
    
    def _update_statistics(self, stats):
        # Update statistics panel
        self.stats_text.configure(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        
        if stats:
            precision_rate = stats.get('precision_rate', 0) * 100
            stats_str = f"""
Frames: {stats.get('frame_count', 0)}
Detections: {stats.get('detection_count', 0)}
Total Markers: {stats.get('total_detections', 0)}
Successful Poses: {stats.get('successful_poses', 0)}
FPS: {stats.get('fps', 0):.1f}
Detection Rate: {stats.get('detection_rate', 0)*100:.1f}%
Precision Rate: {precision_rate:.1f}%
Runtime: {stats.get('runtime', 0):.1f}s
Movements: {stats.get('movement_count', 0)}
"""
            self.stats_text.insert(1.0, stats_str)
        else:
            self.stats_text.insert(1.0, "No statistics")
        
        self.stats_text.configure(state=tk.DISABLED)
    
    def _on_start(self):
        # Start button clicked
        logging.info("Start button clicked")
        if not self.controller.running:
            if self.controller.init():
                self.controller.start()
            else:
                logging.error("Failed to initialize controller")
    
    def _on_stop(self):
        # Stop button clicked
        logging.info("Stop button clicked")
        self.controller.stop()
    
    def _on_closing(self):
        # Window close button clicked
        logging.info("Closing application...")
        self.running = False
        self.controller.cleanup()
        if self.root is not None:
            self.root.quit()
            self.root.destroy()
    
    def run(self):
        # Start GUI main loop (blocking call)
        if self.root is None:
            self.initialize()
        
        logging.info("Starting GUI main loop")
        self.root.mainloop()
        logging.info("GUI main loop ended")