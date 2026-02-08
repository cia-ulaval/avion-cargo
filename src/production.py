#!/usr/bin/env python3
# Production mode - uses real camera and real drone
# WARNING: Only use after testing! Real drone will respond to commands.
import sys
import os
import argparse
import logging
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera import Camera  # Real camera
from image_treatment import ImageTreatment
from vehicle_interface import VehicleInterface  # Real vehicle
from interface.interface_waiter import Interface
from interface.controller import Controller
from interface.main_view import MainView

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('production.log'),
            logging.StreamHandler()
        ]
    )

def print_banner():
    print("\n" + "="*50)
    print("  AVION CARGO - Production Mode")
    print("  WARNING: Real hardware!")
    print("="*50)
    print("  Camera: Raspberry Pi Camera")
    print("  Vehicle: Real drone (MAVLink)")
    print("  Test on ground first!")
    print("="*50 + "\n")

def verify_safety():
    # Confirm safety checks before starting
    print("\nSafety checks:")
    print("1. Tested with demo and test_webcam?")
    print("2. Landing area clear?")
    print("3. Drone disarmed for ground tests?")
    print("4. Manual override ready?")
    
    response = input("\nAll checks OK? (yes/no): ").strip().lower()
    if response not in ['yes', 'y']:
        print("Safety check failed. Please complete checks first.")
        return False
    
    print("Safety checks OK")
    return True

def main():
    # Main entry point for production mode
    parser = argparse.ArgumentParser(description='Production mode with full hardware')
    parser.add_argument('--auto-start', action='store_true', 
                       help='Automatically start processing (CAREFUL!)')
    parser.add_argument('--skip-safety', action='store_true',
                       help='Skip safety verification (NOT RECOMMENDED)')
    parser.add_argument('--no-vehicle', action='store_true',
                       help='Run without vehicle connection (debug mode)')
    args = parser.parse_args()
    
    # Setup
    setup_logging()
    print_banner()
    
    # Safety verification
    if not args.skip_safety and not args.no_vehicle:
        if not verify_safety():
            return
    
    logging.info("Starting production mode...")
    logging.info("Camera: Real | Vehicle: Real")
    
    # Create the observer pattern base
    waiter = Interface()
    
    # Create components
    logging.info("Initializing components...")
    camera = Camera()
    image_treatment = ImageTreatment()
    
    # Create vehicle interface (or skip if debugging)
    if args.no_vehicle:
        logging.warning("Running without vehicle (debug mode)")
        vehicle = None
    else:
        vehicle = VehicleInterface()
        logging.info("Vehicle interface created")
    
    logging.info("Camera and image treatment ready")
    
    # Create controller
    controller = Controller(
        waiter=waiter,
        camera=camera,
        image_treatment=image_treatment,
        vehicle_interface=vehicle
    )
    
    # Create GUI
    main_view = MainView(controller)
    main_view.initialize()
    controller.waiter = waiter
    
    # Initialize hardware
    logging.info("Connecting to camera...")
    if not controller.init():
        logging.error("Failed to initialize camera!")
        logging.error("Check camera connection and try: libcamera-hello")
        return
    logging.info("Camera connected")
    
    if vehicle is not None:
        logging.info("Connecting to vehicle via MAVLink...")
        if controller.vehicle_interface and controller.vehicle_interface.is_connected:
            logging.info("Vehicle connected")
            status = controller.vehicle_interface.get_vehicle_status()
            logging.info(f"Mode: {status['mode']}, Battery: {status['battery']:.1f}V")
        else:
            logging.warning("Vehicle not connected, continuing anyway")
    
    logging.info("System ready")
    
    # Auto-start if requested
    if args.auto_start:
        logging.warning("Auto-start enabled - processing will begin automatically")
        main_view.root.after(1000, controller.start)
    else:
        logging.info("Click Start when ready")
    
    # Run GUI
    try:
        logging.info("Starting GUI...")
        main_view.run()
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    finally:
        logging.info("Cleaning up...")
        controller.cleanup()
        logging.info("Done")

if __name__ == "__main__":
    main()
