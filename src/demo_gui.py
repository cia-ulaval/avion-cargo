#!/usr/bin/env python3
"""
Avion Cargo - GUI Demo Mode
============================
Complete demo using Tkinter GUI with mock camera and vehicle.
Perfect for presentations and testing without hardware.

Usage:
    python demo_gui.py              # Normal startup with GUI
    python demo_gui.py --auto-start # Auto-start processing
"""
import sys
import os
import argparse
import logging

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mock_camera import MockCamera
from mock_vehicle import MockVehicleInterface
from image_treatment import ImageTreatment
from interface.interface_waiter import Interface
from interface.controller import Controller
from interface.main_view import MainView

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('demo_gui.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def print_banner():
    print("\n" + "="*50)
    print("  AVION CARGO - Landing System Demo")
    print("="*50)
    print("  Using simulated camera and vehicle")
    print("  GUI shows real-time ArUco detection")
    print("="*50 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Avion Cargo Demo")
    parser.add_argument('--auto-start', action='store_true', help='Start automatically')
    parser.add_argument('--marker-size', type=float, default=0.05, help='Marker size in meters')
    args = parser.parse_args()
    
    print_banner()
    
    try:
        logger.info("Starting demo...")
        
        # Create the observer pattern base
        waiter = Interface()
        
        # Setup mock components for demo
        camera = MockCamera(width=640, height=480)
        image_treatment = ImageTreatment(marker_length_m=args.marker_size)
        vehicle = MockVehicleInterface()
        
        # Create controller to manage everything
        controller = Controller(
            waiter=waiter,
            camera=camera,
            image_treatment=image_treatment,
            vehicle_interface=vehicle
        )
        
        # Create the GUI
        main_view = MainView(controller)
        controller.waiter = waiter
        
        # Start automatically if requested
        if args.auto_start:
            if controller.init():
                controller.start()
                logger.info("Started automatically")
            else:
                logger.error("Failed to start")
                return 1
        
        # Run the GUI (this blocks until user closes window)
        logger.info("GUI ready - you can now interact with it")
        main_view.run()
        
        logger.info("GUI closed - cleaning up...")
        controller.cleanup()
        
        return 0
        
    except KeyboardInterrupt:
        logger.info("\nInterrupted by user")
        return 0
    except Exception as e:
        logger.exception(f"Fatal error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
