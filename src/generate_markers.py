#!/usr/bin/env python3
# Generate ArUco markers for printing and testing with real camera
import cv2
import numpy as np
import argparse
import os

def generate_single_marker(marker_id, size_px=500, output_dir='markers'):
    # Generate single ArUco marker
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    
    # Add white border (important for detection)
    border_size = size_px // 10
    marker_with_border = cv2.copyMakeBorder(
        marker_img, 
        border_size, border_size, border_size, border_size,
        cv2.BORDER_CONSTANT, 
        value=255
    )
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Save marker
    filename = os.path.join(output_dir, f'aruco_marker_id{marker_id}.png')
    cv2.imwrite(filename, marker_with_border)
    print(f"Generated: {filename}")
    return marker_with_border, filename

def generate_marker_sheet(marker_ids, size_px=400, output_dir='markers'):
    # Generate sheet with multiple markers in 2x2 grid
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Calculate sheet size (2x2 grid)
    cols = 2
    rows = (len(marker_ids) + cols - 1) // cols
    
    spacing = 100  # Space between markers
    border_size = size_px // 10
    cell_size = size_px + 2 * border_size + spacing
    
    sheet_width = cols * cell_size + spacing
    sheet_height = rows * cell_size + spacing
    
    # Create white sheet
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    # Place markers
    for idx, marker_id in enumerate(marker_ids):
        row = idx // cols
        col = idx % cols
        
        # Generate marker
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
        marker_with_border = cv2.copyMakeBorder(
            marker_img,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255
        )
        
        # Calculate position
        x = spacing + col * cell_size
        y = spacing + row * cell_size
        
        # Place marker on sheet
        h, w = marker_with_border.shape
        sheet[y:y+h, x:x+w] = marker_with_border
        
        # Add ID label
        label_y = y + h + 40
        cv2.putText(sheet, f'ID: {marker_id}', (x + 20, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
        cv2.putText(sheet, f'5cm x 5cm', (x + 20, label_y + 35),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, 100, 2)
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Save sheet
    filename = os.path.join(output_dir, 'aruco_markers_sheet.png')
    cv2.imwrite(filename, sheet)
    print(f"Generated sheet: {filename}")
    return filename

def print_instructions():
    instructions = """
INSTRUCTIONS D'IMPRESSION:
  - Imprimer sur papier blanc mat (pas brillant)
  - Chaque marqueur doit faire EXACTEMENT 5cm x 5cm
  - Qualité maximale, bordures blanches importantes

Pour tester:
  cd src/
  python3 test_webcam.py --auto-start
"""
    print(instructions)

def main():
    parser = argparse.ArgumentParser(
        description='Generate ArUco markers for printing',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate single marker ID 0
  python generate_markers.py --id 0
  
  # Generate multiple markers
  python generate_markers.py --ids 0 1 2 3
  
  # Generate a print-ready sheet
  python generate_markers.py --sheet --ids 0 1 2 3
  
  # High resolution for large prints
  python generate_markers.py --id 0 --size 1000
        """
    )
    
    parser.add_argument('--id', type=int, help='Generate single marker with this ID')
    parser.add_argument('--ids', type=int, nargs='+', help='Generate multiple markers')
    parser.add_argument('--sheet', action='store_true', help='Generate printable sheet')
    parser.add_argument('--size', type=int, default=500, help='Marker size in pixels (default: 500)')
    parser.add_argument('--output', default='markers', help='Output directory (default: markers/)')
    
    args = parser.parse_args()
    
    print("\nArUco Marker Generator")
    print("Dictionary: DICT_4X4_50 (IDs 0-49)")
    print(f"Output directory: {args.output}/\n")
    
    # Determine which markers to generate
    marker_ids = []
    if args.id is not None:
        marker_ids = [args.id]
    elif args.ids:
        marker_ids = args.ids
    else:
        # Default: generate first 4 markers
        marker_ids = [0, 1, 2, 3]
        print("No IDs specified, generating default markers: 0, 1, 2, 3\n")
    
    # Generate markers
    if args.sheet:
        # Generate sheet with all markers
        generate_marker_sheet(marker_ids, args.size, args.output)
    else:
        # Generate individual markers
        for marker_id in marker_ids:
            generate_single_marker(marker_id, args.size, args.output)
    
    # Print instructions
    print_instructions()
    
    print(f"Files saved in: {args.output}/")
    print("Ready to print and test!\n")

if __name__ == "__main__":
    main()
