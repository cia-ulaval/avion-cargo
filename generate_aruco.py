import argparse
from io import BytesIO
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

# PDF (reportlab)
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader


DICT_ID_TO_NAME = {
    0: "DICT_4X4_50",
    1: "DICT_4X4_100",
    2: "DICT_4X4_250",
    3: "DICT_4X4_1000",
    4: "DICT_5X5_50",
    5: "DICT_5X5_100",
    6: "DICT_5X5_250",
    7: "DICT_5X5_1000",
    8: "DICT_6X6_50",
    9: "DICT_6X6_100",
    10: "DICT_6X6_250",
    11: "DICT_6X6_1000",
    12: "DICT_7X7_50",
    13: "DICT_7X7_100",
    14: "DICT_7X7_250",
    15: "DICT_7X7_1000",
    16: "DICT_ARUCO_ORIGINAL",
}


def get_dictionary(dict_id: int):
    if dict_id not in DICT_ID_TO_NAME:
        raise SystemExit("Erreur: -d doit être dans 0..16")
    dict_name = DICT_ID_TO_NAME[dict_id]
    if not hasattr(cv2.aruco, dict_name):
        raise SystemExit(f"Erreur: ton OpenCV ne fournit pas {dict_name} dans cv2.aruco")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))


def draw_marker(dictionary, marker_id: int, marker_size_px: int, border_bits: int) -> np.ndarray:
    """
    Retourne une image uint8 (marker_size_px x marker_size_px).
    Supporte drawMarker ou generateImageMarker selon la build OpenCV.
    """
    if marker_size_px <= 0:
        raise ValueError("marker_size_px doit être > 0")
    if border_bits <= 0:
        raise ValueError("border_bits doit être > 0")

    img = np.zeros((marker_size_px, marker_size_px), dtype=np.uint8)

    if hasattr(cv2.aruco, "drawMarker"):
        cv2.aruco.drawMarker(dictionary, marker_id, marker_size_px, img, border_bits)
        return img

    if hasattr(cv2.aruco, "generateImageMarker"):
        cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size_px, img, border_bits)
        return img

    raise RuntimeError("Ton cv2.aruco n'a ni drawMarker ni generateImageMarker.")


def meters_to_points(m: float) -> float:
    # 1 inch = 0.0254 m, 1 point = 1/72 inch
    return (m / 0.0254) * 72.0


def meters_to_pixels(m: float, dpi: int) -> int:
    # pixels = inches * dpi
    inches = m / 0.0254
    return int(round(inches * dpi))


def save_pdf_with_physical_size(pdf_path: Path, marker_img_u8: np.ndarray, side_m: float, margin_m: float):
    """
    Crée un PDF dont:
      - la page fait (side_m + 2*margin_m) de côté
      - le marker fait side_m de côté, placé avec marges
    """
    side_pt = meters_to_points(side_m)
    margin_pt = meters_to_points(margin_m)

    page_w = side_pt + 2 * margin_pt
    page_h = side_pt + 2 * margin_pt

    # Convertir le marker en PNG en mémoire pour reportlab
    pil = Image.fromarray(marker_img_u8).convert("RGB")
    buf = BytesIO()
    pil.save(buf, format="PNG")
    buf.seek(0)

    c = canvas.Canvas(str(pdf_path), pagesize=(page_w, page_h))
    c.drawImage(ImageReader(buf), margin_pt, margin_pt, width=side_pt, height=side_pt, mask="auto")
    c.showPage()
    c.save()


def main():
    ap = argparse.ArgumentParser(description="Create an ArUco marker image (PNG/JPG) or print-accurate PDF (meters).")

    ap.add_argument("outfile", help="Sortie: .png/.jpg (pixels) ou .pdf (taille physique)")
    ap.add_argument("-d", type=int, required=True, help="Dictionary id (0..16)")
    ap.add_argument("--id", type=int, required=True, help="Marker id dans le dictionnaire")
    ap.add_argument("--bb", type=int, default=1, help="Border bits (default: 1)")
    ap.add_argument("--si", action="store_true", help="Afficher l'image générée (GUI)")

    # Mode image (pixels)
    ap.add_argument("--ms", type=int, default=200, help="Marker size en pixels (si sortie image).")

    # Mode PDF (taille physique)
    ap.add_argument("--size-m", type=float, default=None,
                    help="Taille du côté du marker en mètres (obligatoire si sortie .pdf). Ex: 0.064")
    ap.add_argument("--dpi", type=int, default=300,
                    help="Résolution interne du marker pour le PDF (qualité impression). Ex: 300 ou 600.")
    ap.add_argument("--margin-m", type=float, default=0.0,
                    help="Marge autour du marker dans le PDF (mètres). Ex: 0.005 pour 5 mm.")

    args = ap.parse_args()

    out = Path(args.outfile)
    out.parent.mkdir(parents=True, exist_ok=True)

    dictionary = get_dictionary(args.d)

    is_pdf = out.suffix.lower() == ".pdf"

    if is_pdf:
        if args.size_m is None or args.size_m <= 0:
            raise SystemExit("Erreur: pour une sortie .pdf, tu dois fournir --size-m (en mètres), ex: --size-m 0.064")

        if args.dpi < 72:
            raise SystemExit("Erreur: --dpi trop bas. Utilise 150, 300, 600...")

        # Générer le marker avec assez de pixels pour l'impression
        marker_size_px = meters_to_pixels(args.size_m, args.dpi)
        marker = draw_marker(dictionary, args.id, marker_size_px, args.bb)

        save_pdf_with_physical_size(out, marker, side_m=args.size_m, margin_m=max(0.0, args.margin_m))
        print(f"[OK] PDF saved to {out}")
        print(f"     dict={DICT_ID_TO_NAME[args.d]}, id={args.id}, side={args.size_m} m, dpi={args.dpi}, px={marker_size_px}, margin={args.margin_m} m")
        print("     Impression: 100% scale, désactiver 'fit to page'.")

        if args.si:
            # Aperçu (redimensionné pour écran)
            preview = cv2.resize(marker, (600, 600), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("marker preview", preview)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    else:
        # Mode image classique: --ms
        if args.ms <= 0:
            raise SystemExit("Erreur: --ms doit être > 0 (pixels).")
        marker = draw_marker(dictionary, args.id, args.ms, args.bb)

        ok = cv2.imwrite(str(out), marker)
        if not ok:
            raise SystemExit(f"Impossible d'écrire: {out}")

        print(f"[OK] Image saved to {out}  (dict={DICT_ID_TO_NAME[args.d]}, id={args.id}, ms={args.ms}px, bb={args.bb})")

        if args.si:
            cv2.imshow("marker", marker)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
