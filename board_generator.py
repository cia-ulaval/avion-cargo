import argparse
from io import BytesIO
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

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


def meters_to_points(m: float) -> float:
    # 1 inch = 0.0254 m, 1 pt = 1/72 inch
    return (m / 0.0254) * 72.0


def meters_to_pixels(m: float, dpi: int) -> int:
    inches = m / 0.0254
    return int(round(inches * dpi))


def build_board_image_px(markers_x: int, markers_y: int,
                         marker_len_px: int, marker_sep_px: int,
                         dictionary, margins_px: int, border_bits: int) -> np.ndarray:
    """
    Génère une image uint8 de la GridBoard comme le C++:
    imageSize = w*(l+s)-s + 2*margins
    """
    if markers_x <= 0 or markers_y <= 0:
        raise ValueError("w/h doivent être > 0")
    if marker_len_px <= 0 or marker_sep_px < 0:
        raise ValueError("l-px doit être > 0 et s-px >= 0")
    if margins_px < 0:
        raise ValueError("margins doit être >= 0")
    if border_bits <= 0:
        raise ValueError("border bits (bb) doit être > 0")

    img_w = markers_x * (marker_len_px + marker_sep_px) - marker_sep_px + 2 * margins_px
    img_h = markers_y * (marker_len_px + marker_sep_px) - marker_sep_px + 2 * margins_px
    image_size = (img_w, img_h)  # (w,h)

    # Board
    board = cv2.aruco.GridBoard((markers_x, markers_y), float(marker_len_px), float(marker_sep_px), dictionary)


    board_img = None

    if hasattr(board, "generateImage"):
        board_img = board.generateImage(image_size, marginSize=margins_px, borderBits=border_bits)
    else:
        # draw(imageSize[, img[, marginSize[, borderBits]]]) -> img
        board_img = board.draw(image_size, margins_px, border_bits)

    # Assurer uint8 grayscale
    board_img = np.array(board_img, dtype=np.uint8)
    return board_img


def save_pdf_exact_size(pdf_path: Path, gray_u8: np.ndarray,
                        page_w_m: float, page_h_m: float,
                        content_w_m: float, content_h_m: float,
                        margin_left_m: float, margin_bottom_m: float):
    """
    Place l'image (gray_u8) dans un PDF, avec une page (page_w_m x page_h_m),
    et un contenu (content_w_m x content_h_m) placé à (margin_left_m, margin_bottom_m).
    """
    page_w_pt = meters_to_points(page_w_m)
    page_h_pt = meters_to_points(page_h_m)
    content_w_pt = meters_to_points(content_w_m)
    content_h_pt = meters_to_points(content_h_m)
    x_pt = meters_to_points(margin_left_m)
    y_pt = meters_to_points(margin_bottom_m)

    pil = Image.fromarray(gray_u8).convert("RGB")
    buf = BytesIO()
    pil.save(buf, format="PNG")
    buf.seek(0)

    c = canvas.Canvas(str(pdf_path), pagesize=(page_w_pt, page_h_pt))
    c.drawImage(ImageReader(buf), x_pt, y_pt, width=content_w_pt, height=content_h_pt, mask="auto")
    c.showPage()
    c.save()


def main():
    ap = argparse.ArgumentParser(description="Create an ArUco GridBoard (PNG/JPG pixels or PDF meters).")

    ap.add_argument("outfile", help="Sortie: .png/.jpg (pixels) ou .pdf (taille physique)")
    ap.add_argument("-d", type=int, required=True, help="Dictionary id (0..16)")
    ap.add_argument("-w", type=int, required=True, help="Nombre de marqueurs en X")
    ap.add_argument("-hm", dest="h_", type=int, required=True, help="Nombre de marqueurs en Y")
    ap.add_argument("--bb", type=int, default=1, help="Border bits (default: 1)")
    ap.add_argument("--si", action="store_true", help="Afficher l'image générée")

    # --- Mode image (pixels)
    ap.add_argument("--l-px", type=int, default=None, help="Marker side length en pixels (mode image)")
    ap.add_argument("--s-px", type=int, default=None, help="Separation en pixels (mode image)")
    ap.add_argument("--m-px", type=int, default=None, help="Margins en pixels (default = s-px)")

    # --- Mode PDF (mètres) ---
    ap.add_argument("--l-m", type=float, default=None, help="Marker side length en mètres (mode PDF)")
    ap.add_argument("--s-m", type=float, default=None, help="Separation en mètres (mode PDF)")
    ap.add_argument("--m-m", type=float, default=None, help="Margins en mètres (default = s-m)")

    ap.add_argument("--dpi", type=int, default=300, help="DPI interne pour le rendu PDF (150/300/600...)")

    args = ap.parse_args()

    out = Path(args.outfile)
    out.parent.mkdir(parents=True, exist_ok=True)

    dictionary = get_dictionary(args.d)
    is_pdf = out.suffix.lower() == ".pdf"

    if is_pdf:
        # --- PDF: tailles physiques en mètres ---
        if args.l_m is None or args.s_m is None:
            raise SystemExit("Pour une sortie .pdf, tu dois fournir --l-m et --s-m (mètres).")
        if args.l_m <= 0 or args.s_m < 0:
            raise SystemExit("--l-m doit être >0 et --s-m >=0")
        if args.dpi < 72:
            raise SystemExit("--dpi trop bas. Utilise 150/300/600...")

        margin_m = args.s_m if args.m_m is None else args.m_m
        if margin_m < 0:
            raise SystemExit("--m-m doit être >= 0")

        # Convertir en pixels pour générer l'image, tout en gardant taille physique pour le PDF
        l_px = meters_to_pixels(args.l_m, args.dpi)
        s_px = meters_to_pixels(args.s_m, args.dpi)
        m_px = meters_to_pixels(margin_m, args.dpi)

        board_img = build_board_image_px(
            markers_x=args.w,
            markers_y=args.h_,
            marker_len_px=l_px,
            marker_sep_px=s_px,
            dictionary=dictionary,
            margins_px=m_px,
            border_bits=args.bb
        )

        # Taille physique du contenu (board) = calcul C++ mais en mètres
        content_w_m = args.w * (args.l_m + args.s_m) - args.s_m + 2 * margin_m
        content_h_m = args.h_ * (args.l_m + args.s_m) - args.s_m + 2 * margin_m

        # On fait une page exactement égale au contenu (simple & fiable)
        page_w_m = content_w_m
        page_h_m = content_h_m

        save_pdf_exact_size(
            pdf_path=out,
            gray_u8=board_img,
            page_w_m=page_w_m,
            page_h_m=page_h_m,
            content_w_m=content_w_m,
            content_h_m=content_h_m,
            margin_left_m=0.0,
            margin_bottom_m=0.0
        )

        print(f"[OK] PDF saved to {out}")
        print(f"     dict={DICT_ID_TO_NAME[args.d]}, w={args.w}, h={args.h_}")
        print(f"     marker={args.l_m} m, sep={args.s_m} m, margin={margin_m} m, dpi={args.dpi}")
        print(f"     page size: {page_w_m:.6f} m x {page_h_m:.6f} m")
        print("     Impression: 100% scale, désactiver 'fit to page'.")

        if args.si:
            preview = cv2.resize(board_img, (min(900, board_img.shape[1]), min(900, board_img.shape[0])),
                                 interpolation=cv2.INTER_NEAREST)
            cv2.imshow("board preview", preview)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    else:
        # --- Image: paramètres en pixels ---
        if args.l_px is None or args.s_px is None:
            raise SystemExit("Pour une sortie image (.png/.jpg), tu dois fournir --l-px et --s-px (pixels).")
        if args.l_px <= 0 or args.s_px < 0:
            raise SystemExit("--l-px doit être >0 et --s-px >=0")

        margins_px = args.s_px if args.m_px is None else args.m_px

        board_img = build_board_image_px(
            markers_x=args.w,
            markers_y=args.h_,
            marker_len_px=args.l_px,
            marker_sep_px=args.s_px,
            dictionary=dictionary,
            margins_px=margins_px,
            border_bits=args.bb
        )

        ok = cv2.imwrite(str(out), board_img)
        if not ok:
            raise SystemExit(f"Impossible d'écrire: {out}")

        print(f"[OK] Image saved to {out}")
        print(f"     dict={DICT_ID_TO_NAME[args.d]}, w={args.w}, h={args.h_}, l={args.l_px}px, s={args.s_px}px, m={margins_px}px")

        if args.si:
            cv2.imshow("board", board_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
