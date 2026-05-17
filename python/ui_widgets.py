"""
ui_widgets.py  —  Custom Qt widgets for ABB IRB 1600 Digital Twin.
"""
from __future__ import annotations
import numpy as np
from PyQt5.QtWidgets import QWidget, QPlainTextEdit
from PyQt5.QtCore    import Qt, QRect, QSize, QRectF
from PyQt5.QtGui     import QPainter, QColor, QFont, QPen, QLinearGradient

_DH = [
    [150, np.pi/2, 486], [700, 0.0,       0],
    [115, np.pi/2,   0], [  0,-np.pi/2, 625],
    [  0, np.pi/2,   0], [  0, 0.0,     100],
]

def _dh_mat(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct,-st*ca,st*sa,a*ct],
                     [st, ct*ca,-ct*sa,a*st],
                     [ 0,    sa,    ca,   d],
                     [ 0,     0,     0,   1]])

def _fk_positions(q_deg: np.ndarray) -> list[tuple[float,float]]:
    q = np.deg2rad(q_deg)
    T = np.eye(4)
    pts = [(0.0, 0.0)]
    for i, (a, alpha, d) in enumerate(_DH):
        T = T @ _dh_mat(q[i], d, a, alpha)
        pts.append((T[0, 3], T[1, 3]))
    return pts



class GripperBar(QWidget):
    """
    Two animated 'fingers' that open / close, plus a % label.
    Drop-in visual companion for the gripper slider.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pct = 100.0
        self.setFixedHeight(30)
        self.setMinimumWidth(80)
        self.setToolTip("Gripper opening visualisation")

    def set_pct(self, pct: float):
        self._pct = float(np.clip(pct, 0, 100))
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H = self.width(), self.height()
        cx   = W // 2
        pad  = 4

        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#0d1117"))
        p.drawRoundedRect(0, 0, W, H, 5, 5)

        max_half_gap = int(cx * 0.45)
        half_gap     = int(self._pct / 100.0 * max_half_gap)
        fw           = max(cx - half_gap - pad * 2, 6)

        color = (QColor("#FF4444") if self._pct < 8
                 else QColor("#FFA500") if self._pct < 20
                 else QColor("#7ecbff"))

        def _finger_gradient(x0):
            g = QLinearGradient(x0, 0, x0 + fw, 0)
            g.setColorAt(0.0, color.darker(120))
            g.setColorAt(1.0, color)
            return g

        lx = cx - half_gap - fw
        p.setBrush(_finger_gradient(lx))
        p.drawRoundedRect(lx, pad, fw, H - pad*2, 4, 4)

        rx = cx + half_gap
        p.setBrush(_finger_gradient(rx))
        p.drawRoundedRect(rx, pad, fw, H - pad*2, 4, 4)

        if self._pct < 8:
            p.setPen(QPen(QColor("#FF4444"), 2))
            p.drawLine(cx, pad + 2, cx, H - pad - 2)

        p.setPen(QColor("#8b949e"))
        p.setFont(QFont("Courier New", 8))
        p.drawText(QRect(0, 0, W, H), Qt.AlignCenter,
                   f"{self._pct:.0f} %")
        p.end()



class WorkspaceMap(QWidget):
    """
    Top-down (XY plane) 2D view of the robot configuration.
    Links drawn in orange, TCP in blue, joints as dots.
    Call update_q() every animation tick.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self._q_deg     = np.zeros(6)
        self._obj_pos   = None   # (x, y) in mm, or None
        self.setMinimumSize(150, 150)
        self.setMaximumSize(240, 240)

    def update_q(self, q_deg: np.ndarray):
        self._q_deg = q_deg.copy()
        self.update()

    def set_object_xy(self, x: float, y: float, visible: bool = True):
        self._obj_pos = (x, y) if visible else None
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        W, H   = self.width(), self.height()
        margin = 14
        size   = min(W, H) - margin * 2
        scale  = size / 3000.0   # 1500 mm radius  → pixels
        cx     = W // 2
        cy     = H // 2

        def px(x, y):
            return int(cx + x * scale), int(cy - y * scale)

        p.fillRect(0, 0, W, H, QColor("#0a0e13"))

        r_ws = int(1450 * scale)
        p.setPen(QPen(QColor("#1c2128"), 1, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(cx - r_ws, cy - r_ws, r_ws * 2, r_ws * 2)

        p.setPen(QPen(QColor("#1c2128"), 1))
        p.drawLine(margin, cy, W - margin, cy)
        p.drawLine(cx, margin, cx, H - margin)

        if self._obj_pos is not None:
            ox, oy = px(*self._obj_pos)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#d8b15a"))
            p.drawEllipse(ox - 5, oy - 5, 10, 10)

        pts = _fk_positions(self._q_deg)
        p.setPen(QPen(QColor("#E85D24"), 2))
        for i in range(len(pts) - 1):
            p.drawLine(*px(*pts[i]), *px(*pts[i+1]))

        for i, (x, y) in enumerate(pts):
            qx, qy = px(x, y)
            if i == 0:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor("#30363d"))
                p.drawEllipse(qx - 5, qy - 5, 10, 10)
            elif i == len(pts) - 1:
                p.setPen(QPen(QColor("#58a6ff"), 1))
                p.setBrush(QColor("#58a6ff"))
                p.drawEllipse(qx - 5, qy - 5, 10, 10)
            else:
                p.setPen(Qt.NoPen)
                p.setBrush(QColor("#ff7700"))
                p.drawEllipse(qx - 3, qy - 3, 6, 6)

        p.setPen(QColor("#30363d"))
        p.setFont(QFont("Segoe UI", 7))
        p.drawText(4, H - 5, "top view (XY)")
        p.end()



class _LineNumberGutter(QWidget):
    def __init__(self, editor: "LineNumberedEdit"):
        super().__init__(editor)
        self._ed = editor

    def sizeHint(self) -> QSize:
        return QSize(self._ed._gutter_width(), 0)

    def paintEvent(self, event):
        self._ed._paint_gutter(event)


class LineNumberedEdit(QPlainTextEdit):

    def __init__(self, parent=None):
        super().__init__(parent)
        self._gutter = _LineNumberGutter(self)
        self.blockCountChanged.connect(self._update_gutter_width)
        self.updateRequest.connect(self._scroll_gutter)
        self._update_gutter_width(0)

    def _gutter_width(self) -> int:
        digits = max(1, len(str(max(1, self.blockCount()))))
        return 10 + self.fontMetrics().boundingRect("9").width() * digits

    def _update_gutter_width(self, _=0):
        self.setViewportMargins(self._gutter_width(), 0, 0, 0)

    def _scroll_gutter(self, rect, dy):
        if dy:
            self._gutter.scroll(0, dy)
        else:
            self._gutter.update(0, rect.y(), self._gutter.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_gutter_width()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._gutter.setGeometry(
            QRect(cr.left(), cr.top(), self._gutter_width(), cr.height()))

    def _paint_gutter(self, event):
        p = QPainter(self._gutter)
        p.fillRect(event.rect(), QColor("#0a0e13"))

        block  = self.firstVisibleBlock()
        num    = block.blockNumber()
        top    = int(self.blockBoundingGeometry(block)
                     .translated(self.contentOffset()).top())
        bottom = top + int(self.blockBoundingRect(block).height())
        lh     = self.fontMetrics().height()

        p.setFont(self.font())
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                cur = self.textCursor().blockNumber() == num
                p.setPen(QColor("#58a6ff") if cur else QColor("#3d444d"))
                p.drawText(0, top,
                           self._gutter.width() - 5, lh,
                           Qt.AlignRight, str(num + 1))
            block  = block.next()
            top    = bottom
            bottom = top + int(self.blockBoundingRect(block).height())
            num   += 1
        p.end()
