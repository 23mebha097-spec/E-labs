from PyQt5 import QtWidgets, QtCore

class JumpSlider(QtWidgets.QSlider):
    """
    A QSlider that jumps directly to the mouse click position,
    improving the feeling of synchronization between cursor and thumb.
    """
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            # Calculate new value based on click position
            opt = QtWidgets.QStyleOptionSlider()
            self.initStyleOption(opt)
            sr = self.style().subControlRect(QtWidgets.QStyle.CC_Slider, opt, QtWidgets.QStyle.SC_SliderHandle, self)
            
            if self.orientation() == QtCore.Qt.Horizontal:
                val = self.minimum() + ((self.maximum() - self.minimum()) * event.x()) / self.width()
            else:
                val = self.maximum() - ((self.maximum() - self.minimum()) * event.y()) / self.height()
            
            self.setValue(int(val))
            event.accept()
        super().mousePressEvent(event)
