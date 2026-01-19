import numpy as np
import cv2

#Lane detection class for a mobile robot using camera input.

class LaneDetector:
    
    def __init__(self, lane_width_cm: float = 8.0, check_offset: int = 50, row_thickness: int = 5):
        self.lane_width_cm = lane_width_cm
        self.check_offset = check_offset
        self.row_thickness = row_thickness

    def compute_error(self, image: np.ndarray) -> float:
        """
        Compute lateral error of robot relative to lane center.
        """
        height, width, _ = image.shape

        # Select row to sample
        start = height - self.check_offset - self.row_thickness
        end = height - self.check_offset
        row_slice = image[start:end, :, :3]

        # Average across rows
        row_image = np.mean(row_slice, axis=0).astype(np.uint8)

        # Convert to grayscale
        gray_row = cv2.cvtColor(row_image[np.newaxis, :, :], cv2.COLOR_RGB2GRAY)

        # Threshold to binary
        _, binary_row = cv2.threshold(
            gray_row, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Find lane edges
        indices = np.where(binary_row[0] > 0)[0]

        if len(indices) >= 2:
            left_edge = indices[0]
            right_edge = indices[-1]
            center = (left_edge + right_edge) // 2
        else:
            left_edge = 0
            right_edge = width
            center = width // 2

        # Convert pixel error to cm
        lane_pixel_width = right_edge - left_edge
        if lane_pixel_width > 0:
            cm_per_pixel = self.lane_width_cm / lane_pixel_width
            pixel_error = center - (width // 2)
            error = pixel_error * cm_per_pixel
        else:
            error = 0.0

        return error