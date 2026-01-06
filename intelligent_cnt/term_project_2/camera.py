from picamera2 import Picamera2
import cv2
import numpy as np

class cam:
    def __init__(self):
        WIDTH = 640
        HEIGHT = 480

        self.picam2 = Picamera2()
        cfg = self.picam2.create_preview_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(cfg)
        self.picam2.start()
    
    def img_preprocess(self, image):
        height, _, _ = image.shape
        image = image[int(height/2):,:,:]
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        image = cv2.resize(image, (200,66))
        # image = cv2.GaussianBlur(image,(5,5),0)
        image = cv2.GaussianBlur(image,(9, 9),0)
        binary_img = image[:, :, 0]

        # this code uses OTSU binarization
        # _, image = cv2.threshold(
        #     binary_img, 
        #     0, 
        #     255, 
        #     cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        # )

        # image = cv2.adaptiveThreshold(
        #     binary_img, 
        #     255, # Max value
        #     cv2.ADAPTIVE_THRESH_GAUSSIAN_C, # Method to calculate threshold
        #     cv2.THRESH_BINARY_INV, # Invert the threshold (like you had)
        #     15, # BlockSize: Size of the neighborhood (must be odd)
        #     3   # C: A constant subtracted from the mean
        # )
        
        # 90 works for clock wise 5 times.
        # _, image = cv2.threshold(binary_img, 90, 255, cv2.THRESH_BINARY_INV)

        _, image = cv2.threshold(binary_img, 95, 255, cv2.THRESH_BINARY_INV)

        return image
    
    def get_cnt(self, processed):
        """
        Gets the 200x66 processed binary image, finds the main path,
        and returns a NEW binary image containing ONLY that path.
        """
        # Get the dimensions of the processed (200x66) image
        print(processed.shape)
        # if len(processed) == 3:

        h, w = processed.shape
        line_mask = np.zeros_like(processed)

        check_point = (w // 2, h - 1)
        min_area = 100

        contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("No contours found.")
            return line_mask
        
        large_contours = []
        for c in contours:
            if cv2.contourArea(c) > min_area:
                large_contours.append(c)
        
        if not large_contours:
            print("No large contours found.")
            return line_mask

        # --- 5. Filter by Position (Bottom-Left) ---
        best_contour = None
        max_distance = -np.inf # We only care about contours that contain the point

        for c in large_contours:
            # Check signed distance
            dist = cv2.pointPolygonTest(c, check_point, True) 

            # If point is inside (dist > 0) and it's the 
            # "most inside" one we've seen, save it.
            if dist > max_distance:
                max_distance = dist
                best_contour = c

        # --- 6. Draw the best contour (if found) ---
        if best_contour is not None:
            # Draw the single best contour onto our blank mask
            # We use 255 (white) as the color
            # We use cv2.FILLED (-1) to fill the shape, not just outline it
            cv2.drawContours(line_mask, [best_contour], -1, 255, cv2.FILLED)
        
        # Return the new binary image (either blank or with the line)
        print("Returning line mask.")
        return line_mask

    def get_image(self):
        """
        Gets current image, and preproceed image.
        """
        image = self.picam2.capture_array()
        processed = self.img_preprocess(image)
        cnt = self.get_cnt(processed)
        cnt = cv2.cvtColor(cnt, cv2.COLOR_GRAY2RGB)
        return image, processed, cnt

if __name__ == "__main__":
    # This che`ck prevents the code from running when imported

    print("Starting camera... press 'q' to quit.")
    my_cam = cam()

    while True:
        # 1. Get the images
        original_rgb, processed, cnt = my_cam.get_image()
        
        # 3. Create visualization images
        # main_contour = cv2.cvtColor(cnt, cv2.COLOR_GRAY2RGB)

        # 4. Show the images
        cv2.imshow("Original Camera (RGB)", original_rgb)
        cv2.imshow("Processed Mask (200x66)", processed)
        cv2.imshow("Final Contour", cnt)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cv2.destroyAllWindows()
    my_cam.picam2.stop()
    print("Camera stopped.")

# import glob
# if __name__ == "__main__":
#     # This che`ck prevents the code from running when imported

#     print("Starting camera... press 'q' to quit.")
#     my_cam = cam()

#     image_path = '/home/user/Documents/edge_data'
#     image_files = glob.glob(f'{image_path}/*.png')

#     for file in image_files:

#         image = cv2.imread(file)
#         cv2.imshow("Original Image", image)

#         image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
#         cnt = my_cam.get_cnt(image)

#         cv2.imshow("Contour", cnt)
#         key = cv2.waitKey(0)
#         if key == ord('n'):
#             continue
#         elif key == ord('q'):
#             break

#     cv2.destroyAllWindows()
#     my_cam.picam2.stop()
#     print("Camera stopped.")