# import cv2
# import numpy as np

# image = cv2.imread('/home/neha/Downloads/object.jpeg', cv2.IMREAD_GRAYSCALE)

# def click_event(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:
#         print(f'Coordinates: (x={x}, y={y})')
#         # Mark the clicked point
#         cv2.circle(param, (x, y), 5, (255, 0, 255), -1)
#         cv2.imshow('Image with Clicked Point', param)

# #Blur
# blurred = cv2.GaussianBlur(image, (17, 17), 0)
# cv2.imshow('Blured Edges', blurred)
# cv2.waitKey(0)

# # Edges
# edges = cv2.Canny(blurred, 100, 200)

# cv2.imshow('Detected Edges', edges)
# cv2.waitKey(0)

# #Computing dx 
# dx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)
# dy = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)

# # Compute the magnitude and angle (this can be useful for visualization)
# magnitude, angle = cv2.cartToPolar(dx, dy, angleInDegrees=True)

# # Normalize to get unit vectors (for visualization)
# normalized_dx = cv2.normalize(dx, None, 0, 10, cv2.NORM_MINMAX)
# normalized_dy = cv2.normalize(dy, None, 0, 10, cv2.NORM_MINMAX)

# cv2.imshow("dx", normalized_dx.astype(np.uint8))
# cv2.imshow("dy", normalized_dy.astype(np.uint8))
# cv2.waitKey(0)

# contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# largest_contour = max(contours, key=cv2.contourArea)

# # Find the CM
# moments = cv2.moments(edges, binaryImage=True)
# cx = int(moments['m10'] / moments['m00'])
# cy = int(moments['m01'] / moments['m00'])

# distance = cv2.pointPolygonTest(largest_contour, (cx, cy), False)
# if distance < 0:
#     print("The center of mass is outside the object.")
# else:
#     print("The center of mass is inside the object.")

# inside_object = cv2.pointPolygonTest(largest_contour, (cx, cy), False) >= 0

# color_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
# cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)  # Center of mass in red
# cv2.imshow('Center of Mass', color_image)
# cv2.waitKey(0)


# # Find the closest points to CM
# y, x = np.where(edges == 255) # white points on the image
# distances = np.sqrt((x - cx)*2 + (y - cy)*2) # distance 
# closest_idx = np.argmin(distances) #min value
# closest_point = (x[closest_idx], y[closest_idx])
# print(closest_point)

# y_coord, x_coord = np.where(edges == 255)
# slopes = {}

# for i in range(len(x_coord)):
#     delta_x = x_coord[i] - cx
#     delta_y = y_coord[i] - cy
    
#     if delta_x != 0:
#         slope = delta_y / delta_x
#     else:
#         slope = np.inf

#     coord = (x_coord[i], y_coord[i])
#     slopes[coord] = slope

# opposite_point=[]
# for i in slopes:
#     for j in slopes:
#         if abs(slopes[j] - slopes[i]) < 0.01 and j != i:
#             opposite_point.append([i,j])

# #print(opposite_point)
# cv2.circle(color_image, (closest_point[0], closest_point[1]), 5, (0, 255, 0), -1)  
# #cv2.circle(color_image, (opposite_point[0], opposite_point[1]), 5, (0, 255, 0), -1)  
# cv2.circle(color_image, (opposite_point[0],opposite_point[1]), 5, (0, 255, 0), -1)  


# cv2.namedWindow('Closest point')
# cv2.setMouseCallback('Closest point', click_event, color_image)
# cv2.imshow('Closest point', color_image)
# cv2.waitKey(0)
# cv2.imshow('Closest point', color_image)
# cv2.waitKey(0)

import cv2
import numpy as np

def preprocess_image(image_path):
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    blurred = cv2.GaussianBlur(image, (17, 17), 0)
    edges = cv2.Canny(blurred, 100, 200)
    cv2.imshow('Detected Edges', edges)
    cv2.waitKey(0)
    return edges, blurred

def compute_gradients(edges):
    dx = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)
    dy = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)
    magnitude, angle = cv2.cartToPolar(dx, dy, angleInDegrees=True)
    normalized_dx = cv2.normalize(dx, None, 0, 10, cv2.NORM_MINMAX)
    normalized_dy = cv2.normalize(dy, None, 0, 10, cv2.NORM_MINMAX)
    cv2.imshow("dx", normalized_dx.astype(np.uint8))
    cv2.imshow("dy", normalized_dy.astype(np.uint8))
    cv2.waitKey(0)
    return normalized_dx, normalized_dy

def compute_center_of_mass(edges):
    moments = cv2.moments(edges, binaryImage=True)
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    return cx, cy

def show_center_of_mass(edges, cx, cy):
    color_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
    cv2.imshow('Center of Mass', color_image)
    cv2.waitKey(0)

def compute_slopes_from_cm(edges, cx, cy):
    y_coord, x_coord = np.where(edges == 255)
    slopes = {}
    for i in range(len(x_coord)):
        delta_x = x_coord[i] - cx
        delta_y = y_coord[i] - cy
        slope = delta_y / delta_x if delta_x != 0 else np.inf
        coord = (x_coord[i], y_coord[i])
        slopes[coord] = slope
    return slopes

def find_opposite_points(slopes, threshold=0.02):
    pairs = []
    seen_pairs = set()
    for key1, value1 in slopes.items():
        for key2, value2 in slopes.items():
            if key1 != key2 and abs(value1 - value2) < threshold:
                pair = sorted([key1, key2])
                pair_tuple = tuple(pair)
                if pair_tuple not in seen_pairs:
                    pairs.append(pair)
                    seen_pairs.add(pair_tuple)
    return pairs

def show_points_on_image(edges, points, color=(0, 255, 0)):
    color_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    for point in points:
        cv2.circle(color_image, point, 5, color, -1)
    cv2.imshow('Marked Points', color_image)
    cv2.waitKey(0)

def main():
    image_path = 'src/pcl_sampling/scripts/object.jpeg'
    edges, blurred = preprocess_image(image_path)
    
    normalized_dx, normalized_dy = compute_gradients(edges)
    
    cx, cy = compute_center_of_mass(edges)
    show_center_of_mass(edges, cx, cy)

    slopes = compute_slopes_from_cm(edges, cx, cy)
    
    opposite_points_pairs = find_opposite_points(slopes)
    print(opposite_points_pairs)
    
    # Assuming you want to show all opposite point pairs on the image
    all_points = [item for sublist in opposite_points_pairs for item in sublist]
    show_points_on_image(edges, all_points)

if __name__ == "__main__":
    main()