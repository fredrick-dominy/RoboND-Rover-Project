import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = \
        (img[:, :, 0] > rgb_thresh[0]) \
        & (img[:, :, 1] > rgb_thresh[1]) \
        & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def obstacle_thresh(img):
    # invert the navigable thresh
    # since the image being passed in is 0 or 1 out of 255
    # I needed to invert the values so that 0 = 1 and 1 = 0
    # subtracting by 2 from each number prior to inverting made the values 254 and 255
    # when inverted the numbers became 1 and 0
    obstacle_select = np.invert(img - 2)
    return obstacle_select


def blur_mask(img):
    # bilateralFilter( src, d )
    # d = Diameter of each pixel neighborhood
    # that is used during filtering. If it is
    # non-positive, it is computed from sigmaSpace.
    return cv2.bilateralFilter(img, 5, 75, 75)


def rock_thresh(img):
    lower_gold = np.array([110, 110, 0])
    upper_gold = np.array([255, 255, 90])
    mask = cv2.inRange(img, lower_gold, upper_gold)
    return mask


# Define a function to convert from image coordinates to rover coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world


# Define a function to perform a perspective transform
def perspective_transform(img, src, dst):
    perspective_transformation_mapping = cv2.getPerspectiveTransform(src, dst)
    warped_image = cv2.warpPerspective(img, perspective_transformation_mapping,
                                       (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped_image


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(rover):
    # Perform perception steps to update rover()
    # NOTE: camera image is coming to you in rover.img
    # 1) Define source and destination points for perspective transform
    image = rover.img
    dst_size = 5
    bottom_offset = 6

    source = np.float32([
        [14, 140],
        [301, 140],
        [200, 96],
        [118, 96]
    ])

    destination = np.float32([
        [image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
        [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
        [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
        [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset]
    ])

    # 2) Apply perspective transform
    nav_perspective_transform = perspective_transform(image, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable_color_threshold = color_thresh(nav_perspective_transform)

    # navigable_blurred_image = blur_mask(navigable_color_threshold)
    obstacle_image = obstacle_thresh(navigable_color_threshold)
    rock_image = rock_thresh(nav_perspective_transform)

    # 4) Update rover.vision_image (this will be displayed on left side of screen)
    rover.vision_image[:, :, 0] = obstacle_image * 255
    rover.vision_image[:, :, 1] = rock_image  # no need to mult by 255
    rover.vision_image[:, :, 2] = navigable_color_threshold * 255

    # 5) Convert map image pixel values to rover-centric coords
    navigable_x, navigable_y = rover_coords(navigable_color_threshold)
    obstacles_x, obstacles_y = rover_coords(obstacle_image)
    rocks_x, rocks_y = rover_coords(rock_image)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    world_size = 200

    obstacles_x_world, obstacles_y_world = pix_to_world(
        obstacles_x,
        obstacles_y,
        rover.pos[0],
        rover.pos[1],
        rover.yaw,
        world_size,
        scale
    )

    rocks_x_world, rocks_y_world = pix_to_world(
        rocks_x,
        rocks_y,
        rover.pos[0],
        rover.pos[1],
        rover.yaw,
        world_size,
        scale
    )

    navigable_x_world, navigable_y_world = pix_to_world(
        navigable_x,
        navigable_y,
        rover.pos[0],
        rover.pos[1],
        rover.yaw,
        world_size,
        scale
    )

    # 7) Update rover worldmap (to be displayed on right side of screen)
    rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
    rover.worldmap[rocks_y_world, rocks_x_world, 1] += 1
    rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update rover pixel distances and angles
    # rover.nav_dists = rover_centric_pixel_distances
    # rover.nav_angles = rover_centric_angles
    rover.nav_dists, rover.nav_angles = to_polar_coords(navigable_x, navigable_y)
    rover.rock_dists, rover.rock_angles = to_polar_coords(rocks_x, rocks_y)

    return rover
