#https://github.com/llSourcell/How_to_simulate_a_self_driving_car/blob/master/utils.py

import cv2, os, random
import numpy as np
import matplotlib.image as mpimg

#Rosey2
IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 66, 200, 3
INPUT_SHAPE = (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS)

def load_image(data_dir, image_file):
    """
    Load RGB images from a file
    """
    return mpimg.imread(os.path.join(data_dir, image_file.strip()))


def crop(image):
    """
    Crop the image (removing the sky at the top and the car front at the bottom)
    """
    return image[220:-1, :, :] # remove the sky


def resize(image):
    """
    Resize the image to the input shape used by the network model
    """
    return cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT), cv2.INTER_AREA)


def rgb2yuv(image):
    """
    Convert the image from RGB to YUV (This is what the NVIDIA model does)
    """
    return cv2.cvtColor(image, cv2.COLOR_RGB2YUV)


def preprocess(image):
    """
    Combine all preprocess functions into one
    """
    image = crop(image)
    image = resize(image)
    image = rgb2yuv(image)
    return image


def choose_image(data_dir, img_path, steering_angle):
    return load_image(data_dir, img_path), steering_angle


def random_flip(image, steering_angle):
    if np.random.rand() < 0.5:
        image = cv2.flip(image, 1)
        steering_angle = -steering_angle
    return image, steering_angle


def random_translate(image, steering_angle, range_x, range_y):
    """
    Randomly shift the image virtially and horizontally (translation).
    """
    trans_x = range_x * (np.random.rand() - 0.5)
    trans_y = range_y * (np.random.rand() - 0.5)
    steering_angle += trans_x * 0.002
    trans_m = np.float32([[1, 0, trans_x], [0, 1, trans_y]])
    height, width = image.shape[:2]
    image = cv2.warpAffine(image, trans_m, (width, height))
    return image, steering_angle


def random_shadow(image):
    """
    Generates and adds random shadow
    """
    # (x1, y1) and (x2, y2) forms a line
    # xm, ym gives all the locations of the image
    height, width, channels = image.shape
    x1, y1 = width * np.random.rand(), 0
    x2, y2 = width * np.random.rand(), height
    xm, ym = np.mgrid[0:height, 0:width]

    # mathematically speaking, we want to set 1 below the line and zero otherwise
    # Our coordinate is up side down.  So, the above the line:
    # (ym-y1)/(xm-x1) > (y2-y1)/(x2-x1)
    # as x2 == x1 causes zero-division problem, we'll write it in the below form:
    # (ym-y1)*(x2-x1) - (y2-y1)*(xm-x1) > 0
    mask = np.zeros_like(image[:, :, 1])
    mask[(ym - y1) * (x2 - x1) - (y2 - y1) * (xm - x1) > 0] = 1

    # choose which side should have shadow and adjust saturation
    cond = mask == np.random.randint(2)
    s_ratio = np.random.uniform(low=0.2, high=0.5)

    # adjust Saturation in HLS(Hue, Light, Saturation)
    hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    hls[:, :, 1][cond] = hls[:, :, 1][cond] * s_ratio
    return cv2.cvtColor(hls, cv2.COLOR_HLS2RGB)


def random_brightness(image):
    """
    Randomly adjust brightness of the image.
    """
    # HSV (Hue, Saturation, Value) is also called HSB ('B' for Brightness).
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    ratio = 1.0 + 0.4 * (np.random.rand() - 0.5)
    hsv[:,:,2] =  hsv[:,:,2] * ratio
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)


def augument(data_dir, img_path, steering_angle, range_x=100, range_y=10):
    """
    Generate an augumented image and adjust steering angle.
    (The steering angle is associated with the image)
    """
    image, steering_angle = choose_image(data_dir, img_path, steering_angle)
    image, steering_angle = random_flip(image, steering_angle)
    image, steering_angle = random_translate(image, steering_angle, range_x, range_y)
    image = random_shadow(image)
    image = random_brightness(image)

    return image, steering_angle


def batch_generator(data_dir, datasets, image_paths, steering_angles, batch_size, is_training):
    """
    Generate training image give image paths and associated steering angles
    """
    images = np.empty([batch_size, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS])
    steers = np.empty(batch_size)
    while True:
        i = 0
        dataset_index = random.randint(0, len(datasets)-1)
        for index in np.random.permutation(len(image_paths[dataset_index])):
            img = image_paths[dataset_index][index]
            steering_angle = steering_angles[dataset_index][index]
            # argumentation
            if is_training and np.random.rand() < 0.6:
                image, steering_angle = augument(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "training_set"), img, steering_angle)
            elif is_training:
                image = load_image(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "training_set"), img)
            else:
                image = load_image(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "test_set"), img)
            # add the image and steering angle to the batch
            images[i] = preprocess(image)
            steers[i] = steering_angle
            i += 1
            if i == batch_size:
                break
        yield images, steers

def fat_npy_builder(data_dir, datasets, image_paths_training, steering_angles_training, image_paths_test, steering_angles_test, num_stacked_images, total_size=20000):
    """
    Generate two numpy arrays that contain `total_size` number of images and associated steering angles
    """
    #image with attached steering value occupies channels 0-2
    images = np.empty([total_size, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS*num_stacked_images])
    steers = np.empty(total_size)
    #Get RAM information for usage prediction
    ram = psutil.virtual_memory()
    initial_ram_usage = ram.used
    for i in range(total_size):
        if np.random.rand() < 0.8: #add set from training data
            dataset_index = random.randint(0, len(datasets)-1)
            index = np.random.randint(0,len(image_paths_training[dataset_index]))
            if index < num_stacked_images-1:
                index = num_stacked_images-1
            #get the num_stacked_images file paths to feed into the network
            imgs = []
            for z in range(num_stacked_images):
                imgs.append(image_paths_training[dataset_index][index-z])
            steering_angle = steering_angles_training[dataset_index][index]
            # argumentation
            if np.random.rand() < 0.6:
                imgs, steering_angle = augument(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "training_set"), imgs, steering_angle)
            else:
                imgs = load_images(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "training_set"), imgs)
        else: #add set from test data
            dataset_index = random.randint(0, len(datasets)-1)
            index = np.random.randint(0,len(image_paths_test[dataset_index]))
            if index < num_stacked_images-1:
                index = num_stacked_images-1
            #get the num_stacked_images file paths to feed into the network
            imgs = []
            for z in range(num_stacked_images):
                imgs.append(image_paths_test[dataset_index][index-z])
            steering_angle = steering_angles_test[dataset_index][index]
            # argumentation
            if np.random.rand() < 0.6:
                imgs, steering_angle = augument(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "test_set"), imgs, steering_angle)
            else:
                imgs = load_images(os.path.join(os.path.join(data_dir, datasets[dataset_index]), "test_set"), imgs)

        # add the image and steering angle to the batch
        images[i] = preprocess(imgs, num_stacked_images)
        steers[i] = steering_angle
        ram = psutil.virtual_memory()
        #print("Loading number: " + str(i) + "/" + str(total_size), end="  ")
        #Note the predicted RAM usage is a very ruff estimate and depends on other programs running on your machine. For greatest accuraccy do not run any other programs or open any new applicaitons while computing estimate
        #print("Total predicted RAM usage: %.3f/%.3fGB"%((total_size*(ram.used-initial_ram_usage)/(i+1))/1000000000,(ram.total-initial_ram_usage)/1000000000), end="  ")
        #print(str(ram.percent) + "%", end="\r")
    return images, steers
