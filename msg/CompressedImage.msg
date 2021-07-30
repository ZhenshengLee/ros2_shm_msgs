# This message contains a compressed image.

shm_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image

shm_msgs/String format                # Specifies the format of the data
                             #   Acceptable values:
                             #     jpeg, png

uint8 size

uint8[] data                 # Compressed image buffer

uint8 MAX_SIZE=255