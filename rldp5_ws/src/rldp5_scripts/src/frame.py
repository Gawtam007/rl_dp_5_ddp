import cv2
import os

def video_to_frames(video_path, output_folder):
    # Open the video file
    video_capture = cv2.VideoCapture(video_path)
    if not video_capture.isOpened():
        print("Error: Could not open the video file.")
        return
    # Create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)

    # Read the video frames
    frame_count = 0
    while True:
        # Read the next frame
        success, frame = video_capture.read()
        if not success:
            break

        # Save the frame as an image file
        if frame_count%8 == 0:
            frame_filename = os.path.join(output_folder, f"frame_{frame_count:04d}.jpg")
            cv2.imwrite(frame_filename, frame)

        frame_count += 1

    # Release the video capture object
    video_capture.release()

    print(f"Frames extracted: {frame_count}")
    print(f"Frames saved in: {output_folder}")

# Example usage
video_path = '/home/acer/Downloads/close_crg.mp4'
output_folder = '/home/acer/Downloads/close_crg_frames'
video_to_frames(video_path, output_folder)