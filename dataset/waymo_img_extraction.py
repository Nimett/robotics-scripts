import os
import tensorflow as tf
from waymo_open_dataset import dataset_pb2 as open_dataset


def process_tfrecord_files(input_dir, output_dir, camera_name=open_dataset.CameraName.FRONT):
    tfrecord_files = [os.path.join(input_dir, f) for f in os.listdir(input_dir) if f.endswith(".tfrecord")]
    os.makedirs(output_dir, exist_ok=True)
  
    for tfrecord_file in tfrecord_files:
        dataset = tf.data.TFRecordDataset(tfrecord_file)
      
        for data in dataset:
            frame = open_dataset.Frame()
            frame.ParseFromString(data.numpy())
            timestamp = frame.timestamp_micros
          
            for image in frame.images:
                if image.name == camera_name:
                    segment_id = tfrecord_file.split("/")[-1].replace("_with_camera_labels.tfrecord", "").replace("segment-", "")
                    filename = f"{timestamp}#{segment_id}.jpg"
                    output_path = os.path.join(output_dir, filename)
                    with open(output_path, "wb") as img_file:
                        img_file.write(image.image)
                    break


if __name__ == "__main__":
    input_dir = "path/to/tfrecord/directory"
    output_dir = "path/to/output/directory"
    process_tfrecord_files(input_dir, output_dir)
