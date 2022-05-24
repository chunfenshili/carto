import rosbag
import sys
import subprocess

def extract_chunks(file_in, chunks):
    bagfile = rosbag.Bag(file_in)
    messages = bagfile.get_message_count()
    m_per_chunk = int(round(float(messages) / float(chunks)))
    chunk = 0
    m = 0
    outbag = rosbag.Bag("_%04d.bag" % chunk, 'w')
    for topic, msg, t in bagfile.read_messages():
        m += 1
        if m % m_per_chunk == 0:
            outbag.close()
            chunk += 1
            outbag = rosbag.Bag(file_in+"_%04d.bag" % chunk, 'w')
        outbag.write(topic, msg, t)
    outbag.close()


path = sys.argv[1]
num = sys.argv[2]
print("Spliting rosbag: ", path)
print("Divided into number of bags: ", num)
# extract_chunks(path, num)

for i in range(1, int(num)+1):
    process_bag = "bag_filenames:=" + path +"_%04d.bag" % i
    print("processing bag: ", process_bag)
    pbstream_file = "pose_graph_filename:=" + path +".pbstream"
    print("processing pbstream: ", pbstream_file)
    subprocess.run(["roslaunch", "cartographer_ros", "assets_writer_backpack_3d_imetai.launch", process_bag, pbstream_file])
