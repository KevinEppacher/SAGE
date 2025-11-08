from sage_datasets.utils import DatasetManager

def main(args=None):
    ds = DatasetManager("00809-Qpor2mEya8F", version="v1.1")
    print("Annotations:", ds.annotations_dir)
    print("Map YAML:", ds.map())
    print("Pose:", ds.pose())
    print("Pointcloud:", ds.pointcloud())
    print("Output dir:", ds.output_dir())

if __name__ == "__main__":
    main()


