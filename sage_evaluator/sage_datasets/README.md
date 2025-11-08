# SAGE Datasets

This package provides convenient access to scene data used in evaluation and exploration.

## Directory structure
/app/src/sage_evaluator/sage_datasets/matterport_isaac/
    ├── 00809-Qpor2mEya8F/
    │   └── annotations/
    │       ├── v1.0/
    │       └── v1.1/
    └── ...

## Configuration
The dataset root path is defined in:
`sage_datasets/utils.py`  
Change the line:
```python
DATASET_ROOT = "/app/src/sage_evaluator/sage_datasets/matterport_isaac"
```

if your data resides in a different location.

## Usage
```python
from sage_datasets.utils import DatasetManager
ds = DatasetManager("00809-Qpor2mEya8F", "v1.1")
print(ds.map())
print(ds.pose())
```