Some useful info:
Script usage:
- To download the entire Matterport3D release (1.3TB): download-mp.py -o [directory in which to download] 
- To download a specific scan (e.g., 17DRP5sb8fy): download-mp.py -o [directory in which to download] --id 17DRP5sb8fy
- To download a specific file type (e.g., *.sens, valid file suffixes listed here): download-mp.py -o [directory in which to download] --type .sens
- *.sens files can be read using the sens-File reader (it's a bit easier to handle than a larger number of separate image files)

License: Matterport3D data is released under the Terms of Use. By downloading the data, you agree to these terms!

# Download command
```
python3 matterport_download.py -o ./matterport_data --id <scan>
```