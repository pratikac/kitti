This is a data loader for KITTI. Download the KITTI dataset from

[KITTI 2012](http://www.cvlibs.net/download.php?file=data_stereo_flow.zip) and
[KITTI 2015](http://www.cvlibs.net/download.php?file=data_scene_flow.zip).

Run the loader as 
```
local kitti = require 'kitti'

local train, val, test = kitti.kitti12.split()
```
