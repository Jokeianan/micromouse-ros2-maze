"""Check exported occupancy orientation and metadata without Gazebo."""
import json
from pathlib import Path
import tempfile
from nav_msgs.msg import OccupancyGrid
from micromouse_practice.map_recorder import save_grid

m=OccupancyGrid()
m.header.frame_id='map'
m.info.width=2;m.info.height=2;m.info.resolution=.04
m.info.origin.position.x=-1.2;m.info.origin.position.y=2.4
m.info.origin.orientation.w=1.
m.data=[-1,0,100,-1]
with tempfile.TemporaryDirectory() as tmp:
    p=Path(tmp);save_grid(m,p,{'source':'unit-test'})
    assert (p/'map.pgm').read_bytes()==b'P5\n2 2\n255\n'+bytes([0,205,205,254])
    data=json.loads((p/'map.json').read_text())
    assert data['origin']==[-1.2,2.4,0.] and data['data']==[-1,0,100,-1]
    assert 'origin: [-1.2, 2.4, 0.0]' in (p/'map.yaml').read_text()
print('Map export orientation, unknown cells, and origin metadata: PASS')
