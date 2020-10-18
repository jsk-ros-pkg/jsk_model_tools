# eusurdfwrl

## How to Build

First, install simtrans
```
git clone https://github.com/fkanehiro/simtrans
cd simtrans
sudo pip install -r requirements.txt
sudo python setup.py install
```

Then, build eusurdfwrl
```
rosdep install -r --from-paths . --ignore-src -y
catkin build eusurdfwrl
```

## Conversion

eusurdf/models/\*/model.urdf -> eusurdfwrl/models/\*.wrl

eusurdf/worlds/\*.world -> eusurdfwrl/worlds/\*.yaml (loadable by hrpsys_choreonoid/launch/add_objects.py)