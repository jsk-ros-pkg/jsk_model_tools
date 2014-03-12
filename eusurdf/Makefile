all: models/full_pr2.urdf

models/full_pr2.urdf:
	mkdir -p models
	rosrun xacro xacro.py `rospack find pr2_description`/robots/pr2.urdf.xacro -o models/full_pr2.urdf

clean:
	- rm models/full_pr2.sxml

wipe: clean
