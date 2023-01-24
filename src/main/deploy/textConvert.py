import json, math

filename = "mikabad.txt"
f = open(filename, "r")
s = f.readlines()
f.close()
poses = []
for i in s:
  x = i.split(",")
  pose = []
  for a in range(3):
    pose.append(x[a])
  poses.append(pose)

waypoints = []
prev = {}

for i in range(len(poses)):
  dict = {}
  anchor = {"x": float(poses[i][0]), "y": float(poses[i][1])}
  dict["anchorPoint"] = anchor
  if (i == 0):
    dict["prevControl"] = None
  else:
    dict["prevControl"] = prev
  prev = anchor

  if (i == len(poses) - 1):
    dict["nextControl"] = None
  else:
    dict["nextControl"] = {"x": float(poses[i + 1][0]), "y": float(poses[i + 1][1])}

  dict["holonomicAngle"] = float(poses[i][2])
  dict["isReversal"] = False
  dict["velOverride"] = None
  dict["isLocked"] = False
  waypoints.append(dict)

path = {"waypoints": waypoints}

file = json.dumps(path)

with open("aflack.path", "w") as outfile:
  outfile.write(file)