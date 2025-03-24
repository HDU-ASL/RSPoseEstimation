import numpy as np

def getCameraIntrinsicsFromTxt(camerapath):
    f=open(camerapath, encoding='utf-8')
    txt=[]
    for line in f:
        outline = line.strip()
        outline = outline.split(' ')
        txt.append(outline)

    txt = np.array(txt)
    return txt[:,4:8]
