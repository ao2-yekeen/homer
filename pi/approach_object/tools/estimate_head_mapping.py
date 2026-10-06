"""Offline image-based head rotation estimate; never commands hardware.

Assumes shared square-pixel intrinsics and a centred principal point. Translation,
distortion and mechanical hysteresis can bias the result. Not motion calibration.
"""
import argparse, json
from pathlib import Path
import cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input',required=True,help='Directory with capture.json and view-COMMAND.png')
args=parser.parse_args()
root=Path(args.input)
records=json.loads((root/'capture.json').read_text())
angles=[int(r['servo_command_deg']) for r in records]
if len(angles)<3 or len(set(angles))!=len(angles) or 120 not in angles:
 raise ValueError('need at least three distinct commands including reference 120')
images=[cv2.imread(str(root/f'view-{s}.png'),0) for s in angles]
if any(a is None or a.shape!=(480,640) for a in images):
 raise ValueError('this diagnostic requires 640x480 images')
sift=cv2.SIFT_create(nfeatures=3500)
features=[sift.detectAndCompute(a,None) for a in images]
pairs=[]
for i in range(len(angles)-1):
 k1,d1=features[i];k2,d2=features[i+1]
 matches=cv2.BFMatcher().knnMatch(d1,d2,k=2)
 good=[a for a,b in matches if a.distance<.7*b.distance]
 a=np.float64([k1[m.queryIdx].pt for m in good]);b=np.float64([k2[m.trainIdx].pt for m in good])
 H,mask=cv2.findHomography(a,b,cv2.USAC_MAGSAC,2.5)
 if H is None or mask.sum()<30:raise RuntimeError(f'insufficient matches {angles[i]} {len(good)}')
 a=a[mask.ravel()>0];b=b[mask.ravel()>0]
 pairs.append((a,b,H))
 print(angles[i],angles[i+1],'inliers',len(a),flush=True)
def K(f):return np.array([[f,0,319.5],[0,f,239.5],[0,0,1.]])
init=[np.log(400.)]
for a,b,H in pairs:
 M=np.linalg.inv(K(400))@H@K(400)
 if np.linalg.det(M)<0:M=-M
 U,_,V=np.linalg.svd(M);R=U@V
 init.extend(Rotation.from_matrix(R).as_rotvec())
def residual(x,sel=None):
 f=np.exp(x[0]);km=K(f);ki=np.linalg.inv(km);errs=[]
 for i,(a,b,H) in enumerate(pairs):
  if sel is not None:a,b=a[sel[i]],b[sel[i]]
  rays=np.column_stack([a,np.ones(len(a))])@ki.T
  rays=rays@Rotation.from_rotvec(x[1+3*i:4+3*i]).as_matrix().T
  proj=rays@km.T;uv=proj[:,:2]/proj[:,2,None]
  errs.extend((uv-b).ravel())
 return np.array(errs)
fit=least_squares(residual,init,loss='soft_l1',f_scale=1.,max_nfev=150)
f=np.exp(fit.x[0]);deltas=[];metrics=[]
for i,(a,b,H) in enumerate(pairs):
 R=Rotation.from_rotvec(fit.x[1+3*i:4+3*i])
 rv=np.degrees(R.as_rotvec());delta=rv[0];deltas.append(delta)
 rays=np.column_stack([a,np.ones(len(a))])@np.linalg.inv(K(f)).T@R.as_matrix().T
 proj=rays@K(f).T;err=np.linalg.norm(proj[:,:2]/proj[:,2,None]-b,axis=1)
 metrics.append(dict(commands=[angles[i],angles[i+1]],inliers=len(a),rotation_vector_deg=rv.tolist(),median_reprojection_px=float(np.median(err)),p95_reprojection_px=float(np.percentile(err,95))))
transforms=[np.eye(3)]
for i in range(len(pairs)):
 transforms.append(Rotation.from_rotvec(fit.x[1+3*i:4+3*i]).as_matrix()@transforms[-1])
reference=transforms[angles.index(120)]
pitch=[]
for transform in transforms:
 axis=reference@transform.T@np.array([0.,0.,1.])
 pitch.append(float(np.degrees(np.arctan2(axis[1],np.hypot(axis[0],axis[2])))))
result=dict(method='SIFT homography inlier selection; joint robust pure-rotation fit with shared square-pixel focal and centred principal point',reference='operator startup approximately horizontal at command 120; absolute offset unvalidated',angle_method='compose fitted rotations; optical-axis elevation relative to reference camera frame',focal_px=float(f),horizontal_fov_deg=float(np.degrees(2*np.arctan(640/(2*f)))),pitch_relative_to_120_deg=dict(zip(map(str,angles),pitch)),mast_view_angle_est_deg=dict(zip(map(str,angles),[90-p for p in pitch])),pairs=metrics,qualified=False,enable_motion=False)
(root/'mapping.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
