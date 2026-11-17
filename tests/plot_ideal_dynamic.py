import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
import sys,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
s=Path(sys.argv[1]);m=np.load(s/'motion.npz');t0=float(m['insertion_time_s']);a=m['trajectory'];a=a[a[:,0]>=t0];t=a[:,0]-t0;o=np.c_[np.full(len(a),1.1),-1.+.1*np.minimum(t,16.)];heading=np.c_[np.cos(a[:,3]),np.sin(a[:,3])];c=np.stack([a[:,1:3]+.18*heading,a[:,1:3]-.18*heading],axis=1);g=np.linalg.norm(np.maximum(abs(c-o[:,None,:])-[.2,.25],0),axis=2)-.25;i,j=np.unravel_index(g.argmin(),g.shape)
fig,ax=plt.subplots(figsize=(9,6));ax.plot(a[:,1],a[:,2],c='#267d69',lw=2,label='Robot center (ideal cmd_vel execution)');ax.plot(o[:,0],o[:,1],c='#c68a3b',ls='--',lw=2,label='Box center path (different times)');ax.add_patch(Rectangle(o[i]-[.2,.25],.4,.5,color='#dc9b4b',alpha=.35,label='Box at closest approach'))
for center in c[i]:ax.add_patch(Circle(center,.25,edgecolor='#267d69',facecolor='#267d69',alpha=.2))
ax.scatter([a[i,1]],[a[i,2]],s=25,c='#267d69');ax.annotate(f'Minimum footprint gap: {g[i,j]*100:.1f} cm\nt = {t[i]:.2f} s after insertion',(a[i,1],a[i,2]),xytext=(.03,.92),textcoords='axes fraction',arrowprops=dict(arrowstyle='->',color='#51606a'),fontsize=10)
ax.set(xlabel='Map X / m',ylabel='Map Y / m',title='Moving obstacle: ROS integration test (not Isaac / hardware)');ax.axis('equal');ax.grid(alpha=.15);ax.legend(loc='lower right',fontsize=9);fig.tight_layout();fig.savefig(s/'移动障碍对照测试.png',dpi=170)
