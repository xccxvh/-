from replay import *
import inspect
# 在现行验证器的 continue 分支采集拒绝原因，仅监控所选帧。
# 行号相对函数开头计算，并校验源码文本，避免后续修改造成静默错标。
source,start=inspect.getsourcelines(lv._verify_topview_circles)
reasons=['proposal_outside_roi','rail_height','patch_border','rim_support',
         'fit_offset','radius_or_residual','fit_outside_roi','rail_support','highlight']
reject=[start+k for k,line in enumerate(source) if line.strip()=='continue']
assert len(reject)==len(reasons)
events=dict(zip(reject,reasons))
events[start+next(i for i,s in enumerate(source) if 'verified.append' in s)]='verified'
gap_x=[262,255,249,245,243,244,246,249,257,266,274,285,296,308,322,335,352,366,379,392,405,415,427]
fast_x=[280,299,316,335,359,380,404,427,451,478,498,518,533,549,561,572,582,594]
targets=dict(enumerate(gap_x,482))
targets.update(enumerate(fast_x,662))
records=[]
def trace(frame,event,arg):
    if frame.f_code is not lv._verify_topview_circles.__code__:return None
    if event=='line' and frame.f_lineno in events:
        v=frame.f_locals
        x=float(v['left']+v['x'])
        if abs(x-targets[i])<40:
            records.append(dict(frame=i,visual_x_approx=targets[i],min_distance=frame.f_back.f_locals['distance'],
                                proposal_x=x,proposal_y=float(v['top']+v['y']),radius=float(v['radius']),
                                stage=events[frame.f_lineno]))
    return trace
rd,bd=dets()
for i,t,j in stream('live_pixel'):
    if i not in targets:continue
    f=cv2.imdecode(j,1)
    g=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY)
    r=rd.detect(g,force=True)
    sys.settrace(trace)
    try:bd.detect(f,g,r)
    finally:sys.settrace(None)
write_csv(dst/'proposal_trace.csv',records)
