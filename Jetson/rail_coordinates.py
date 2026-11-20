# -*- coding: utf-8 -*-
"""同帧球冠二维点 -> 参考轨道轴；不拥有相机、舵机或UART。"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from measurement_contract import LOCAL_MEASUREMENT_VERSIONS
from ruler_reference import similarity_from_points

MEASUREMENT_VERSION='relative-cap-v1'
RAIL_COORDINATE_VERSION='rail-reference-v1'


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode('utf-8')).hexdigest()


def reference_image_context(geometry):
    """raw图可被points模式复用；整帧矫正的参考必须绑定同一K/D和像素投影。"""
    c=geometry.context
    return dict(image_space=geometry.image_space,resolution=c['resolution'],camera_id=c['camera_id'],
                focus_id=c['focus_id'],intrinsics_sha256=c['intrinsics_sha256'] if geometry.image_space=='undistorted_image' else None)


def validate_reference_context(cfg,geometry,required=False):
    ref=cfg.get('rail',{}).get('upper_reference',{})
    if not ref.get('enabled',False):
        if required:raise ValueError('rail coordinates require confirmed upper reference')
        return
    declared=ref.get('image_context')
    if declared is None:
        if required or geometry.image_space!='raw_image':
            raise ValueError('reference image context missing; rebuild or explicitly confirm reference setup')
        return  # 未启用新坐标的旧raw流程保持兼容，不能据此产生新厘米值。
    if declared!=reference_image_context(geometry):
        raise ValueError('reference/camera coordinate context mismatch')


class RailCoordinates(object):
    def __init__(self,cfg,geometry,base_dir=None):
        self.geometry=geometry;self.cfg=cfg.get('rail_coordinates',{})
        self.enabled=bool(self.cfg.get('enabled',False));self.last_frame=None;self.last_reference=None
        self.binding=None;self.binding_id=None;self.calibration=None
        self.root=Path(base_dir or Path(__file__).resolve().parent)
        validate_reference_context(cfg,geometry,self.enabled)
        if not self.enabled:return
        if not geometry.context.get('camera_id') or not geometry.context.get('focus_id'):
            raise ValueError('rail coordinates require explicit camera_id and focus_id')
        self.points=np.asarray([self.cfg[k] for k in ('left_px','origin_px','right_px')],dtype=float)
        if self.points.shape!=(3,2) or not np.isfinite(self.points).all():raise ValueError('invalid rail reference axis')
        w,h=geometry.context['resolution']
        if np.any(self.points<0) or np.any(self.points[:,0]>=w) or np.any(self.points[:,1]>=h):raise ValueError('rail axis outside reference image')
        ref=cfg['rail']['upper_reference'];path=Path(ref['image'])
        if not path.is_absolute():path=self.root/path
        reference_hash=hashlib.sha256(path.read_bytes()).hexdigest()
        if ref.get('sha256')!=reference_hash:raise ValueError('reference image hash mismatch')
        config_ref={k:v for k,v in ref.items() if k!='image'}
        source_names=['ball_detector.py','ball_tracker.py','measurement_contract.py','local_vision.py','ruler_reference.py','rail_detector.py','rail_coordinates.py','camera_geometry.py']
        # 单元测试可用几何替身；实机主入口总是加载实际camera_geometry模块。
        source_hashes={n:hashlib.sha256((Path(__file__).resolve().parent/n).read_bytes()).hexdigest()
                       for n in source_names if (Path(__file__).resolve().parent/n).is_file()}
        # 文件路径不应决定几何：部署到Nano新目录时参考图内容相同即可。
        settings=dict(ball=cfg.get('ball',{}),rail={k:v for k,v in cfg['rail'].items() if k!='upper_reference'},
                      upper_reference=config_ref,tracking=cfg.get('tracking',{}),ball_search=cfg.get('ball_search',{}))
        self.method = cfg.get('ball', {}).get('mode', 'relative_cap')
        self.measurement_version = LOCAL_MEASUREMENT_VERSIONS.get(self.method, MEASUREMENT_VERSION)
        self.binding=dict(schema='rail-binding-v1',measurement_version=self.measurement_version,
                          rail_coordinate_version=RAIL_COORDINATE_VERSION,geometry=geometry.context,
                          reference_image_sha256=reference_hash,reference_config_sha256=digest(dict(reference=config_ref,axis=self.points.tolist())),
                          algorithm_sha256=digest(dict(sources=source_hashes,settings=settings)))
        self.binding_id=digest(self.binding)
        self.axis_points=self.geometry.correct_points(self.points) if geometry.context['mode']=='points' else self.points.copy()
        axis=self.axis_points[2]-self.axis_points[0];norm=float(np.linalg.norm(axis))
        if norm<10:raise ValueError('degenerate reference rail axis')
        self.tangent=axis/norm
        origin_projection=float(np.dot(self.axis_points[1]-self.axis_points[0],self.tangent))
        if not 0<origin_projection<norm:raise ValueError('origin must be between left/right reference points')
        path=self.cfg.get('calibration_file')
        if path:
            from rail_calibration import RailCalibration
            path=Path(path)
            if not path.is_absolute():path=self.root/path
            self.calibration=RailCalibration(str(path),self.binding)

    def process(self,out,candidates,rail,frame_id):
        """每帧返回全新结果；只使用当前候选，不继承上帧厘米或参考坐标。"""
        r=dict(pixel_valid=False,reference_valid=False,calibration_valid=False,cm_valid=False,feedback_valid=False,
               measurement_x_px=None,measurement_y_px=None,reference_x_px=None,reference_y_px=None,rail_s_px=None,
               rail_x_cm=None,rail_angle_deg=None,reference_rotation_deg=None,reference_scale=None,reference_residual_px=None,
               origin_x_px=None,origin_y_px=None,rail_binding_id=self.binding_id,
               measurement_image_space=self.geometry.image_space,rail_coordinate_version=RAIL_COORDINATE_VERSION,
               rail_coordinate_status='disabled' if not self.enabled else 'not_updated')
        if not self.enabled:return r
        if type(frame_id) is not int or (self.last_frame is not None and frame_id<=self.last_frame):
            r['rail_coordinate_status']='stale_frame';return r
        self.last_frame=frame_id
        r['calibration_valid']=bool(self.calibration is not None and self.calibration.ready and self.calibration.validated)
        try:
            x=out.get('x_px');r['pixel_valid']=bool(out.get('valid') and not out.get('predicted') and isinstance(x,(int,float,np.floating)) and math.isfinite(x))
            seq=getattr(rail,'reference_sequence',None)
            if type(seq) is not int or (self.last_reference is not None and seq<=self.last_reference):
                r['rail_coordinate_status']='stale_reference';return r
            self.last_reference=seq
            if not (rail.valid and rail.geometry_valid and rail.identity_valid):
                r['rail_coordinate_status']='reference_invalid';return r
            inverse=np.asarray(rail.reference_inverse,dtype=float);translation=np.asarray(rail.reference_translation,dtype=float)
            if inverse.shape!=(2,2) or translation.shape!=(2,) or not np.isfinite(inverse).all() or not np.isfinite(translation).all():
                r['rail_coordinate_status']='reference_invalid';return r
            # 当前画面轨道方向和零点用于日志；不是机械倾角，不反向驱动舵机。
            current=self.points.dot(np.linalg.inv(inverse).T)+translation
            direction=current[2]-current[0]
            r.update(origin_x_px=float(current[1,0]),origin_y_px=float(current[1,1]),rail_angle_deg=math.degrees(math.atan2(direction[1],direction[0])))
            registration=rail.registration
            if self.geometry.context['mode']=='points':
                before=self.geometry.correct_points(registration['reference_points'])
                after=self.geometry.correct_points(registration['current_points'])
                transform,registration=similarity_from_points(before,after)
                if transform is None:
                    r['rail_coordinate_status']='corrected_reference_'+registration['reason'];return r
                inverse,translation=transform
            r.update(reference_valid=True,reference_rotation_deg=registration.get('image_rotation_deg'),
                     reference_scale=registration.get('scale'),reference_residual_px=registration.get('residual_px'))
            if not r['pixel_valid']:
                r['rail_coordinate_status']='pixel_invalid';return r
            if len(candidates)!=1:
                r['rail_coordinate_status']='candidate_ambiguous';return r
            c=candidates[0];extra=getattr(c,'extra',{})
            if (c.x!=x or c.method!=out.get('method') or c.method!=self.method or
                    extra.get('coordinate_version')!=self.measurement_version or extra.get('identity_valid') is not True or
                    extra.get('in_search_roi') is not True):
                r['rail_coordinate_status']='measurement_contract';return r
            y=extra.get('measurement_y_px');w,h=self.geometry.context['resolution']
            if not isinstance(y,(int,float,np.floating)) or not math.isfinite(y) or not 0<=x<w or not 0<=y<h:
                r['rail_coordinate_status']='measurement_xy_invalid';return r
            r.update(measurement_x_px=float(x),measurement_y_px=float(y))
            p=np.array([[x,y]],dtype=float)
            if self.geometry.context['mode']=='points':p=self.geometry.correct_points(p)
            reference=inverse.dot(p[0]-translation)
            s=float(np.dot(reference-self.axis_points[1],self.tangent))
            if not np.isfinite(reference).all() or not math.isfinite(s):
                r['rail_coordinate_status']='nonfinite_reference_position';return r
            r.update(reference_x_px=float(reference[0]),reference_y_px=float(reference[1]),rail_s_px=s,
                     rail_coordinate_status='calibration_unavailable')
            if self.calibration is not None:
                cm=self.calibration.position(s)
                if cm is not None and math.isfinite(cm):r.update(rail_x_cm=float(cm),cm_valid=True,rail_coordinate_status='ok')
                else:r['rail_coordinate_status']=('outside_calibrated_range' if r['calibration_valid'] else getattr(self.calibration,'status','calibration_invalid'))
            return r
        except (AttributeError,KeyError,TypeError,ValueError,np.linalg.LinAlgError):
            r.update(reference_valid=False,cm_valid=False,rail_x_cm=None,rail_s_px=None,rail_coordinate_status='coordinate_invalid')
            return r
