"""电脑端数值实验；读取模拟数组，输出曲线和 CSV。单位在表头显式标注。"""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
DT = 1/300.  # 十个积分步对应一帧 30 Hz 合成视觉测量
B, C = 12., .8  # 假设单位 cm/s²/deg、1/s；正角使球沿轨道 +x 加速
G = 9.80665


def save(path, header, rows):
    with path.open('w', newline='') as f:
        w = csv.writer(f, lineterminator='\n')
        w.writerow(header.split())
        w.writerows(rows)


def advance(x, v, theta):
    """步内角度固定时解析积分，避免数值阻尼掩盖真实的模型超调。"""
    veq = B * theta / C
    decay = np.exp(-C*DT)
    return x + veq*DT + (v-veq)*(-np.expm1(-C*DT))/C, veq + (v-veq)*decay


def motion(initial, seconds, kp=0.6, kd=0.3, limit=2., slew=20., delay=0, step=None):
    """位置样本按帧序延迟到达。只在到达时差分估速和更新 PD，帧间保持指令。"""
    x, v, theta, cmd, vf = initial, 0., 0., 0., 0.
    samples, trace = [], []
    prev = None
    for i in range(round(seconds/DT)+1):
        if i % 10 == 0:
            samples.append(x)
            if len(samples)>delay and step is None:
                xm = samples[-1-delay]
                if prev is not None:
                    vf = .60*vf + .40*(xm-prev)*30.
                cmd = float(np.clip(-kp*xm-kd*vf, -limit, limit))
                prev = xm
        if step is not None:
            cmd = step
        trace.append((i*DT, x, v, B*theta-C*v, cmd, theta))
        # 当前行记录时刻 i 的状态；使用步内中点倾角推进平滑限速斜坡。
        end = theta + np.clip(cmd-theta, -slew*DT, slew*DT)
        x,v = advance(x,v,(theta+end)/2.)
        theta=end
    return np.array(trace)


def curves(dest):
    cols = 'time_s x_cm v_cm_s accel_cm_s2 command_deg applied_deg'
    fig, axes = plt.subplots(3,1,sharex=True,figsize=(8,7))
    step_rows = []
    # 无斜率过渡的恒角解析解，独立核对时间、单位及积分符号。
    x, v = 0., 0.
    for _ in range(300):
        x, v = advance(x, v, 1.)
    np.testing.assert_allclose([x, v], [B/C*(1-(1-np.exp(-C))/C), B/C*(1-np.exp(-C))], atol=1e-10)
    for angle in (-2.,-1.,-.5,0.,.5,1.,2.):
        m = motion(0., 2., step=angle)
        for col, ax in enumerate(axes,1):
            ax.plot(m[:,0], m[:,col], label='%g°'%angle)
        step_rows.extend((angle,*row) for row in m[::3])
        if angle==0: assert np.allclose(m[:,1:4],0)
        if angle>0: assert np.allclose(m[:,1:],-motion(0.,2.,step=-angle)[:,1:])
    axes[0].legend(ncol=4)
    for ax,y in zip(axes,['Position (cm)','Speed (cm/s)','Acceleration (cm/s²)']):
        ax.set_ylabel(y)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Simulated beam steps from rest | open track model')
    fig.tight_layout()
    fig.savefig(dest/'angle_steps.png',dpi=135)
    plt.close(fig)
    save(dest/'angle_steps.csv','step_deg '+cols,step_rows)

    # 每个变体相对 baseline 只改一个量；正负初值分开计算。
    cases = dict(baseline={},low_kp=dict(kp=.3),low_kd=dict(kd=.08),
                 tight_limit=dict(limit=.5),slow_servo=dict(slew=3.),delay_100ms=dict(delay=3))
    traces, scores = [], []
    fig, axes=plt.subplots(2,1,sharex=True,figsize=(8,6))
    for name,changes in cases.items():
        args=dict(kp=.6,kd=.3,limit=2.,slew=20.,delay=0)
        args.update(changes)
        for x0,ax in zip((2.,-2.),axes):
            m=motion(x0,8.,**args)
            t,x=m[:,0],m[:,1]
            outside=np.flatnonzero(np.abs(x)>.1)
            settle=None if outside[-1]==len(t)-1 else t[outside[-1]+1]
            scores.append((name,x0,*[args[k] for k in ('kp','kd','limit','slew')],args['delay']/30,
                           settle,max(0.,max(-np.sign(x0)*x)),np.mean(x[t>=7.]),np.max(np.abs(x[t>=7.]))))
            traces.extend((name,x0,*row) for row in m[::3])
            assert np.max(np.abs(m[:,5]))<=args['limit']+1e-10
            assert np.max(np.abs(np.diff(m[:,5])))<=args['slew']*DT+1e-10
            ax.plot(t,x,label=name)
            if x0>0: positive=m
            else: np.testing.assert_allclose(m[:,1:],-positive[:,1:],atol=1e-10)
    for ax in axes:
        ax.axhspan(-.1,.1,color='gray',alpha=.15)
        ax.set_ylabel('Position (cm)')
    axes[0].legend(ncol=3,fontsize=8)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Simulated return to O | 30 Hz sampled PD')
    fig.tight_layout()
    fig.savefig(dest/'pd_return.png',dpi=135)
    plt.close(fig)
    save(dest/'pd_trace.csv','case x0_cm '+cols,traces)
    save(dest/'pd_metrics.csv','case x0_cm kp kd limit_deg slew_deg_s delay_s settling_s overshoot_cm final_mean_cm final_max_abs_cm',scores)


# 车体 x 向轨道正端、y 向左、z 向上。模拟安装使 IMU x 指向车体 y，IMU y 指向 -x。
BODY_FROM_IMU = np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])


def imu_ff(timestamps, accel, gyro, rest, rail_dir, rotation):
    """加速度计读比力。已知水平静止段用于估计偏置，保留向上的重力基准作分母。"""
    fb=accel @ rotation.T
    wb=gyro @ rotation.T
    fb-=fb[rest].mean(axis=0)-np.array([0.,0.,G])
    wb-=wb[rest].mean(axis=0)
    along=fb @ rail_dir
    angle=np.degrees(np.arctan2(along,fb[:,2]))
    return np.column_stack((timestamps, fb, wb, along, angle))


def imu(dest):
    t=np.arange(500)/100.
    segment=np.repeat(['calibration','stationary','accelerate','brake','pitch_up'],100)
    ax=np.select([segment=='accelerate',segment=='brake'],[.8,-.8],default=0.)
    # 最后 1 s 前半段向上俯仰 5°，后半段保持；按右手系抬头角速度沿车体 -y。
    pitch=np.clip((t-4)*10,0,5)
    rad=np.radians(pitch)
    # 世界水平加速度和重力比力旋转到车体坐标，静止倾斜时也有沿轨分量。
    specific=np.column_stack((ax*np.cos(rad)+G*np.sin(rad), np.zeros(len(t)), G*np.cos(rad)-ax*np.sin(rad)))
    omega=np.zeros_like(specific)
    omega[:,1]=-np.radians(np.where((t>=4)&(t<4.5),10.,0.))
    acc=specific @ BODY_FROM_IMU + [.03,-.05,.08]
    gyro=omega @ BODY_FROM_IMU + [.002,-.003,.001]
    trace=imu_ff(t,acc,gyro,segment=='calibration',np.array([1.,0.,0.]),BODY_FROM_IMU)
    np.testing.assert_allclose(trace[:,1:4],specific,atol=1e-12)
    np.testing.assert_allclose(trace[:,4:7],omega,atol=1e-12)
    expected=pitch+np.degrees(np.arctan(ax/G))
    np.testing.assert_allclose(trace[:,-1],expected,atol=1e-10)
    lateral=imu_ff(t,acc,gyro,segment=='calibration',np.array([0.,1.,0.]),BODY_FROM_IMU)
    np.testing.assert_allclose(lateral[:,-1],0.,atol=1e-10)
    # 正前馈使轨道 +x 端向下；转至该轨道方向后比力切向投影应为零。
    ff=np.radians(trace[:,-1])
    np.testing.assert_allclose(trace[:,-2]*np.cos(ff)-trace[:,3]*np.sin(ff),0.,atol=1e-10)
    save(dest/'imu.csv','case timestamp_s ax_m_s2 ay_m_s2 az_m_s2 gx_rad_s gy_rad_s gz_rad_s body_fx body_fy body_fup body_gx body_gy body_gz f_parallel_m_s2 theta_ff_deg expected_deg',
         ((segment[i],t[i],*acc[i],*gyro[i],*trace[i,1:],expected[i]) for i in range(len(t))))
    print('IMU final angles',[(s,round(trace[segment==s,-1][-1],4)) for s in dict.fromkeys(segment)])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=HERE/'simulation')
    dest=p.parse_args().out
    dest.mkdir(parents=True,exist_ok=True)
    curves(dest)
    imu(dest)
