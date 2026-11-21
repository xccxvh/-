from replay import *
def sheet(name,tiles,columns):
    while len(tiles)%columns:tiles.append(np.zeros_like(tiles[0]))
    img=np.vstack([np.hstack(tiles[i:i+columns]) for i in range(0,len(tiles),columns)])
    assert cv2.imwrite(str(dst/name),img)
def gap():
    allrows={int(r['frame']):r for r in csv.DictReader((dst/'live_pixel_all.csv').open())}
    sampled={int(r['frame']):r for r in csv.DictReader((dst/'live_pixel_sampled.csv').open())}
    tiles=[];later=[];records=[]
    for i,t,j in stream('live_pixel'):
        if not (482<=i<=504 or 662<=i<=679):continue
        f=cv2.imdecode(j,1)
        r=allrows[i];s=sampled.get(i,{})
        if i<=504:
            # 逐帧原图来自和采集脚本相同的 imdecode，以绝对像素标尺供独立目测。
            u,v=200,520
            crop=f[v:652,u:470].copy()
            for x in range(220,470,20):
                cv2.line(crop,(x-u,0),(x-u,6),(255,255,255),1)
                if x%40==0:cv2.putText(crop,str(x),(x-u-12,18),0,.32,(255,255,255),1)
            left=round(float(r['left']))-u
            cv2.line(crop,(left,22),(left,122),(255,175,0),1)
            z=cv2.copyMakeBorder(crop,0,26,0,0,cv2.BORDER_CONSTANT)
            cv2.putText(z,str(i)+' '+s.get('logged_detector','skipped'),(3,149),0,.48,(255,255,255),1)
            tiles.append(z)
            records.append(dict(frame=i,relative_s=round(t-float(allrows[482]['read_s']),6),
                                left_px=r['left'],logged=s.get('logged_detector','skipped'),
                                old_all_xy=r['old_xy'],new_all_xy=r['new_xy'],
                                old_sampled_tracker=s.get('old_track',''),new_sampled_tracker=s.get('new_track',''),
                                new_sampled_valid=s.get('new_valid','')))
        else:
            crop=f[505:675,180:690]
            z=cv2.copyMakeBorder(crop,0,26,0,0,cv2.BORDER_CONSTANT)
            cv2.putText(z,str(i)+' '+r['rail']+' '+r['new_track'],(4,188),0,.46,(255,255,255),1)
            later.append(z)
    write_csv(dst/'gap_482_504.csv',records)
    sheet('gap_detail.jpg',tiles,4)
    sheet('remaining_662_679.jpg',later,3)
def negative():
    file=base.parents[1]/'6/topview_20261120/circle_test/tool_only.mkv'
    config=json.loads((jet/'config_topview_circle_20261120.json').read_text())
    ref=config['rail']['upper_reference']
    ref['image']=str(jet/ref['image'])
    cap=cv2.VideoCapture(str(file))
    rd=RailDetector(config['rail'],1280,720)
    bd=BallDetector(dict(config['ball'],ball_search=config['ball_search']),1280,720)
    rows=[];tiles=[]
    while True:
        ok,f=cap.read()
        if not ok:break
        i=len(rows);g=cv2.cvtColor(f,cv2.COLOR_BGR2GRAY);r=rd.detect(g,force=True)
        row=dict(frame=i,rail=r.reason)
        for tag,fn in [('old',original.topview_balls),('new',current)]:
            lv.topview_balls=fn
            c=bd.detect(f,g,r)
            row[tag+'_xy']=json.dumps([[b.x,b.extra['measurement_y_px']] for b in c])
        rows.append(row)
        if i in (0,60,119):
            tile=cv2.resize(f[425:620,140:1150],(656,127))
            cv2.putText(tile,str(i)+' old/new '+row['old_xy']+' / '+row['new_xy'],(5,20),0,.5,(20,255,255),1)
            tiles.append(tile)
    cap.release()
    write_csv(dst/'tool_only.csv',rows)
    sheet('tool_only_control.jpg',tiles,1)
    print('negative',len(rows),sum(r['new_xy']!='[]' for r in rows))
if __name__=='__main__':
    gap()
    negative()
