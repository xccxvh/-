from replay import *
# 合并两种时序的候选/测量变化，每个位置以原片 96 px 方窗单独展示。
# 黄线仅在窗边标出候选的横纵坐标，便于看清原图球心与高光。
for n in ('baseline','live_pixel'):
    tables=[list(csv.DictReader((dst/(n+'_all.csv')).open()))]
    if n=='live_pixel':tables.append(list(csv.DictReader((dst/(n+'_sampled.csv')).open())))
    marks={}
    for rows in tables:
        for r in rows:
            i=int(r['frame'])
            labels=[]
            if r['new_xy']!=r['old_xy']:
                labels.extend(('C',x,y) for x,y in json.loads(r['new_xy']))
            if r['old_valid']!=r['new_valid']:
                k='new' if r['new_valid']=='1' else 'old'
                labels.append(('V' if k=='new' else 'L',float(r[k+'_x']),float(r[k+'_y'])))
            if i%89==0 and r['old_xy']==r['new_xy']:
                labels.extend(('B',x,y) for x,y in json.loads(r['old_xy']))
            for tag,x,y in labels:
                marks.setdefault(i,{}).setdefault((round(x),round(y)),set()).add(tag)
    tiles=[];index=[]
    for i,ts,jpeg in stream(n):
        if i not in marks:continue
        f=cv2.imdecode(jpeg,1)
        for (x,y),tags in marks[i].items():
            tile=f[y-48:y+48,x-48:x+48].copy()
            cv2.line(tile,(48,0),(48,11),(0,255,255),1)
            cv2.line(tile,(0,48),(11,48),(0,255,255),1)
            tile=cv2.copyMakeBorder(tile,18,0,0,0,cv2.BORDER_CONSTANT)
            flags=''.join(sorted(tags))
            cv2.putText(tile,str(i)+' '+flags,(2,14),0,.45,(255,255,255),1)
            index.append(dict(frame=i,x=x,y=y,flags=flags,sheet=1+len(tiles)//80,tile=1+len(tiles)%80))
            tiles.append(tile)
    write_csv(dst/(n+'_review_index.csv'),index)
    for k in range(0,len(tiles),80):
        page=tiles[k:k+80]
        while len(page)%10:page.append(np.zeros_like(tiles[0]))
        im=np.vstack([np.hstack(page[t:t+10]) for t in range(0,len(page),10)])
        assert cv2.imwrite(str(dst/(n+'_review_'+str(1+k//80)+'.jpg')),im)
    print(n,len(tiles))
