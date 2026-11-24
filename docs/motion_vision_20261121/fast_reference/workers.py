"""现场帧号子序列单工作线程与三工作线程对照；计时仅包含轨道检测入口。"""
from bench import *

def main():
    src=ROOT/'runs/moving_free'
    with (src/'measurements.csv').open() as f:
        board={int(x['frame']):x for x in csv.DictReader(f)}
    rows=[]
    for repeat in range(2):
        pipes=[pipeline(Original), pipeline(candidate.RulerReference,1), pipeline(candidate.RulerReference,3)]
        for i,ts,im,g,_ in decoded(replay(src)):
            if i not in board:continue
            row={'repeat':repeat,'frame':i,'board_mode':board[i]['registration_mode'],
                 'board_ms':board[i]['reference_and_edge_ms']}
            order=(len(rows)+repeat)%3
            for j in ((order+off)%3 for off in range(3)):
                tag=('old','one','three')[j]
                cpu=time.process_time()
                d,reg=run(pipes[j],im,g,ts,i,False)
                row[tag+'_cpu_ms']=1000*(time.process_time()-cpu)
                row[tag+'_ms']=d['ms']
                row[tag+'_mode']=d['mode']
                row[tag+'_valid']=d['valid']
                assert d['valid']==(board[i]['reference_valid']=='True')
            rows.append(row)
    with (HERE/'workers.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    for k in ('old','one','three'):
        print(k,stats([a[k+'_ms'] for a in rows]),stats([a[k+'_cpu_ms'] for a in rows]))
if __name__=='__main__':main()
