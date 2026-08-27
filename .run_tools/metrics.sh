#!/usr/bin/env bash
# 内部子程序：CPU 快照。被 collect_all.sh 以 SAMPLE_SEC=<秒> TAG=<标签> 调用。
set -u
TAG="${TAG:?need TAG}"
SEC="${SAMPLE_SEC:-8}"
OUT=".run_metrics"; mkdir -p "$OUT"

cpu_line() { grep -E '^cpu ' /proc/stat; }
read_line() { set -- $1; idle=$(( $5 + $6 )); total=$(( $2+$3+$4+$5+$6+$7+$8+$9 )); echo "$idle $total"; }

A=$(cpu_line); sleep "$SEC"; B=$(cpu_line)
IA=$(read_line "$A"); IB=$(read_line "$B")
{ echo "### busy% over ${SEC}s ($(date +%T))";
  awk -v ia=$(echo $IA|cut -d' ' -f1) -v ta=$(echo $IA|cut -d' ' -f2) \
      -v ib=$(echo $IB|cut -d' ' -f1) -v tb=$(echo $IB|cut -d' ' -f2) \
      'BEGIN{printf "%.1f%%\n", 100*(tb-ib-(ta-ia))/(tb-ta)}'; } > "$OUT/${TAG}_cpu_total.txt"

python3 - "$OUT/${TAG}_cpu_cores.txt" <<'PY'
import sys, time
def snap():
    d={}
    for l in open('/proc/stat'):
        p=l.split()
        if p[0].startswith('cpu') and p[0]!='cpu':
            v=list(map(int,p[1:])); d[p[0]]=(v[3]+v[4], sum(v))
    return d
a=snap(); time.sleep(3); b=snap()
rows=[(k,100*((b[k][1]-a[k][1])-(b[k][0]-a[k][0]))/max(b[k][1]-a[k][1],1)) for k in a]
rows.sort(key=lambda r:int(r[0][3:]) if len(r[0])>3 else 99)
with open(sys.argv[1],'w') as f:
    f.write(' '.join(f"c{k[3:]}:{v:.0f}%" for k,v in rows))
    hot=sum(1 for _,v in rows if v>30); mid=sum(1 for _,v in rows if 10<v<=30)
    f.write(f"\n>30%核数:{hot}  10~30%:{mid}  <10%:{len(rows)-hot-mid}\n")
PY

PAT='gzserver|glim|containe|localiz|decision|waypoint|gimbal|fake_vel|serial_driver|spawn'
pids=$(pgrep -f "$PAT" | grep -vw $$ | tr '\n' ' ')
snap() { for p in $pids; do
    ticks=$(awk '{print $14+$15}' "/proc/$p/stat" 2>/dev/null)
    echo "$p ${ticks:-0}"
  done; }
A=$(snap); sleep "$SEC"; B=$(snap)
hz=$(getconf CLK_TCK)
{ printf "%-8s %-18s %10s %10s\n" PID COMM cpu% RSS_MB
  join -j1 <(echo "$A") <(echo "$B") | while read p a b; do
    c=$(grep -E '^(Comm|Name)' /proc/$p/status 2>/dev/null | head -1 | awk '{print $2}')
    comm_name=$(cat /proc/$p/comm 2>/dev/null)
    rss=$(awk '/VmRSS/{printf "%.0f",$2/1024}' /proc/$p/status 2>/dev/null)
    cpu=$(awk -v d=$((b-a)) -v hz=$hz -v sec=$SEC 'BEGIN{printf "%.1f",100*d/hz/sec}')
    printf "%-8s %-18s %10s %10s\n" "$p" "$comm_name" "$cpu" "${rss:-0}"
  done | sort -k3 -nr; } > "$OUT/${TAG}_procs.txt"
