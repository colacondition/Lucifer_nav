# Waypoint CSV Layout

Decision waypoint CSV files should be saved here by map name:

```text
config/waypoints/<map_name>/center.csv
config/waypoints/<map_name>/range.csv
config/waypoints/<map_name>/home_tunnel.csv
config/waypoints/<map_name>/home_safe.csv
config/waypoints/<map_name>/wait_home.csv
```

Use absolute paths when passing them to `ros2 run decision bt_action_replacement_node`.

## 两类回家路线（决策用）

- `home_tunnel.csv`：**复活后**（20% 血、无敌 30s、虚弱）回家走这条，优先隧道快线，
  尽快到己方补给区解除虚弱。
- `home_safe.csv`：**残血但没死过**回家走这条，绕远路避开隧道/开阔过道，
  避免被对面拦截打死（死亡 -10 控制度，且复活读条递增）。

⚠️ 当前这两个文件是**占位**，只有同一个点。2027 新场地录制时
必须分别录成两条真实路线；否则两种回家行为没有区别。
