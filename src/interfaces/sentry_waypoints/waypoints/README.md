# Waypoint CSV Layout

Decision waypoint CSV files should be saved here by map name:

```text
config/waypoints/<map_name>/patrol.csv
config/waypoints/<map_name>/center.csv
config/waypoints/<map_name>/home.csv
config/waypoints/<map_name>/wait_home.csv
```

Use absolute paths when passing them to `ros2 run decision bt_action_replacement_node`.
