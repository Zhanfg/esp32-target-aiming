# 08_DATA

实验原始数据落在这里。§20 这个目录本身不分子目录，用文件名区分实验与日期。

每次动作保存一行，字段按规格书 §19：

```
timestamp
experiment_id
prototype_version
mechanism_version
mode
boundary_state
preload_state
membrane_id
payload_id
cycle_count
stage1_event
stage2_event
mechanism_recovered
magazine_position
magazine_index_ok
fault_code
operator_note
```

A 到 E 各级的采集记录都进这里，`experiment_id` 里带上原型级别，免得混在一起。遥测原始 CSV 可以先
留在 `07_EXPERIMENTS/` 对应级别下，整理成规范字段后再归档到这里。出图的数据从这份表读，不要在
`09_PLOTS/` 里手改数据。
