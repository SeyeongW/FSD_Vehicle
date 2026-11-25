# 외부 저장소 로컬 수정분

`src/livox_ros_driver2`, `src/livox_laser_simulation_RO2`는 외부 저장소(gitlink)라 이 저장소에 직접 커밋되지 않는다.
로컬에서 수정한 내용을 patch로 보관한다.

```bash
git -C src/livox_ros_driver2 apply ../../patches/livox_ros_driver2_local.patch
git -C src/livox_laser_simulation_RO2 apply ../../patches/livox_laser_simulation_RO2_local.patch
```
