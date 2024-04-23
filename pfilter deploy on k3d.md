## 实验
### 实验流程
#### 1. 创建k3d集群
`k3d cluster create lstmpftest --agents 8 --network k3d`
#### 2. LSTM deployment
`kubectl apply -f uwb_lstm_deployment.yaml`
#### 3. Rosbag deployment
- 部署
`kubectl apply -f rosbag_play_record_deployment.yaml`
- 进入play pod开始play rosbag
```
kubectl exec -it play-pod-name -- /bin/bash
#pod 内
source /opt/ros/galactic/setup.bash
ros2 bag play 
```
- 进入record pod开始record rosbag
```
kubectl exec -it record-pod-name -- /bin/bash
#pod 内
source /opt/ros/galactic/setup.bash
ros2 bag record -o k3d_result -a 
```
#### 4. pfilter deployment
`kubectl apply -f pf_ros2_multi_ulv_deployment.yaml`

**注意修改containers中的image名字**
```yaml
      containers:
        - name: pf-ros2
          # image: 192.168.193.113:5000/pf_ros2_multi_ulv:ros2core
          image: 192.168.193.113:5000/pf_ros2_multi_ulv:ros2core_nolstm
```

#### 5. 重启lstm
- 查询pod信息，获得pod的name：`kubectl get pods`
- 手动删除pod，使k8s拉起新pod：`kubectl delete pod $name1 $name2 $name3 ...`

#### 6. 停止rosbag录制，复制rosbag
- 在record pod停止record （ctrl+c）
- 将rosbag复制出来
`kubectl cp record-rosbag-deployment-845bf6cdb6-6r96j:k3d_result/ ./rosbag/k3d_result/`

#### 7. 停止LSTM和pf服务
- 查询deployment信息，获得deployment的name：`kubectl get deployment`
- 删除deployment：`kubectl delete deployment $name1 $name2`

### 实验内容
暂时先完成1个replica的实验
1. 无lstm
2. 全lstm
3. 1 lstm down
4. 2 lstm down
5. 3 lstm down
6. 4 lstm down
7. 