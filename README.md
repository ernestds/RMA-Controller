## Resolved Motion Acceleration Controller
 ROS Package for RMA Controller

 Instruções para uso do pacote rma_controller

- Deve-se adcionar o arquivo rma_controller.yaml ao pacote wam_bring_up
- Para dar launch no controlador:

	roslaunch roslaunch wam_bringup gazebo.launch controller:=rma_controller

- launch do nodo de referencias

	roslaunch pose_ref pose_ref.launch

- Para enviar referencias para este nodo pode-se usar

	rostopic pub -1 /reference pose_ref/PoseRPY "{x: ,y: ,z: ,roll: ,pitch: ,yaw: }"

- ou algum dos scripts já presentes em seu pacote com diversos pontos, por exemplo

	rosrun pose_ref cart_step_near.sh

- pode-se ainda utilizar o nodo simple_traj, porém esta ainda está em desenvolvimento. Neste caso existe um arquivo yaml na pasta config do pacote em que podem ser configurados pontos. Depois é necessário alterar o código para que estes pontos sejam incluídos na trajetória (mudar o array que guarda pontos da trajetória)

## Estrutura de diretórios

``` bash
rma_controller/
├── CMakeLists.txt
├── include
│   └── rma_controller
│       └── rma_controller.h
├── log_todo.txt
├── package.xml
├── PontosTeste.txt
├── README.md
├── rma_controller_plugins.xml
└── src
    └── rma_controller.cpp


pose_ref/
├── CMakeLists.txt
├── config
│   └── points.yaml
├── include
│   └── pose_ref
├── launch
│   ├── pose_ref.launch
│   └── simple_traj.launch
├── msg
│   └── PoseRPY.msg
├── package.xml
├── scripts
│   ├── cart_step_p10.sh

```

