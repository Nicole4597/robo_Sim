## scripts
This is a set of helper scripts to launch single nodes without the need to source ```/opt/ros``` and ```devel/``` every time

## usage

make all the scripts executable:
    
    chmod +x ./*.sh

execute the script related to each node, in order to launch the single node

|   node                |   script                      |
|-----------------------|-------------------------------|
|   main_controller     |   ```./main_controller.sh```  |
|   triskarone_slam     |   ```./triskarone_slam.sh```  |
|   move_base           |   ```./move_base.sh```        |
|   ai_server           |   ```./ai_server.sh```        |
|   bm_server           |   ```./bm_server.sh```        |
|   reaction_server     |   ```./reaction_server.sh```  |
|   rqt_reconfigure     |   ```./rqt_reconfigure.sh```  |
