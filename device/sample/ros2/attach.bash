CID=`sudo docker ps | grep foxy | awk '{print $1}'`
sudo docker exec -it ${CID} bash

