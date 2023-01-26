IMAGE_NAME="hyped_ros"
CONTAINER_NAME="hyped_ros"

if [ "$1" = "help" ] || [ -z "$1" ]; then
  echo "Usage: ./build.sh [container|pod]"
  echo "       container - run a ros2 shell in a container"
  echo "       pod       - run all configured containers"
  echo "       rebuild   - rebuild the docker image"
  exit 0
fi

if ! [ -x "$(command -v docker)" ]; then
  echo '[!] Error: docker is not installed.' >&2
  exit 1
fi

if ! [ -x "$(command -v docker-compose)" ]; then
  echo '[!] Error: docker-compose is not installed.' >&2
  exit 1
fi

image=$( docker images -q $IMAGE_NAME 2> /dev/null )
if [[ -z ${image} ]]; then
  echo "[!] Building image"
  docker build -t $IMAGE_NAME .
else 
  echo "[>] Image already built"
fi

if [ "$1" = "container" ]; then
  echo "[>] Running container"
  container=$( docker ps -a -f name=$CONTAINER_NAME | grep $CONTAINER_NAME 2> /dev/null )
  if [[ ! -z ${container} ]]; then 
    echo "[!] $CONTAINER_NAME container exists"
    if [ "$( docker container inspect -f '{{.State.Running}}' $CONTAINER_NAME )" == "true" ]; then
      echo "  - Container is already running"
      echo "  - Attaching to container"
      docker exec -it $CONTAINER_NAME bash
    else
      echo "  - Container is not running"
      echo "  - Starting container"
      docker start $CONTAINER_NAME > /dev/null
      echo "    - Attaching to container"
      docker exec -it $CONTAINER_NAME bash
    fi
  else
    echo "[!] $CONTAINER_NAME container does not exist"
    docker run -it -v $(pwd):/home/hyped -v /dev:/dev --name $CONTAINER_NAME $IMAGE_NAME bash
  fi
elif [ "$1" = "pod" ]; then
  echo "[>] Running pod"
  docker-compose up
elif [ "$1" = "rebuild" ]; then
  echo "[>] Rebuilding image"
  docker build -t $IMAGE_NAME .
else
  echo "[!] Invalid argument"
fi


