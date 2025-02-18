SHLINK=$(readlink -f "$0")
SHPATH=$(dirname "$SHLINK")
BASE_PATH=$SHPATH

echo $BASE_PATH

source $BASE_PATH/devel/setup.bash
export LD_LIBRARY_PATH=$BASE_PATH/src/async_stream/lib:$LD_LIBRARY_PATH

echo $LD_LIBRARY_PATH

roslaunch async_stream osi_grpc.launch

trap killgroup SIGINT

killgroup(){
	echo "Killing ..."
	pkill -f async_streaming
    pkill -f osi_bridge_main
}