import invoke
invoke.run( "gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++ -O3 -Wall -shared -std=c++11 -fPIC ball_tracking.cpp "
    "-o libball_tracking.so -lpthread -lc -ldl -L. libfastcv.a "
)
