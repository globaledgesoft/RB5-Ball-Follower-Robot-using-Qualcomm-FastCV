import invoke
invoke.run( "gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc -O3 -Wall -shared -fPIC src/ball_tracking.c "
    "-o lib/libball_tracking.so -I. -lpthread -lc -ldl -L. libfastcv.a "
)
