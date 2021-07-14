import invoke
invoke.run( "gcc -shared -fPIC src/ball_tracking.c "
    "-o lib/libball_tracking.so -lfastcvopt"
)
