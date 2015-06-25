"nmap <F5> :wa<CR>:exec "!make && ./main"<CR>

nmap <F5> :wa<CR>:exec "!cd build/debug   && make -j4 dense_odometry && cd ../.. && xfce4-terminal --geometry=106x30 -x ./build/debug/dense_odometry"<CR>
nmap <F6> :wa<CR>:exec "!cd build/release && make -j4 dense_odometry && cd ../.. && xfce4-terminal --geometry=106x30 -x ./build/release/dense_odometry"<CR>
nmap <F7> :wa<CR>:exec "!cd build/debug && make -j4 all && make test"<CR>

command InitCmake :exec "!rm -rf build
            \ && mkdir build build/debug build/release
            \ && cd build/debug
            \ && cmake -D CMAKE_BUILD_TYPE=Debug ../..
            \ && cd ../release
            \ && cmake -D CMAKE_BUILD_TYPE=Release ../.."
