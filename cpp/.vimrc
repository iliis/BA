"nmap <F5> :wa<CR>:exec "!make && ./main"<CR>

nmap <F5> :wa<CR>:exec "!cd build/debug   && make && cd ../.. && ./build/debug/dense_odometry"<CR>
nmap <F6> :wa<CR>:exec "!cd build/release && make && cd ../.. && ./build/release/dense_odometry"<CR>
nmap <F7> :wa<CR>:exec "!cd build/debug && make && make test"<CR>
