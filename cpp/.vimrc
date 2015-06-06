"nmap <F5> :wa<CR>:exec "!make && ./main"<CR>

nmap <F5> :wa<CR>:exec "!cd build && make && cd .. && ./build/dense_odometry"<CR>
