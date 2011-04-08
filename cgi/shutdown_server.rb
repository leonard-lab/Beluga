#!/usr/bin/ruby

require 'socket'

socket = TCPSocket.open('127.0.0.1',1234)

resp = socket.gets

if(resp)
    socket.puts "S"
    puts socket.gets 
end
socket.close

