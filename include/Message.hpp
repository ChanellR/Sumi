#ifndef MESSAGE_H
#define MESSAGE_H

#include <string>

enum Client {Debug, Game, App, Emu};
enum Subject {Shutdown, Pause, Reboot, LoadRom}; 

struct Message {
    Client from;
    Client to;
    Subject subject;
    std::string contents;
};

#endif