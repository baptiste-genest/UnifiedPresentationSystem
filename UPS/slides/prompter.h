#ifndef PROMPTER_H
#define PROMPTER_H

#include "../UPS.h"
#include "../content/primitive.h"
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>


namespace UPS {

using promptTag = std::string;

class Prompter
{
public:
    Prompter(std::string script_file);
    ~Prompter() {
        //std::cout << "KILL PROMPT " << prompt_pid << std::endl;
        system("pkill xterm");
        //pkill(prompt_pid, SIGTERM);
    }

    void write(promptTag tag) const;
    void loadScript();

private:
    pid_t prompt_pid;
    std::string script_file;
    std::string current_tag;
    std::map<std::string,std::string> scripts;
};

}

#endif // PROMPTER_H
